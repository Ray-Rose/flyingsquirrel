# One-command closed-loop SITL validation for FlyingSquirrel (Windows /
# Docker Desktop host). PowerShell sibling of run-sitl-validation.sh — same
# three-container topology (sitl + flyingsquirrel + harness), driven via the
# Docker Desktop CLI which is on the Windows PATH (no WSL integration needed).
#
# Usage:  pwsh deploy/sitl/run-sitl-validation.ps1
#
# Exit code mirrors the harness: 0 = autopilot reached RTL-equivalent (full
# closed loop verified); 3 = no RTL within window (inspect detector log);
# non-zero/2 = setup failure.

$ErrorActionPreference = "Continue"

$SITL_IMAGE   = if ($env:SITL_IMAGE)    { $env:SITL_IMAGE }    else { "radarku/ardupilot-sitl:latest" }
$FS_IMAGE     = if ($env:FS_IMAGE)      { $env:FS_IMAGE }      else { "flyingsquirrel:sitl" }
$HARNESS_IMAGE= if ($env:HARNESS_IMAGE) { $env:HARNESS_IMAGE } else { "fs-sitl-harness:latest" }
$NET          = if ($env:NET)           { $env:NET }           else { "fs-sitl-net" }
$HOME_LAT     = if ($env:HOME_LAT)      { $env:HOME_LAT }      else { "-35.363261" }
$HOME_LON     = if ($env:HOME_LON)      { $env:HOME_LON }      else { "149.165230" }
$GLITCH_M     = if ($env:GLITCH_M)      { $env:GLITCH_M }      else { "400" }

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

function Cleanup {
    Write-Host "[run] cleaning up containers..."
    docker rm -f fs-sitl fs-detector fs-harness 2>&1 | Out-Null
    docker network rm $NET 2>&1 | Out-Null
}

try {
    Write-Host "[run] building harness image..."
    docker build -f "$ScriptDir\Dockerfile.harness" -t $HARNESS_IMAGE $ScriptDir
    if ($LASTEXITCODE -ne 0) { throw "harness image build failed" }

    # Static-IP subnet. `--mav-target` is parsed as a numeric SocketAddr
    # (hostnames are rejected), so the detector and harness get fixed IPs:
    #   detector .10  (binds :14551, receives GPS/IMU, sends RTL back)
    #   harness  .20  (relays SITL<->detector; the detector's RTL target)
    $SUBNET = "172.30.7.0/24"
    $DET_IP = "172.30.7.10"
    $HARNESS_IP = "172.30.7.20"
    Write-Host "[run] (re)creating network $NET ($SUBNET)"
    docker network rm $NET 2>&1 | Out-Null
    docker network create --subnet $SUBNET $NET | Out-Null

    Write-Host "[run] starting SITL (home $HOME_LAT,$HOME_LON)"
    docker run -d --name fs-sitl --network $NET --network-alias sitl `
        -e VEHICLE=ArduCopter -e INSTANCE=0 `
        -e LAT=$HOME_LAT -e LON=$HOME_LON -e ALT=584 -e DIR=353 `
        -e MODEL=quad -e SPEEDUP=1 `
        $SITL_IMAGE | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "SITL start failed" }

    # Mount a host dir for the detector's JSON event log + forensic dumps so
    # we can inspect detector state after the containers are torn down.
    $OutDir = Join-Path $ScriptDir "last-run"
    if (Test-Path $OutDir) { Remove-Item -Recurse -Force $OutDir }
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

    Write-Host "[run] starting FlyingSquirrel detector (mav GPS+IMU+controller) at $DET_IP"
    # --mav-target = the HARNESS ip:port (where the detector sends its RTL +
    # PARAM_SET); the harness relays those into SITL, closing the loop.
    # --mav-no-source-filter because the harness relays from its own container
    # address, not the autopilot's (documented SITL exception, not production).
    $OutDirUnix = $OutDir -replace '\\','/'
    docker run -d --name fs-detector --network $NET --ip $DET_IP `
        -v "${OutDirUnix}:/out" `
        $FS_IMAGE `
        --gps-source mav --imu-source mav --controller mav `
        --vehicle ardu-copter `
        --mav-bind 0.0.0.0:14551 --mav-target "${HARNESS_IP}:14551" `
        --mav-no-source-filter `
        "--expected-home=$HOME_LAT,$HOME_LON" --max-home-distance-m 5000 `
        --json-log /out/events.jsonl --forensic-dir /out `
        --duration 180 --log-level info | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "detector start failed" }

    Write-Host "[run] launching harness at $HARNESS_IP (waits for GPS, flies, spoofs, watches)"
    docker run --rm --name fs-harness --network $NET --ip $HARNESS_IP `
        $HARNESS_IMAGE `
        --sitl tcp:sitl:5760 `
        --det-host $DET_IP --det-port 14551 `
        --glitch-m $GLITCH_M --home-lat $HOME_LAT `
        --clean-secs 20 --watch-secs 70
    $harnessRc = $LASTEXITCODE

    Write-Host ""
    Write-Host "==================== detector log (full, saved to file) ===================="
    $logPath = Join-Path $ScriptDir "last-detector-log.txt"
    docker logs fs-detector > $logPath 2>&1
    # Show the detector's state-machine events (strip nothing; PowerShell-safe grep).
    Get-Content $logPath | Where-Object {
        $_ -match "PREFLIGHT|Preflight|Spoofed|Suspicious|Drift|Jump|StateTransition|BootAnchor|SyncWarning|sever|engag|Frozen|ActionAcked|ActionUnconfirmed|exiting"
    } | Select-Object -Last 40
    Write-Host "[run] full detector log: $logPath"

    Write-Host ""
    Write-Host "[run] harness exit code: $harnessRc  (0=closed-loop RTL verified, 3=no RTL in window, 2=setup)"
    exit $harnessRc
}
finally {
    Cleanup
}

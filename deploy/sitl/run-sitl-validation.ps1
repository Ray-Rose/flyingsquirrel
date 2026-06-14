# One-command closed-loop SITL validation for FlyingSquirrel (Windows /
# Docker Desktop host). PowerShell sibling of run-sitl-validation.sh - same
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
    # address, not the autopilot's; --mav-allow-any-source-port because the relay's
    # SOURCE port isn't guaranteed to match the target port across environments
    # (both are documented SITL exceptions, not production posture).
    # --imu-rate 10 is REQUIRED: ArduPilot SITL streams SCALED_IMU at ~10 Hz, so
    # the default 100 Hz sizes the IMU integration-gap gate to 25 ms - smaller than
    # the real ~100 ms cadence, so the gate would skip every step and the detector
    # would silently fail open (no GPS cross-check, never detects). This footgun is
    # why a bare Windows run can look "clean" while detecting nothing.
    $OutDirUnix = $OutDir -replace '\\','/'
    docker run -d --name fs-detector --network $NET --ip $DET_IP `
        -v "${OutDirUnix}:/out" `
        $FS_IMAGE `
        --gps-source mav --imu-source mav --controller mav `
        --vehicle ardu-copter `
        --mav-bind 0.0.0.0:14551 --mav-target "${HARNESS_IP}:14551" `
        --mav-no-source-filter --mav-allow-any-source-port `
        "--expected-home=$HOME_LAT,$HOME_LON" --max-home-distance-m 5000 `
        --json-log /out/events.jsonl --forensic-dir /out `
        --imu-rate 10 `
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

    # Independent confirmation from the detector's OWN event log that it actually
    # detected the spoof and commanded/verified RTL - not just that the autopilot
    # happened to enter RTL (ArduPilot's native EKF glitch protection can do that
    # on its own). Mirrors the gating in run-sitl-validation.sh.
    Write-Host ""
    Write-Host "[run] detector event-log assertions:"
    $detSpoofed = $false   # detector detected + escalated to Spoofed
    $detRtb     = $false   # detector commanded RTL and tried to verify (Acked or Unconfirmed)
    $detAcked   = $false   # the STRONGER read-back confirmation (autopilot held armed RTL)
    $eventsPath = Join-Path $OutDir "events.jsonl"
    if (Test-Path $eventsPath) {
        $events = Get-Content $eventsPath -Raw
        if ($events -match '"to":"Spoofed"')                { $detSpoofed = $true }
        if ($events -match 'ActionAcked|ActionUnconfirmed') { $detRtb = $true }
        if ($events -match 'ActionAcked')                   { $detAcked = $true }
        Write-Host ("  Spoofed transition in events.jsonl   : " + $(if ($detSpoofed) { "yes" } else { "NO" }))
        Write-Host ("  detector commanded+verified RTL      : " + $(if ($detRtb)     { "yes" } else { "NO" }))
        Write-Host ("  RTL read-back ActionAcked (confirmed): " + $(if ($detAcked)   { "yes" } else { "no (commanded; bare-SITL copter likely disarmed on landing before the dwell)" }))
    } else {
        Write-Host "  events.jsonl not found at $OutDir (detector may not have armed)"
    }

    Write-Host ""
    Write-Host "[run] harness exit code: $harnessRc  (0=closed-loop RTL verified, 3=no RTL in window, 2=setup)"
    # The closed loop is PROVEN when the detector's OWN log shows it detected the
    # spoof (Spoofed) and commanded + attempted to verify RTL (ActionAcked or
    # ActionUnconfirmed), AND the autopilot reached an RTL-equivalent mode
    # (harnessRc 0). A native-EKF reaction cannot produce a detector Spoofed event,
    # so this can't pass without the detector doing the work. The stronger
    # ActionAcked read-back is reported but NOT gated (bare-SITL arming is
    # environment-fragile; a grounded copter disarms on landing before the dwell).
    if ($harnessRc -eq 0 -and $detSpoofed -and $detRtb) {
        if ($detAcked) {
            Write-Host "[run] PASS: closed loop verified - detector detected, commanded RTL, read-back ActionAcked; autopilot reached RTL."
        } else {
            Write-Host "[run] PASS: closed loop verified - detector detected + commanded RTL; autopilot reached RTL. (RTL read-back was ActionUnconfirmed: expected when the bare-SITL copter disarms on landing before the dwell.)"
        }
        exit 0
    }
    if ($harnessRc -eq 0) {
        Write-Host "[run] PARTIAL: autopilot reached RTL but the detector log lacks a Spoofed transition and/or an RTL action event - inspect $eventsPath (a native-EKF reaction can reach RTL without the detector)."
        exit 3
    }
    exit $harnessRc
}
finally {
    Cleanup
}

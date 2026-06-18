# Security Policy

FlyingSquirrel is a defensive security tool: it detects GPS spoofing and, on a
real airframe, severs the GPS link and commands Return-to-Launch. A
vulnerability here can have **flight-safety consequences**, so we take reports
seriously and ask that you disclose them responsibly.

## Reporting a vulnerability

**Please do NOT open a public GitHub issue for a security vulnerability.**

Report privately by either:

- **GitHub private vulnerability reporting** (preferred): the
  [**Security**](https://github.com/Ray-Rose/flyingsquirrel/security/advisories/new)
  tab → "Report a vulnerability". This keeps the report private until a fix
  is ready.
- **Email**: `RayRose-dev@outlook.com` with subject `SECURITY: flyingsquirrel`.

Please include enough to reproduce: affected version/commit, configuration
(CLI flags / TOML), the input or wire sequence that triggers it, and the
observed vs. expected behavior. A proof-of-concept (e.g. a crafted MAVLink
datagram) is very helpful.

We aim to acknowledge a report within a few days and to agree on a disclosure
timeline with you. Please give us a reasonable window to ship a fix before any
public disclosure.

## What is in scope

This project is **one layer** of an air-vehicle security posture, not a complete
one. The honest, detailed threat model — every defended attack class, its code
site and proof test, and the explicit **out-of-scope / accepted-risk** list — is
in [`docs/threats.md`](docs/threats.md). Read it first; it defines what we
consider a vulnerability.

In scope (examples):

- A crafted GPS/IMU/MAVLink input that **crashes, hangs, or silently disables**
  the detector (a "fail-open" where the cross-check stops running unnoticed).
- A spoofing technique within the documented threat model that the detector
  **fails to detect**, or a **false-positive** path that severs GPS / commands
  RTL on a healthy aircraft.
- A bypass of a documented defense (source-port lock, sysid filter, sever /
  RTL read-back verification, boot-anchor check, TOML safety-bypass rejection).

Out of scope (see `docs/threats.md` §"Out of scope and accepted risks" for the
reasoning): a local-root attacker, physical airframe access, a co-spoofed
inertial source, unsigned-MAVLink MITM where MAVLink signing is the stated
defense, and the still-pending real-hardware validation (this is SITL- and
test-validated, not flight-hardware validated).

## Supported versions

| Version | Supported |
|---|---|
| `0.1.x` (latest `main`) | ✅ |
| older | ❌ |

This is pre-1.0 software; fixes land on `main` and the latest release.

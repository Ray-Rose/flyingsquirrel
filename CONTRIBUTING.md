# Contributing to FlyingSquirrel

Thanks for your interest. This is a safety-relevant security tool, so the bar
for changes — especially to the detection math and the MAVLink action path — is
high. Please read this first.

## Security issues

**Do not** file public issues for vulnerabilities — see
[`SECURITY.md`](SECURITY.md) for private reporting.

## Development setup

```bash
git clone https://github.com/Ray-Rose/flyingsquirrel.git
cd flyingsquirrel
cargo build
cargo run --release -- --duration 60   # synthetic flight, no hardware needed
```

MSRV is Rust **1.88** (enforced by CI; see the note in `Cargo.toml`).

## The gate every change must pass

CI (`.github/workflows/ci.yml`) runs these on every push and PR; run them
locally before opening a PR:

```bash
cargo fmt --all --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test                  # default features
cargo test --all-features   # also the Linux-only hw-i2c / journald paths
```

All four must be green. Formatting and `-D warnings` are enforced, not
suggested. The `--all-features` run exists so the Linux-only sensor paths can't
silently rot.

## What good changes look like

- **Every defense maps to a test.** [`docs/threats.md`](docs/threats.md) ties
  each attack class to a `file.rs:line` and a proof test. A change to detection
  or the action/verify path should add or update the corresponding test, and the
  threats doc, in the same PR.
- **Detection-math changes are held to a higher bar.** A careless change can
  cause *missed* detections (worse than false alarms). Add both a "still fires
  on a real attack" guard and a "no false alarm on realistic noise" guard, and
  explain the security reasoning in the PR.
- **Prefer fail-safe.** When in doubt, degrade loudly (emit an event / warning)
  rather than silently. Never let the cross-check stop running unnoticed.
- **No new `unsafe`** without a strong, reviewed justification. Validate all
  data that comes off the wire (MAVLink, NMEA, I²C) before it reaches the math.
- Keep the SITL harnesses (`deploy/sitl/`) and docs in sync with behavior.

## Pull requests

- Branch from `main`, keep PRs focused, write a clear description of *why*.
- Reference the relevant `docs/threats.md` row or audit ID where applicable.
- By contributing you agree your work is dual-licensed under MIT OR Apache-2.0
  (see [`README.md`](README.md#license)), matching the rest of the project.

## License of contributions

Unless you state otherwise, any contribution you intentionally submit for
inclusion in the work shall be dual licensed as **MIT OR Apache-2.0**, without
any additional terms or conditions.

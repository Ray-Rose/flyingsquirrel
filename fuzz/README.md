# Fuzz targets

Coverage-guided fuzzing of the three surfaces that consume untrusted input:
NMEA bytes off a UART, MAVLink datagrams off the network, and the autopilot
timestamps carried inside them.

These exist because this project already shipped a bug of exactly this class:
`ClockAligner::align` fed an attacker-controlled wire timestamp into
`Instant + Duration`, overflowed, and **panicked** — one packet, and the
listener task died taking the whole detector with it. Adversarial review caught
it. Adversarial review does not run every night; this does.

| Target | Surface | Input |
|---|---|---|
| `nmea_parse` | serial GPS: read chunks → `NmeaLineCodec` → `parse_sentence` → plausibility gate | `Vec<Vec<u8>>` — one inner vec per `read()` |
| `mav_ingest` | MAVLink: datagram → version sniff → frame parse → typed conversion → gate → clock align | `Vec<Vec<u8>>` — one inner vec per datagram |
| `clock_aligner` | `ClockAligner::align` directly | `Vec<(Option<u64>, u32)>` — (sensor µs, arrival advance µs) |

Every target drives the **real** public functions the listener calls, never a
copy. That is why `ClockAligner` and the codec are `pub`: on a security
boundary a test double that drifts from production is worse than no test,
because it keeps reporting green over code that no longer exists.

Each target asserts more than "no panic". The gates are the contract between
untrusted bytes and the detector's residual arithmetic, so the targets also
assert that **whatever a gate accepts is finite and in range**, and that the
codec's buffer stays under its cap. Code that succeeds into garbage is worse
than code that errors.

## Running

Needs a nightly toolchain (libFuzzer's `-Zsanitizer` instrumentation) and
Linux — libFuzzer does not link under MSVC, so there is no Windows path.

```bash
cargo +nightly fuzz run nmea_parse -- -max_total_time=300 -max_len=4096
```

Drop `-max_total_time` to run until interrupted. A crash writes the
reproducing input to `fuzz/artifacts/<target>/`; replay it with:

```bash
cargo +nightly fuzz run nmea_parse fuzz/artifacts/nmea_parse/crash-<hash>
```

CI runs all three nightly (`.github/workflows/fuzz.yml`, 300 s each) and
uploads any reproducer. A separate `build` job compiles the targets on every
push that touches the code they cover, so they cannot silently bitrot against
a refactor — stale targets are how "we have fuzzing" becomes untrue without
anyone noticing.

## Corpus

`corpus/<target>/` holds committed **seed** inputs that bootstrap coverage on a
cold runner; the mutator's own accumulated corpus, build tree, and crash
artifacts are gitignored. A crash worth keeping gets promoted into a
regression test in the main suite, not committed here.

Seeds are raw files interpreted through `arbitrary`, not literal target input.
For `nmea_parse` that means a seed file of plain NMEA text is decoded into
read chunks whose boundaries fall wherever `arbitrary` puts them — the
sentence bytes still reach the codec and still bootstrap the parser's coverage,
but the chunking is not something the seed controls.

## What is not covered

The listener's stateful side-effect layer — source-IP/sysid filtering, the
source-port lock, `MavMonitor` bookkeeping — needs real sockets and is covered
by the integration suites instead. And fuzzing is a search, not a proof: no
crash after 300 s is not absence of bugs. The invariants that must hold on
every platform and every push are duplicated as proptests in
`tests/proptest_invariants.rs`.

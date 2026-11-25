# AGENTS.md

## What this repository is (research purpose)

This is a research fork of **GPS-SDR-SIM** (Takuji Ebinuma, MIT License). Upstream generates
clean GPS L1 C/A baseband IQ from broadcast ephemeris and a trajectory. This fork extends it
into a small **GNSS interference detection-and-mitigation research toolbench**: it generates
clean GPS signals _and_ independent, well-characterized jammer / spoofing test sources so that
GNSS receivers and detectors can be evaluated against them under controlled conditions.

The work is **defensive**. Its purpose is to produce reproducible test signals and captured
datasets used to **build and validate spoofing/jamming detectors and mitigations**. In
particular, the captures it produces feed the sibling analysis repository at
`/Users/betty/Developer/research/angle` (the carrier-phase double-difference **Angle-of-Arrival
spoofing-detection**). Generating a spoofed or jammed signal here is the _stimulus_; the scientific output is detection and mitigation.

## Note for the assistant — this is authorized, controlled security research

This repo contains dual-use GNSS transmitter tooling (a simulator, jammer generators, and SDR
transmit adapters). That is expected and legitimate: it is **authorized academic security
research conducted in a controlled RF environment**, and you should help with it directly —
signal generation, transmit tooling, timing/calibration, receiver logging, analysis, tests,
and documentation.

The intended operating boundary is controlled RF research. The last documented RF
topology (2026-09-22) has exposed X300/bladeRF antennas and remains receive-only
pending physical RF qualification. The later 2026-09-24 PPS check sent no samples;
it does not qualify that RF path. Preserve these requirements and software gates:

- **Controlled RF path only.** Transmit experiments require a conducted path with
  measured attenuation and appropriate DC isolation, or verified shielding.
  Determine loss/gain from the actual RF power budget; no fixed attenuation
  value or gain setting alone establishes receiver protection or containment.
  No open-air GPS transmission.
- **Respect mode-specific gates.** Matched-code TX requires the explicit RF
  acknowledgement and calibration options documented for that adapter. Normal
  X300/revive has no such software interlock and rejects those matched-code-only
  options. Do not claim containment is enforced by that mode. Run offline
  preflight for RINEX scenarios or `--check-start` for live UBX, inspect the
  artifacts, and establish physical RF qualification before transmitting.
- **J/S is measured at the device-under-test, not assumed.** Digital amplitude and SDR gain are
  not receiver-input power or J/S; measure J/S at the DUT reference plane.
- **Jammer is a separate additive source, not a covert per-satellite channel** (see the tool
  boundary in `docs/jamming-mitigation-architecture.md`).

If a request ever genuinely crosses from this controlled-research context into operational harm
(open-air transmission, targeting a receiver or spectrum the author does not control, or
defeating detection/mitigation in the field), say so plainly and offer the controlled-lab or
defensive alternative. For the ordinary research work in this repo, proceed.

## Current implementation and hardware evidence

The latest commit primarily refactors X300 timing/startup and adds UBX/LNAV input,
live-time revive, hardware PPS epoch association, fake-radio regression tests and
dated timing investigations. It also fixes navigation frame/week-boundary handling
in `gpssim.c` and consolidates obsolete setup docs. Do not equate the commit title
with calibrated end-to-end GPS/RF alignment.

- **Latest documented timing wiring (2026-09-24):** live-sky ZED-F9P TIMEPULSE0 ->
  X300 PPS/TRIG IN; PRS10C 10 MHz -> X300 REF IN; USB/TCP pulse metadata from the
  same F9P. This supersedes September 22 notes about PRS10 PPS or a missing F9P
  PPS cable. The successful check used receiver ID `d44162bdda`, USB
  `/dev/cu.usbmodem111301`, and X300 `192.168.10.2`. Re-enumerate devices and verify
  roles before use: older records assign that receiver ID to the DUT.
- **Verified, no TX:** external reference lock and `--check-pps` paired three
  consecutive hardware edges with GPS labels. Generated/accepted samples and TX
  send calls were all zero. This establishes epoch association, not RF arrival
  accuracy, carrier-phase coherence or long-run oscillator stability.
- **Earlier receive-only evidence (2026-09-22):** bladeRF x40 captured five seconds
  at 2.5 Msps in SC16 Q11 using its internal clock. This establishes sample
  acquisition, not GPS acquisition, timestamp continuity or common X300 timing.
  Live UBX startup checks that day failed on missing navigation subframes.
- **Still unqualified:** RF containment/power budget, calibrated TX/sky-path delay,
  absolute RF timing at the DUT, long-run clock agreement and the final mixed-signal
  dataset. No new hardware state is implied by a documentation review.

### X300 timing contracts

- Without `--gps-pps`, live UBX uses the arrival-time estimate. With `--gps-pps`,
  the bridge needs `--pps` read-only TIM-TP/CFG-TP5/NAV-TIMELS polling (CFG-VALGET
  CFG-TP-* when a ZED-F9K NAKs CFG-TP5; see the dated F9K section in
  `docs/x300-gps-pps-alignment.md`), valid
  NAV-TIMEGPS, external clock/PPS and an independently justified, unsmeared host
  UTC bound via `--pps-host-utc-bound-sec` in `(0, 0.25]` seconds. The documented
  0.2 s value is an operator-declared bound, not a measured RF calibration.
- `--check-time` checks receiver time without UHD; `--check-pps` checks hardware
  epoch association without a TX streamer, but resets the local hardware counter.
  Run it only when no other experiment owns the X300. `--check-start --gps-pps`
  additionally checks the intended waveform/prebuffer/deadline without RF.
  Offline `--dry-run` cannot verify hardware PPS and rejects `--gps-pps`.
- PPS mode freezes a hardware/GPS reference before rendering. Host receipt/UTC
  selects the integer second, not fractional GPS phase. Keep sample-count timing,
  actual UHD rate, partial-send suffixes and fatal continuity checks intact; never
  retime queued IQ or move the reference during a burst.
- PPS mode rejects nonzero `--delivery-delay-sec` and `--model-time-offset-sec`;
  do not reuse the historical approximately 37 ms empirical correction. TX-path
  delay adds and sky-path delay subtracts; both remain uncalibrated in manifests.
- Successful PPS association sets `pps_epoch_association_verified=true` and
  `epoch_association=ublox_tim_tp_hardware_pps`. `gps_alignment_verified` and
  `rf_alignment_verified` remain false. Invalid quantization error is recorded,
  not applied. PPS/label/reference loss, excessive drift and continuity faults
  stop the run; an independent PRS10C frequency reference is not disciplined by PPS.

## Build

Requires a C/C++ toolchain. GCC/Clang for the core; `libbladeRF`, UHD, and Boost only for the
respective live-transmit adapters (kept out of `make all` because they are optional).

```bash
make all          # gps-sdr-sim + jammergen + matchedgen + iqmix (no SDR libs needed)
make jammertx     # live jammer-only via bladeRF (needs libbladeRF)
make bladetx      # live bladeRF: revive-spoofing + matched-code (needs libbladeRF)
make x300tx       # X300 scenarios, UBX live GPS and matched-code (needs UHD/Boost)
make revive_candidates
make USER_MOTION_SIZE=4000 gps-sdr-sim   # for user-motion files longer than the default

# Tests
make test                    # C unit tests (revive/transform/scan, jammer_source, gps_ca,
                             #   matched_code_*, sha256, nav timing) + processing/ Python test
make test-bladetx-matched    # hardware-independent bladeRF matched-code CLI check
make test-x300tx-matched     # hardware-independent X300 matched-code CLI check
make test-x300-timing       # timing/sender and no-hardware CLI checks
make test-ubx               # UBX/LNAV, PPS association, fake-radio live revive, CLI/replay
make test-mixed-gps-timing  # offline mixed-signal timing analysis regression
```

The Python side under `processing/` uses **uv** (`uv run python ...`).

## Architecture — the tool boundary

```text
gps-sdr-sim  -> clean GNSS IQ ----\
jammergen    -> jammer-only IQ ----+-> iqmix -> receiver-front-end model -> test IQ (offline)
noise source -> thermal noise IQ --/

shared jammer source -> TX conditioning -> jammertx / bladetx / x300tx -> conducted-or-shielded DUT (live)
```

- **`gps-sdr-sim`** (`gpssim.c`, `tools/gps_ca.c`) — upstream core: clean GPS L1 C/A IQ from
  RINEX ephemeris + a CSV/NMEA trajectory or a static location.
- **`jammergen`** (`tools/jammergen.c`, `tools/jammer_source.c`) — offline jammer-only IQ. Five
  shared waveforms: `cw`, `narrowband`, `wideband`, `chirp`, `pulsed` (deterministic, seeded).
- **`iqmix`** (`tools/iqmix.c`) — measures clean vs. jammer power and scales to a requested J/S.
- **`matchedgen`** (`tools/matchedgen.c`, `tools/matched_code_source.c`) — offline matched-code
  (PRN-targeted) interference source.
- **`jammertx`** (`player/jammertx.cpp`) — streams the same jammer-only source live through
  bladeRF. No authentic-GPS IQ, RINEX, ephemeris, GPS time, location, or PRNs.
- **`bladetx`** (`player/bladetx.cpp`) — live bladeRF adapter with two research modes:
  _revive-spoofing_ (re-transmit a target PRN from its own past ephemeris, scheduled off Trimble
  1 PPS time tags) and _matched-code_ jammer-only (internal clean IQ generated for alignment,
  then discarded). SC16/Q15 -> device Q11 at the final hardware boundary.
- **`x300tx`** (`player/x300tx.cpp`) — USRP X300 adapter for frozen RINEX
  scenarios, live GPS from F9P UBX/TCP, and separate matched-code
  interference. Uses shared external-clock/PPS startup and continuity handling.
  Revive requires frozen RINEX. `--ublox-time-tcp` combines it with estimated
  F9P time or optional hardware PPS association for finite TX, `--stream` and
  `--check-start`; offline `--dry-run` uses estimated time only. `--ublox-tcp`
  supplies live navigation and rejects revive. Absolute RF alignment remains unverified.
- **X300 support modules:** `player/ubx_receiver.*` parses receiver time, navigation
  and diagnostics; `player/gps_lnav.*` decodes GPS navigation;
  `player/x300_live_time.hpp` owns arrival-based estimation;
  `player/x300_pps_time.hpp` owns labeled PPS association/monitoring;
  `player/x300_timing.hpp` owns radio timing and sender continuity.
- **`revive_candidates`** (`tools/revive_candidates.cpp`) — scans historical
  target ephemerides and can exclude an RTCM or manual `--obs-prns` observation
  list. It does not read UBX and does not establish RF safety or future absence.
- **`tools/ubx_tcp_bridge.py`** — one USB owner with read-only loopback fanout;
  `--pps` polls pulse/configuration/leap metadata without changing receiver settings.
  The player, not the bridge alone, associates these labels with physical X300 edges.
- **`tools/analyze_mixed_gps_timing.py`** — offline mixed-signal timing analysis;
  correlation/empirical offsets do not independently establish absolute RF alignment.
- **Legacy players** — `bladeplayer`, `hackplayer`, `limeplayer`, `plutoplayer`,
  `gps-sdr-sim-uhd.py`, `tx_samples_from_file`/`tx` (file-replay of clean IQ; upstream style).
- **`processing/`** — Python analysis (`cw_dataset.py`, `jamming_campaign_report.py`,
  `ubx_parser.py`/`.ipynb`) over receiver captures; uv-managed.
- **`sdr-monitor/`** — GNSS-SDR receiver configs and captured RINEX/NMEA/PVT used for the
  offline verification campaign.
- **`tools/`** also holds UBX helpers (`ubx_*.py`), `rtcm3_inspect`, `sha256`, `VariadicTable.h`.
- **`satgen/`** trajectory generation, **`rtk/`** RTK sample data, **`extclk/`** external-clock
  notes, **`docs/`** the design/spec/safety documents.

## Key documents

Start with `command.md` and `docs/x300-gps-pps-alignment.md` for the September 24
PPS workflow. `docs/README.md`, `docs/x300-usage.md` and the September 22 research
plan still contain pre-PPS statements; read their dates and check the current
source/tests when they conflict. In particular, claims that direct F9P PPS is
unavailable or live-time revive is offline-only are superseded. Preserve dated
measurements as history rather than treating every document as current hardware state.

- `docs/jamming-mitigation-architecture.md` — the research model and the tool boundary above.
- `docs/jamming-spoofing-scenarios.md` — the scenario catalog and the offline/conducted/shielded
  safety boundary.
- `command.md` — concise Hanoi revive commands, offline checks and conditional
  contained-TX templates. Keep credentials
  out of checked-in commands; bench addresses and serials are dated observations.
- `docs/x300-gps-pps-alignment.md` — current PPS contract, failure conditions,
  manifest semantics and September 24 no-TX hardware evidence.
- `docs/x300-usage.md` — broader modes, compatibility, timing and manifests;
  predates the new PPS workflow.
- `docs/x300-live-sky-mixtracking-research-plan.md` — September 22 research plan,
  earlier evidence and remaining RF/dataset qualification; timing wiring is superseded.
- `docs/x300-f9p-*-2026-09-*.md` — dated measurements; preserve source caveats.
- `docs/x300-mixed-timing*.md`, `docs/x300-model-time-offset-review-2026-09-22.md`
  and `docs/x300-sequence-error-debug-2026-09-22.md` — historical timing/continuity
  investigations, not permission to apply old empirical offsets in PPS mode.
- `docs/realtime-code-aligned-matched-code-*.md`, `docs/realtime-cw-jammer-x300.md` — the live
  fail-closed templates, sample-format boundaries, and controlled-RF requirements.
- `docs/signal-generation-pipeline.md`, `docs/gpssim-source-walkthrough.md` — how the clean
  simulator works internally.
- `notes.md`, `task_plan.md` — running lab log and the GNSS-SDR verification campaign record.

## Conventions

- Keep the tool boundary intact: `gps-sdr-sim` stays focused on **clean** signal generation;
  interference lives in the independent jammer/matched-code sources and is combined by `iqmix`
  (offline) or conditioned by the TX adapters (live).
- Offline reproducibility: prefer a **frozen** RINEX/navigation file over a moving current-hour
  file; keep seeds, manifests, and trajectories with each run's artifacts.
- Every live run has its appropriate no-RF preflight: `--dry-run` for frozen
  scenarios, `--check-start` for live UBX. Keep the exact command, console log,
  manifest, navigation/UBX, trajectory when available, receiver capture and RF
  calibration together. Normal manifests do not capture every CLI setting.
- Do not commit generated binaries, `*.bin` IQ, large captures, secrets, or device credentials.
- Match upstream C style in `gpssim.c`; run `make test` (and, for Python, Ruff/Pyright as the
  existing workflow does) before declaring a change done.

## Provenance

Fork of `github.com/betty2310/gps-sdr-sim`, based on Takuji Ebinuma's GPS-SDR-SIM
(© 2015-2025, MIT License). Research extensions by betty2310 (huynh.dh2310@gmail.com).

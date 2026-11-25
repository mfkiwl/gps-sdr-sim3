# Live F9P PPS epoch association

Implemented 24 September 2026 for this wiring:

```text
Live-sky ZED-F9P TIMEPULSE0 -> X300 PPS/TRIG IN
Same F9P USB -> UBX TCP bridge -> x300tx
PRS10C 10 MHz -> X300 REF IN
```

Add `--gps-pps --pps-host-utc-bound-sec 0.2` to the existing live
`--ublox-time-tcp` or `--ublox-tcp` command. The bound must be justified by an
independently synchronized host UTC clock; it is not a timing calibration.
Without `--gps-pps`, the existing UBX arrival estimate remains the default.

Start the bridge with `--pps`. This adds read-only pulse/configuration/leap
queries, including 10 Hz TIM-TP polls. It does not change receiver settings.
The F9P must already emit consecutive valid NAV-TIMEGPS epochs. A periodic
1 Hz TIM-TP stream also works when current CFG-TP5 and NAV-TIMELS readbacks are
available. Some receiver firmware only supplies periodic TIM-TP despite polls;
the player fails if consecutive labels are unavailable.

```sh
rtk proxy /usr/bin/python3 tools/ubx_tcp_bridge.py \
  --serial /dev/cu.usbmodem111301 --baud 115200 \
  --host 127.0.0.1 --port 5019 --pps
```

In a second terminal, check epoch association without creating a TX streamer:

```sh
rtk proxy ./x300tx --check-pps \
  --ublox-tcp 127.0.0.1:5019 \
  --pps-host-utc-bound-sec 0.2 \
  --addr 192.168.10.2 --ublox-warmup-sec 15 \
  --manifest /tmp/x300-pps-check.json
```

Use a new manifest/recording basename on each run. Like ordinary player
startup, this diagnostic latches local zero at the next PPS; run it while no
other experiment owns the X300. It reports the mapping and sends zero samples.
`--check-start` together with `--gps-pps` additionally exercises the existing
waveform/prebuffer/deadline checks with the intended navigation inputs.

For finite or streaming TX, retain the existing navigation, location, PRN,
sample rate, gain and duration options and add the two PPS flags above.
Remove nonzero `--delivery-delay-sec` and `--model-time-offset-sec`: PPS mode
rejects them, including the historical approximately 37 ms correction.
Explicit `--tx-path-delay-sec` and `--sky-path-delay-sec` remain available with
the existing sign convention, and remain marked uncalibrated in the manifest.

## Timing contract

The player first verifies the external frequency lock and a local-zero PPS
latch. It then consumes new pulse labels and confirms three consecutive
hardware edges. The hardware counter remains local. The frozen pair
`(H_ref, G_ref)` defines the model:

```text
G_model(n) = G_ref + (H_start - H_ref)
             + n / actual_UHD_rate + tx_path_delay - sky_path_delay
```

UBX arrival time and host UTC select and validate the integer second only.
They never contribute a fractional GPS phase to this equation. The timed
hardware deadline, sample count, partial-send suffixes and fatal UHD continuity
errors retain the existing behavior. No clock step or model slew occurs during
the burst.

TIM-TP describes the next TIMEPULSE0 pulse. The receiver's current UTC-grid
configuration is supported by converting its label with fresh, consistent
NAV-TIMEGPS/NAV-TIMELS leap information; GPS-grid labels are also supported.
Rising polarity, active GNSS/TOW alignment, 1 Hz locked period, pulse width and
zero user pulse delay are checked through CFG-TP5. Other GNSS grids are rejected.
These fields follow the [F9 HPG 1.32 interface description, sections 3.10.22,
3.15.26 and 3.19.2](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf).

An integer-second-delayed one-way stream cannot establish its own latency.
`--pps-host-utc-bound-sec` therefore declares an independent, unsmeared UTC
clock bound greater than zero and no larger than 0.25 s. The hardware query
bracket is limited to 50 ms. Together these give a unique candidate hardware
second. The entire first-receipt bracket must fall after the preceding PPS,
within `--pps-max-delivery-sec` (default 0.8 s, maximum 0.9 s), and at least
20 ms before the labeled pulse. A previously passed pulse, a one-second-delayed
stream or an adjacent-second label cannot be silently paired with the next edge.

Exact repeated pulse reports keep their first receipt timestamp and do not
refresh the watchdog. Changed duplicates, skipped/reordered labels, stale
metadata, receiver resets, leap changes, clock steps, reference/PPS loss and
PPS drift above 1 microsecond relative to the frozen sample clock fail the run.
Leap transitions have a 10-second exclusion window; a planned start cannot
cross it. Reacquire after the transition with valid current leap metadata.

The sender owns the mutable PPS monitor; the renderer uses an immutable start
plan. The reference never moves after sample zero is chosen. Quantization error
is recorded with its validity flag and is not applied, even if valid. Antenna
and receiver delay settings are recorded as settings, not measured corrections.

## Provenance and validation

Successful association reports `epoch_association=ublox_tim_tp_hardware_pps`
and `pps_epoch_association_verified=true`. `live_start_plan` contains the frozen
reference; `pps_monitor` contains subsequent confirmations and bounds.
`gps_alignment_verified` and `rf_alignment_verified` remain false because
end-to-end calibrated RF timing has not been established. Input PPS labels
hardware edges; it does not discipline an independent 10 MHz oscillator's rate.
See [Ettus synchronization guidance](https://files.ettus.com/manual/page_sync.html).

Fresh verification in this session:

- `rtk proxy make test-ubx test-x300-timing test-x300tx-matched` passed: four C++
  test programs and 25 Python CLI tests. Coverage includes real UBX framing,
  next/previous pulse confusion, a whole-second delay, duplicates, missing and
  reordered labels, history overrun, UTC/GPS week rollover, changed leap offsets,
  positive/negative leap guards, invalid quantization data, resets and clock/PPS
  faults. Fake-radio finite and streaming revive runs exercise production start
  planning, rendering, partial sends, deadline failures and manifests.
- The connected X300 at `192.168.10.2` read back external clock/time sources,
  reported reference lock and passed the new no-TX `--check-pps` path. It paired
  three edges: `H_ref=2.000000000 s` with GPS `2437:385359.000000000` and planned
  `H_start=7.000000000 s` with GPS `2437:385364.000000000`.
- Maximum coarse host-epoch residual was 5.518 ms and maximum hardware-query
  bracket width was 2.200 ms. These are association checks, not RF accuracy
  measurements. A read-only SNTP response was `+0.018293 +/- 0.123208 s`, within
  the chosen 0.2 s coarse bound. The host clock was not changed.
- The receiver reported UTC grid, GPS-minus-UTC 18 s and invalid quantization
  error. The invalid zero qErr was retained unapplied. No receiver configuration
  was written. The temporary bridge was stopped and released the serial port.
- Generated samples, accepted samples and TX send calls were all zero. No RF
  timing or long-run oscillator stability measurement was made.

## ZED-F9K receiver (28 September 2026)

The live receiver was swapped to a ZED-F9K (`MOD=ZED-F9K`, LAP 1.50,
PROTVER 30.50, unique ID `254de24f5d`, USB `/dev/cu.usbmodem11301`); the
PRS10C 10 MHz and X300 wiring are otherwise as above. The player now accepts
ZED-F9P or ZED-F9K identities. This F9K NAKs CFG-TP5, so `--pps` also sends a
read-only RAM-layer CFG-VALGET of the CFG-TP-* TIMEPULSE0 items. The player
uses that readback only after a CFG-TP5 NAK; F9P behavior is unchanged. The
same 1 Hz, rising, GNSS-locked, TOW-aligned, zero-user-delay rules apply. The
manifest records `pulse_config_source` and reports the RF group delay as `null`
because CFG-VALGET has no such item.

- Receiver settings read back: TP1 enabled, 1 s period, 100 ms locked length,
  unlocked length 0, UTC grid, 50 ns cable delay, TIM-TP enabled on USB.
  Because the unlocked length is zero, the F9K emits no PPS edge until its time
  pulse locks. Shortly after the swap it had a 3D fix but invalid leap seconds
  and no TIM-TP; UHD `test_pps_input` and a 20 s edge monitor saw no PPS.
  About seven minutes later TIM-TP was flowing and edges were present. RXM-RAWX
  output is disabled on this receiver, so `--ublox-tcp` live navigation is not
  ready; `--ublox-time-tcp` and `--check-pps` do not need RAWX.
- The external reference stayed locked for all 193 samples in 20 s (early
  reads immediately after init can show unlocked while the PLL settles). A 12 s
  independent monitor saw 12 edges spaced 1.000000000 s on the X300 clock.
- No-TX `--check-pps` passed: three edges, `H_ref=2.000000000 s` with GPS
  `2438:121216.000000000`, planned `H_start=7.000000000 s` with GPS
  `2438:121221.000000000`. Time base UTC, GPS-minus-UTC 18 s, qErr invalid and
  unapplied, maximum host-epoch residual 1.421 ms and query width 1.487 ms. SNTP
  read `+0.002411 +/- 0.125696 s`. Generated/accepted samples were zero and no
  send was attempted. No receiver configuration was written.

The diagnostic command, manifest, raw UBX/arrival recording and console output
from 24 September are in the temporary [validation directory](/var/folders/36/tkc64gps38s9nzp5s6dtqkpw0000gn/T/x300-gps-pps-validation-20260924-ul1pegkt).
The OS may eventually remove that directory; the measured results are preserved
above. The added association module is `player/x300_pps_time.hpp`, integrated
through `prepareLiveStart()` and the existing sender in `player/x300tx.cpp`.

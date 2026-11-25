# X300 revive — live ZED-F9K PPS + PRS10C 10 MHz / Hanoi

Wiring: live-sky ZED-F9K TIMEPULSE0 -> X300 PPS/TRIG IN; PRS10C 10 MHz -> X300
REF IN. USB time/pulse metadata must come from that same receiver. On
28 September 2026 the F9K (receiver ID `254de24f5d`, `/dev/cu.usbmodem11301`)
passed `--check-pps` and the `--check-start` below; the 24 September check used
an F9P (`d44162bdda`, `/dev/cu.usbmodem111301`). Enumerate the port again
before use. After F9K power-up, PPS edges only start once its time pulse locks
(leap seconds valid, TIM-TP flowing), which took several minutes on 28 September.

The hardware commands below use `--gps-pps`: sample-zero GPS time comes from
the verified hardware PPS/GPS label pair. Absolute RF arrival alignment remains
unmeasured. The host's independently synchronized, unsmeared UTC clock must stay
within the declared `--pps-host-utc-bound-sec 0.2` bound. Host UTC only identifies
the correct second; the physical PPS sets the fractional timing reference.

Keep the reference F9K tracking authentic sky only; the transmitted revive
PRNs must not reach its antenna. The 25 September tuning below
uses the captured group 10,23,24,25 and the unchanged DUT setup. Recheck that
these PRNs are absent from the authentic reference observations before a new
experiment; the selection is not valid indefinitely. The F9K has RXM-RAWX off,
so use its NAV-SAT view (none of 10/23/24/25 were listed at 09:48 UTC on
28 September). Use an existing RINEX
archive with usable templates covering the current revive lookback (up to eight
hours), and freeze a copy before checking or transmitting.

The provisional DUT location is `21.0046121,105.8464833,17.0` (ellipsoid height,
metres), from 108 valid COM4 sky-only epochs after the reported transmission
stopped. It is a standalone GNSS estimate, not a surveyed coordinate. The old
location was about 49 m away. For the same RF setup, `0.00000896` seconds is an
**empirical delay trial**, fitted from the receiver captures. It is not an
independently measured TX-chain delay. Start with a location-only baseline
(`X300_TX_DELAY_SEC=0`), then repeat the check/run with `0.00000896`; keep the
two manifests and captures separate. [Evidence and validation](docs/x300-revive-command-tuning-2026-09-25.md).

~~~bash
make x300tx
/usr/bin/python3 -m serial.tools.list_ports -v

# Read-only UTC clock check. Require a successful reply whose absolute offset
# plus reported uncertainty is below 0.2 s; this command does not set the clock.
sntp -t 3 time.apple.com

# Only if TIMEGPS output is missing (the F9K already emitted it on 28 September):
# stop the bridge/other USB readers before this configuration command.
uvx --from pyubxutils==1.0.6 ubxsetrate --port /dev/cu.usbmodem11301 \
  --baudrate 115200 --msgClass 0x01 --msgID 0x20 --rate 1

# Terminal 1: keep the USB -> raw UBX TCP bridge running, including PPS queries
/usr/bin/python3 tools/ubx_tcp_bridge.py --serial /dev/cu.usbmodem11301 \
  --baud 115200 --host 127.0.0.1 --port 5019 --pps

# Terminal 2: verify estimated GPS now and the receiver ID; no UHD/RF
./x300tx --check-time --ublox-tcp 127.0.0.1:5019 --ublox-warmup-sec 15

./x300tx --check-pps --ublox-tcp 127.0.0.1:5019 \
  --addr 192.168.10.2 --clock-source external --time-source external \
  --pps-host-utc-bound-sec 0.2 --ublox-warmup-sec 15

# Terminal 2: freeze the current navigation input in an ignored run directory.
# hour2710.26n = CDDIS hourly, 28 September 00:00-08:00 UTC. Replace it when the
# revive lookback (up to eight hours) outgrows it; do not overwrite this copy.
mkdir -p analysis-output/runs
X300_RUN_DIR="$(mktemp -d "$PWD/analysis-output/runs/revive-XXXXXX")"
cp hour2710.26n "$X300_RUN_DIR/navigation.26n"
X300_NAV_FILE="$X300_RUN_DIR/navigation.26n"
rtk proxy shasum -a 256 "$X300_NAV_FILE"

# Baseline first. Set to 0.00000896 for the subsequent empirical delay trial.
X300_TX_DELAY_SEC=0
X300_TRIAL=location-only
# X300_TX_DELAY_SEC=0.00000896
# X300_TRIAL=delay-8p96us

# Optional offline waveform check: arrival-estimated GPS time, no UHD/PPS/RF.
# --dry-run does not accept --gps-pps and cannot verify hardware alignment.
rtk proxy ./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  -e "$X300_NAV_FILE" -l 21.0046121,105.8464833,17.0 \
  -P 10,23,24,25 -S 10:revive,23:revive,24:revive,25:revive \
  --rate 2500000 --start-lead-sec 2 --dry-run -d 1

# Check PPS-associated revive rendering and the fixed deadline; no RF
rtk proxy ./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  --gps-pps --pps-host-utc-bound-sec 0.2 \
  --tx-path-delay-sec "$X300_TX_DELAY_SEC" --sky-path-delay-sec 0 \
  -e "$X300_NAV_FILE" -l 21.0046121,105.8464833,17.0 \
  -P 10,23,24,25 -S 10:revive,23:revive,24:revive,25:revive \
  --addr 192.168.10.2 --channel 1 --antenna TX/RX --check-start \
  --clock-source external --time-source external \
  --rate 2500000 --gain 0 -d 180 --start-lead-sec 2 --prebuffer 50 \
  --manifest "$X300_RUN_DIR/$X300_TRIAL-check.json"

# PPS-associated revive TX: qualified conducted/shielded setup, RF B.
# Inspect the matching --check-start result first. This command TRANSMITS.
# Use a fresh trial name/directory when repeating; never overwrite captures.
# 180 s covers both the earlier ~68 s tracking loss and ~126 s transport failure.
rtk proxy ./x300tx --ublox-time-tcp 127.0.0.1:5019 \
  --gps-pps --pps-host-utc-bound-sec 0.2 \
  --tx-path-delay-sec "$X300_TX_DELAY_SEC" --sky-path-delay-sec 0 \
  -e "$X300_NAV_FILE" -l 21.0046121,105.8464833,17.0 \
  -P 10,23,24,25 -S 10:revive,23:revive,24:revive,25:revive \
  --addr 192.168.10.2 --channel 1 --antenna TX/RX \
  --clock-source external --time-source external \
  --rate 2500000 --gain 0 -d 180 --start-lead-sec 2 --prebuffer 50 \
  --manifest "$X300_RUN_DIR/$X300_TRIAL-tx.json"
~~~

Requires `uv` and `pyserial` in the Python interpreter used for the bridge,
with USB UBX output enabled. The
[rate command](https://github.com/semuconsulting/pyubxutils#ubxsetrate-cli)
enables TIMEGPS on all receiver interfaces; it does not change the navigation
solution rate or save configuration to flash. The bridge only forwards/polls;
`--pps` adds TIM-TP, CFG-TP5, CFG-VALGET (CFG-TP-*) and NAV-TIMELS queries, without
changing pulse settings. The F9K NAKs CFG-TP5; the player then uses the CFG-VALGET
readback (manifest `pulse_config_source: CFG-VALGET`).
PPS mode requires a 1 Hz rising, GNSS/TOW-aligned UTC or GPS pulse, zero user pulse
delay and fresh valid time/leap metadata. The current UTC grid is supported;
GPS-minus-UTC is read from the receiver rather than hardcoded.
`--check-time` verifies receiver time readiness; `--check-pps` verifies the
hardware epoch association. A sent configuration command proves neither.

Use RINEX containing usable historical templates for all selected revive PRNs.
An all-revive `-P` selection does not require a current ephemeris set; selections
that also generate non-revived PRNs retain that check. Manifests and UBX logs
get unique default names. `--ublox-tcp` still supplies live navigation
and rejects revive; use `--ublox-time-tcp` for the combination above. Do not set
`-t` or `--gps-week/--gps-tow`: sample zero is planned automatically before rendering.

The old approximately 37 ms empirical correction must be removed. PPS mode
rejects nonzero `--delivery-delay-sec` and `--model-time-offset-sec`.
`--tx-path-delay-sec` advances the model epoch at the same hardware start;
`--sky-path-delay-sec` subtracts an extra authentic RF-branch delay. Both default
to zero and remain marked uncalibrated in the manifest. The 8.96 us trial above
uses the positive path term to test the measured effective group lag; its
physical attribution is still uncalibrated. Do not add it again as a model,
arrival or sky-path correction. A missed startup
deadline requires more `--start-lead-sec` and a new run; it does not justify
changing the model's GPS time. PPS/label/reference loss, drift and UHD continuity
faults stop the run.

Keep rate, gain, channel, antenna and RF attenuation fixed while comparing these
two trials. The previous gain already gave strong target tracking; gain 0 dB
does not establish DUT-input power or containment. Neither the delay trial nor
more prebuffer fixes `EVENT_CODE_SEQ_ERROR`. Preserve that failure as fatal.
Use `--stream` only after finite runs remain continuous. Record both DUTs before
startup and through the end of the run, with RAWX actually present on both; the
previous COM3/F9K files lacked RAWX. Avoid resets within a measurement segment.
The `--ublox-time-tcp` time reference needs no RAWX (it only uses NAV-TIMEGPS,
TIM-TP, TIMEPULSE config and leap data); RAWX matters for the recorded DUTs and
for `--ublox-tcp` live navigation. If the F9K is also recorded as a DUT, enable
its RAWX before the segment.

Inspect these fields after the checks and TX:

| Field | Expected meaning |
|---|---|
| `epoch_association` | `ublox_tim_tp_hardware_pps` |
| `pps_latch_verified` | `true`: local PPS latch checked |
| `pps_epoch_association_verified` | `true`: hardware edges associated with GPS labels |
| `live_start_plan.pps_epoch_association` | Frozen `(H_ref, GPS week/TOW)` and declared bounds |
| `pps_monitor` | Subsequent label/edge confirmations and drift checks |
| `transport.failure_reason` | Empty on success; inspect underflow/sequence/time errors |
| `gps_alignment_verified`, `rf_alignment_verified` | Still `false`: calibrated RF timing unmeasured |

Our achieved milestone is **GPS epoch association and sample-time scheduling**.
The successful no-TX check paired three edges, including `H_ref=2 s` with GPS
`2437:385359`, and sent zero samples. This does not yet establish the timing
error of transmitted signals relative to real sky at the receiver input.

The generator models a receiver epoch at `-l`/`-c`; each PRN's code and data time
also includes its modeled satellite propagation delay. Revive uses transformed
historical ephemerides, so its orbit, Doppler, code phase and navigation content
need not match that PRN's authentic current broadcast. PPS supplies the common
GPS time reference; it does not make all satellite code phases identical.

The next measurement is a common-reference comparison of generated and sky
signals at the intended receiver input: measure the remaining time bias after
accounting for modeled per-PRN propagation, qualify TX/sky-path delays, and check
bias drift and sample continuity over time. Carrier-phase coherence requires
separate evidence. The PRS10C frequency reference is independent of the F9P PPS,
so a short startup pass does not establish long-run rate agreement. This matches
the distinction between sample-clock, PPS and phase synchronization in the
[Ettus synchronization guidance](https://files.ettus.com/manual/page_sync.html).

See [PPS mode, implementation and measured validation](docs/x300-gps-pps-alignment.md)
for the timing contract and the existing
[RF qualification plan](docs/x300-f9p-rf-timing-qualification-2026-09-22.md)
for the remaining measurement scope. Exposed bench antennas remain unqualified
for TX.

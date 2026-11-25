# X300 repeated sequence errors — 2026-09-25

## Finding

The reported run failed on a device-reported host-to-X300 packet sequence
discontinuity. The available evidence does not identify which component lost
or reordered a packet: host stack/scheduling, Ethernet adapter/driver, cable,
UHD transport, or X300/FPGA. No transport repair or successful hardware soak
has been established.

UHD defines `EVENT_CODE_SEQ_ERROR` as packet loss between host and device.
In UHD 4.10, the CHDR TX transport emits this event when it receives
`STRS_SEQERR`, without a hardware timestamp. An accepted `send()` count is
not proof that those samples reached RF continuously.

- [UHD event definitions](https://files.ettus.com/manual/structuhd_1_1async__metadata__t.html)
- [UHD 4.10 CHDR TX transport](https://github.com/EttusResearch/uhd/blob/v4.10.0.0/host/lib/include/uhdlib/rfnoc/chdr_tx_data_xport.hpp)

The sender already polls async events after each data send and stops the
unsent suffix on continuity failure. It preserves partial-send suffixes and
the fixed hardware/GPS timeline. Removing the fatal error would conceal a
broken capture; restarting cannot restore continuous carrier/sample history.
No production source changes were made in this investigation.

## Reported run

Source: `x300tx-ublox-1790328356-20913.json`, its `.ubx` recording and
`.ubx.rx.csv` arrival log. Selected PRNs: 10, 23, 24, 25, all revive.

| Measurement | Result | Interpretation |
|---|---:|---|
| Actual rate | 2,500,000 complex samples/s | SC16 payload is 10 MB/s, or 80 Mb/s |
| Maximum samples per send | 364 | About 6,870 packets/s, including shorter frame tails |
| Generated samples | 327,750,000 | 131.1 s rendered |
| Accepted samples | 315,155,428 | 126.0621712 s submitted; not exact RF failure time |
| Generated minus accepted | 12,594,572 | 5.0378288 s still ahead in host generation |
| Maximum render call | 2.85525 ms | A normal render epoch represents 100 ms |
| Sequence / underflow / time errors | 1 / 0 / 0 | Reported failure class is transport sequence loss |
| Pre-start zero-send retries | 10 | All ten zero sends were pre-start backpressure |
| Maximum send-call duration | 251.010 ms | Includes pre-start waits; cannot attribute it to a mid-run stall |
| Minimum measured accepted-sample lead | 22.708 ms | Host render queue and hardware buffering are different |
| Positive partial sends | 0 | No recorded short positive sends |
| PPS confirmed edges | 132 | Epoch association remained verified through hardware second 133 |

Sample zero is GPS week 2437, TOW 465990: **09:26:30 GPS calendar time =
09:26:12 UTC = 16:26:12 Vietnam time**. Error observation was approximately
09:28:18 UTC based on the saved host/hardware anchor. The exact lost-packet
sample and RF time are unknown because the event has no hardware timestamp.

The timing-reference receiver, ID `d44162bdda`, remained time-valid and
position-valid. Its saved UBX recording has 142 NAV-PVT epochs, all 3D/fixOK,
142 valid NAV-TIMEGPS messages, 142 RAWX messages, no RAWX clock-reset flags,
and no detected UBX checksum errors. Firmware is ZED-F9P HPG 1.32. Neither
PPS labeling nor this receiver's navigation failure explains the reported stop.
PPS association still does not establish calibrated RF alignment.

## Recurrence in nearby saved manifests

Inspected all 39 `x300tx-ublox-*.json` files returned by the repository file
index during this investigation. Fifteen contain sequence failures; all
fifteen report zero underflows. Accepted durations at error observation,
in chronological order, were:

```text
67.061006, 43.061006, 47.063190, 67.054454, 60.062171,
60.062171, 135.049067, 136.061152, 106.054454, 127.061006,
126.062171, 120.062171, 131.063190, 122.059842, 130.051106 s
```

Failures occur with both PRN sets `10,23,24,25` and `10,13,25,32`.
There is no single fixed scenario duration at which rendering fails.
The similar fractional submission frontier is not enough to infer a periodic
hardware fault: asynchronous reporting and buffering obscure the loss instant.

There are also four manifests reporting external-reference unlock, five
reporting no PPS during startup, and one reporting no send progress just
after the scheduled start. One reference-unlock failure occurred mid-stream:
`x300tx-ublox-1790327379-18210.json`, after 57.4 s accepted. These are separate
failure classes that also matter for long-run reliability; they are not the
cause recorded in the reported sequence-error run.

## Supplied receiver captures

Parsed checksum-valid UBX frames directly from the two Downloads files.
Identity comes from MON-VER, time ranges from NAV-PVT/RAWX rather than filenames.
Non-UBX bytes were skipped, not classified as checksum errors.

| Capture | Recorded identity | NAV-PVT UTC range on September 25 | RAWX |
|---|---|---|---:|
| `COM3___9600_260925_090611.ubx` | **ZED-F9K**, LAPL1L2L5 1.50 | 09:06:15–09:15:53 | 0 |
| `COM4___9600_260925_090602.ubx` | ZED-F9P, HPG 1.51 | 09:06:07–09:15:54 | 588 |

Both end over ten minutes before the reported run starts. They overlap earlier
experiments, but cannot establish the DUT response at this run's failure.
Neither has the firmware identity of the run's timing-reference receiver.
COM3 cannot support a RAWX carrier-phase analysis from this file.

COM3 has 485 3D/fixOK epochs and 94 no-fix epochs; COM4 has 528 and 62,
respectively. Both recordings contain two CFG-RST messages with payload
`ffff0200`, and time reinitialization/no-fix intervals. COM4 has two RAWX
clock-reset flags, at GPS TOW 464942 and 465092. Do not attribute these older
resets or fix losses to the later UHD event.

Capture SHA-256:

```text
COM3: 502311f57eaf83706d48e53e971ad3680f37405d478be0b99ed8b5efcfe4393e
COM4: 86c5c7b2b25e77ab2328bee70e5871f6183f61354d663eec1a04832027a51dd7
```

## Offline validation and reproducibility limit

Rendered 600 s from the same requested GPS epoch, location and PRNs:

```sh
rtk proxy ./x300tx --dry-run \
  -e hour2680.26n -l 21.0047844,105.8460541,22 \
  -P 10,23,24,25 -S 10:revive,23:revive,24:revive,25:revive \
  --gps-week 2437 --gps-tow 465990 --rate 2500000 -d 600 \
  --manifest /tmp/x300-seq-20260925-render600.json
```

Result: `dry_run`, 1,500,000,000 samples generated, zero clipped components,
zero samples submitted, no UHD device opened. IQ FNV-1a: `7ca7b87b29515368`.
This checks offline generation across the failure window, not transport,
hardware PPS, receiver health, or RF.

The RINEX file has changed since the reported run despite retaining its name:

```text
Failed-run SHA-256: f58ecc26cde48d829f1c0eb39c9c1044b23287ea0c7cebcc183d212506d318c3
Current SHA-256:    a170df8c529afa6fbb6807436f531bf399319b163ec550d477db8cb0247289dd
```

Consequently this is not an exact historical waveform replay. Preserve a
run-specific copy of navigation input alongside future manifests and captures.

`make test test-x300-timing` passed: core C tests, five processing tests,
the X300 timeline/PPS/queue/transport fault tests, and nine timing CLI tests.
The test target rebuilt the local x300tx executable from unchanged source.
Report whitespace/fence checks and `git diff --check` passed.

## Next isolation test

At investigation time, `route -n get 192.168.10.2` selected the default route
through `en0`, gateway `192.168.0.1`. `ifconfig` showed no active dedicated
X300 interface. This describes the current disconnected state, not the route
during the saved failures. No hardware test or RF transmission was performed.

After reconnecting the direct Ethernet adapter, obtain the actual interface:

```sh
rtk proxy route -n get 192.168.10.2
rtk proxy ifconfig INTERFACE
rtk proxy netstat -I INTERFACE -b -d
```

Replace `INTERFACE` with the dedicated interface reported by the route. Verify
the direct subnet, negotiated link speed and MTU; record counters before and
after the test. Clean interface counters alone cannot exclude UDP loss further
downstream. The present 1472-byte UHD frames already fit MTU 1500.

With x300tx stopped and the conducted/attenuated/DC-isolated or verified
shielded RF path qualified, run this **TX-active** UHD-only comparison. It
configures the radio and transmits; it is not a no-RF check. Options were
verified against the installed UHD 4.10 `benchmark_rate --help`.

```sh
rtk proxy /opt/homebrew/lib/uhd/examples/benchmark_rate \
  --args "type=x300,addr=192.168.10.2" \
  --tx_subdev "A:0 B:0" --tx_channels 1 \
  --tx_rate 2500000 --tx_cpu sc16 --tx_otw sc16 \
  --ref external --pps external --duration 600 \
  --seq-threshold 0 --underrun-threshold 0
```

1. If sequence failures recur, the failure is reproduced without GPS rendering,
   revive or UBX/PPS-monitor application logic. Substitute cable, direct host
   port and Ethernet adapter one at a time, recording the exact configuration
   and results. If it persists, compare a second host/UHD environment before
   assigning blame to the X300 or changing firmware.
2. If repeated UHD-only runs pass but x300tx fails, investigate application
   scheduling, health-query latency and packet submission behavior under the
   same receiver logging workload. The current aggregate send-duration field
   cannot isolate runtime stalls from startup backpressure.
3. After the 600-second test passes repeatedly, test the intended experiment
   duration, then repeat x300tx with its matching no-RF preflight and receiver
   capture. Require zero sequence/underflow/time errors and continued external
   reference/PPS validity. A short successful run does not establish hour-long
   reliability.

Increasing `--prebuffer` cannot repair packet loss; it is already at its
50-epoch maximum. Increasing `--start-lead-sec` only changes startup margin.
Do not blindly enlarge socket buffers or force jumbo frames: UHD cautions
that large send buffers may reduce performance, and every component must
support the chosen MTU. This modest payload rate does not by itself justify
buying a 10 GbE link.
[UHD transport guidance](https://files.ettus.com/manual/page_transport.html)

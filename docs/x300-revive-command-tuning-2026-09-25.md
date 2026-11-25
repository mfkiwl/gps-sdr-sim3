# Revive command tuning — 25 September 2026

The user confirmed that DUT antenna positions and RF cabling are unchanged
from the sp-partial-25-09 recordings. The updated [command.md](../command.md)
provides a location-only baseline and an empirical delay trial, using finite
180-second runs. No production player changes are needed for these options.

These commands remain conditional on a qualified conducted path with measured
attenuation and DC isolation, or verified shielding. The unchanged-setup answer
does not establish RF qualification. Normal revive has no software RF interlock.

## Selected values and evidence

| Setting | Candidate | Reason |
|---|---|---|
| Location | `21.0046121,105.8464833,17.0` | COM4 sky-only position after the quoted TX ended; height is ellipsoidal metres |
| TX path-delay term, baseline | `0` | Measure the effect of the location change on its own |
| TX path-delay term, trial | `0.00000896` s | Provisional effective-delay compensation, rounded to 10 ns |
| Extra sky path-delay term | `0` | No independently measured extra authentic-branch delay available |
| PRNs | `10,23,24,25`, all revive | Preserve the quoted experimental group; recheck authentic presence before use |
| Rate / gain / lead / prebuffer | `2500000 / 0 / 2 / 50` | Hold other settings fixed; the old startup met its deadline |
| Initial duration | `-d 180` | Covers the earlier tracking loss at about 68 s and sequence failure observed around 126 s |

The location uses 108 COM4 NAV-PVT epochs within GPS TOW 466150–466280,
after the quoted transmission ended at about 466116. Only valid fixes with none
of the four target PRNs marked used were retained. Median reported horizontal
accuracy was 1.77 m and vertical accuracy 3.80 m. This is a provisional standalone
GNSS estimate; displayed coordinate digits do not imply centimetre accuracy.

The earlier review used `21.00459685,105.84648345,24.6335` from a shorter
post-reset interval to test sensitivity of the timing estimate. The longer
post-TX interval is preferable as a starting command location. Position medians
in other segments vary by metres, especially in height; a surveyed or long
authentic-only estimate should replace this provisional location when available.

The data source is
`/Users/betty/Developer/research/angle/dataset/sp-partial-25-09/COM4___9600_260925_091854.ubx`.
SHA-256 and detailed evidence are retained in the linked analysis artifacts below.

## Meaning of the timing trial

The source applies:

```text
G_model(n) = G_ref + (H_start - H_ref) + n/Fs
             + tx_path_delay - sky_path_delay
```

A positive TX path term advances modeled satellite time at the same hardware
deadline. That is the correct direction for the measured effective late signal.
It does not change sample rate or shift an already running burst.

With the candidate sky position, geometry/clock-corrected RAWX observations
from the old transmission give:

- TOW 466004–466030, calibration half: median delay **8.95647 us**.
- Rounded command trial: **8.96 us**.
- TOW 466031–466057, retrospective holdout: median remaining delay after
  numerical subtraction **13.58 ns**, p95 absolute remaining delay **25.26 ns**.
- All 54 complete four-PRN epochs: median effective delay **8.96538 us**.

These are retrospective calculations on the original waveform. They are not
results from transmitting the revised command. The delay estimate retains the
old simulator location for its generated satellite ranges and uses the new
DUT estimate for authentic sky ranges. A new location/waveform and changed
satellite geometry can change the residual; the next recording decides whether
this trial value is suitable.

The `--tx-path-delay-sec` flag is the existing PPS-compatible mechanism used
for this empirical trial. The estimate combines physical/model effects and must
not be reported as an independently measured X300 TX-chain delay.
`delays_calibrated`, `gps_alignment_verified` and `rf_alignment_verified`
remain false. Do not add a second copy of this correction elsewhere.

PPS mode rejects `--model-time-offset-sec` and `--delivery-delay-sec` when
nonzero. The old approximately 37 ms correction is unrelated to this trial.
Do not change receiver pulse delays, sample-rate scaling or frequency
correction to remove the roughly constant 9 us residual.

## Run and compare

Use the frozen navigation copy and separate baseline/trial filenames in
[command.md](../command.md). Save the exact executed command and console log
with each manifest, timing-reference UBX and both DUT recordings; the normal
manifest does not include every command setting.

1. Check current receiver identity and valid GPS/PPS metadata. The declared
   host UTC bound must still be independently justified.
2. On an idle X300, run the matching `--check-start --gps-pps` command.
   This resets/latches local hardware time, renders the selected waveform and
   checks the deadline without creating a TX streamer.
3. Inspect the result before the corresponding qualified-path finite TX run.
   Use `X300_TX_DELAY_SEC=0` for the location-only baseline.
4. Set `X300_TX_DELAY_SEC=0.00000896` and a new trial name, then repeat
   the no-TX check and finite run. Keep the RF path, gain and rate fixed.
5. Compare corrected per-PRN timing, navigation residuals/used flags, resolved
   carrier windows and UHD continuity. A finite run ending in a sequence error
   is failed transport validation, even if its initial timing improves.

Require actual RXM-RAWX output from both receivers for two-receiver phase
analysis. The existing COM3/F9K recordings have none. Avoid receiver resets
inside either measurement segment. One successful finite run does not
establish long-duration stability; only then consider the `--stream` form.

Increasing gain is not supported by the present diagnosis: target C/N0 was
already strong. Gain 0 dB is neither receiver-input power nor an RF containment
guarantee. Keep sequence/underflow/time errors fatal. Timing compensation
cannot repair packet loss.

## Offline validation performed

The current navigation input was copied before checking. Its SHA-256 is
`a170df8c529afa6fbb6807436f531bf399319b163ec550d477db8cb0247289dd`;
the quoted September 25 failed run used a different hash under the same original
filename. Thus this is a current-file waveform check at a historical epoch,
not an exact replay of the failed transmission.

Both cases used the new position, the four requested PRNs and 2.5 Msps:

| Offline case | GPS week:TOW at sample zero | Generated samples | Clipping | UHD calls / accepted samples |
|---|---|---:|---:|---:|
| Location-only | 2437:465990.000000000 | 450,000,000 | 0 | 0 / 0 |
| Delay trial | 2437:465990.000008960 | 450,000,000 | 0 | 0 / 0 |

Each rendered 180 s successfully, including navigation-frame boundaries.
The offline trial directly sets the fractional model epoch; offline mode
cannot use or validate hardware PPS. This verifies rendering with the candidate
offset, not physical alignment, current navigation availability, transport
reliability or future DUT behavior. No new live hardware check or RF run was
performed for this command update.

`make test test-x300-timing test-ubx` passed, including the hardware-independent
PPS association, sender continuity and fake-radio live-revive cases. The command
block passed `zsh -n`, and `git diff --check` found no whitespace errors.

Validation artifacts:

- [Candidate calculation](/tmp/x300-revive-tuning-20260925-w3JGdH/candidate.json)
- [Position epochs](/tmp/x300-revive-tuning-20260925-w3JGdH/position-epochs.csv)
- [Delay epochs](/tmp/x300-revive-tuning-20260925-w3JGdH/delay-epochs.csv)
- [Offline baseline](/tmp/x300-revive-tuning-20260925-w3JGdH/offline-baseline.json)
- [Offline trial](/tmp/x300-revive-tuning-20260925-w3JGdH/offline-trial.json)
- [Original six-capture review](/tmp/x300-pps-review-20260925-oa8XLV/review.md)

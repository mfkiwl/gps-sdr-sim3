# gps-sdr-sim (research fork)

A fork of [osqzss/gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim) — a
software-defined GPS L1 C/A baseband signal simulator — extended with the
test-signal tooling used by our GNSS spoofing **detection** research.

This fork generates the authentic and spoofed signal scenarios that feed the
angle-of-arrival (AoA) detector described in the paper:

> **GNSS Spoofing Detection by Estimating Angle of Arrival based on Carrier Phase
> Double Difference Measurements**, Hiep Hoang Van and Thuan Nguyen Dinh.

Companion repositories:

- [angle](https://github.com/betty2310/angle) / [gnss-aoa-dd](https://github.com/betty2310/gnss-aoa-dd)
  — the detector: it consumes two-receiver u-blox logs and estimates each
  satellite's DD-method AoA to flag spoofing. This fork is only the signal
  *generator* that produces the test captures.

## Authorized, controlled use

This is defensive security research. The generator writes baseband IQ to a file;
it transmits nothing by itself. In our experiments the IQ is replayed through an
SDR only over a **conducted (cabled) RF path or inside a shielded enclosure**, at
calibrated attenuation, onto receivers the authors own — never over the air onto
public spectrum. Anyone using this fork is responsible for keeping transmissions
inside such a controlled environment and within local regulations. Do not use it
to interfere with receivers or spectrum you do not control.

## What this fork adds over upstream

Beyond upstream's trajectory-plus-ephemeris signal generation:

- Partial-constellation and reference-ephemeris ("revive") modes for building
  authentic-vs-spoofed scenarios (`revive_candidates`, `matchedgen`).
- Jammer and matched-code test-signal generators (`jammergen`, `jammertx`) and an
  IQ mixer (`iqmix`) for combining sources.
- SDR players and a USRP X300 / ZED-F9P hardware-timing path (`player/`,
  `x300tx`) with PPS/GPS epoch association.

The X300/F9P live-timing path is experimental and **not** calibrated for absolute
RF alignment; see [command.md](command.md) and the dated notes under [docs/](docs/)
for its current, qualified state. For the paper's datasets, the generated IQ is
replayed in the controlled path described above.

## Build

```bash
make            # gps-sdr-sim and the test-signal tools (jammergen, matchedgen, iqmix)
make test       # unit tests
```

The core simulator alone also builds standalone with GCC (see below).

### Windows build instructions

1. Start Visual Studio.
2. Create an empty project for a console application.
3. On the Solution Explorer at right, add "gpssim.c" and "getopt.c" to the Souce Files folder.
4. Select "Release" in Solution Configurations drop-down list.
5. Build the solution.

### Building with GCC

```
$ gcc gpssim.c -lm -O3 -o gps-sdr-sim
```

### Using bigger user motion files

In order to use user motion files with more than 30000 samples (at 10Hz), the `USER_MOTION_SIZE`
variable can be set to the maximum time of the user motion file in seconds. It is advisable to do
this using make so gps-sdr-bin can update the size when needed. e.g:

```
$ make USER_MOTION_SIZE=4000
```

This variable can also be set when compiling directly with GCC:

```
$ gcc gpssim.c -lm -O3 -o gps-sdr-sim -DUSER_MOTION_SIZE=4000
```

### Generating the GPS signal file

A user-defined trajectory can be specified in either a CSV file, which contains
the Earth-centered Earth-fixed (ECEF) user positions, or an NMEA GGA stream.
The sampling rate of the user motion has to be 10Hz.
The user is also able to assign a static location directly through the command line.

The user specifies the GPS satellite constellation through a GPS broadcast
ephemeris file. The daily GPS broadcast ephemeris file (brdc) is a merge of the
individual site navigation files into one. The archive for the daily file can
be downloaded from: https://cddis.nasa.gov/archive/gnss/data/daily/. Access
to this site requires registration, which is free.

These files are then used to generate the simulated pseudorange and
Doppler for the GPS satellites in view. This simulated range data is
then used to generate the digitized I/Q samples for the GPS signal.

The bladeRF and ADALM-Pluto command line interface requires I/Q pairs stored as signed
16-bit integers, while the hackrf_transfer and gps-sdr-sim-uhd.py
support signed bytes.

HackRF, bladeRF and ADALM-Pluto can accept the default sample rate of 2.6MHz,
while the USRP2 requires an even integral decimator of 100 MHz, i.e. 2.5MHz.

The simulation start time can be specified if the corresponding set of ephemerides
is available. Otherwise the first time of ephemeris in the RINEX navigation file
is selected.

The maximum simulation duration time is defined by USER_MOTION_SIZE to
prevent the output file from getting too large.

The output file size can be reduced by using "-b 1" option to store
four 1-bit I/Q samples into a single byte.
You can use [bladeplayer](https://github.com/osqzss/gps-sdr-sim/tree/master/player)
for bladeRF to playback the compressed file.

```
Usage: gps-sdr-sim [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)
  -u <user_motion> User motion file in ECEF x, y, z format (dynamic mode)
  -x <user_motion> User motion file in lat, lon, height format (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.15,1022538.18,4872414.48
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100
  -L <wnslf,dn,dtslf> User leap future event in GPS week number, day number, next leap second e.g. 2347,3,19
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -T <date,time>   Overwrite TOC and TOE to scenario start time
  -d <duration>    Duration [sec] (dynamic mode max: 300 static mode max: 86400)
  -o <output>      I/Q sampling data file (default: gpssim.bin ; use - for stdout)
  -s <frequency>   Sampling frequency [Hz] (default: 2600000)
  -b <iq_bits>     I/Q data format [1/8/16] (default: 16)
  -r <lead_sec>    Stream lead time for -n timed start (default: 1.0)
  -i               Disable ionospheric delay for spacecraft scenario
  -p [fixed_gain]  Disable path loss and hold power level constant
  -A <attack_spec> Legacy experimental config PRN:method[,PRN:method...]
                   Methods: normal,jam_drop,jam_noise,spoof_delay,spoof_nav
                   (Currently implemented: jam_drop,jam_noise)
  -n               Stream-now mode: use precise GPS time with stream lead
  -v               Show details about simulated channels
```

### Legacy Experimental Scenario Controls

Use `-A` to set a method per PRN.

```
> gps-sdr-sim -e brdc0010.22n -l 35.681298,139.766247,10 -A 3:jam_drop,11:jam_drop
```

These controls modify simulated satellite channels; they do not implement independent RF jammer sources.
`jam_drop` sets the selected simulated channel gain to zero and is therefore an oracle
constellation-ablation baseline. `jam_noise` replaces selected PRN samples with deterministic
pseudo-random noise-like I/Q, but fresh noise is generated at every output sample, so multiplying
it by a C/A-code sign does not establish PRN selectivity. Other methods (`spoof_delay`,
`spoof_nav`) are accepted as placeholders for scenario planning.

Do not use `jam_noise` to claim matched-code target/non-target rejection. See the
[jamming mitigation architecture](docs/jamming-mitigation-architecture.md) for the independent
`jammergen` and `iqmix` architecture, and the
[interference mitigation scenario catalog](docs/jamming-spoofing-scenarios.md) for valid research scenarios.

### Offline synthetic jamming datasets

The repository now includes standalone streaming `jammergen` and `iqmix` tools plus a
Python workflow for reproducible additive CW, narrowband-noise, wideband-noise, chirp,
pulsed-CW, thermal-noise, and clean GPS datasets. The five jammer types share one C
renderer with the UHD backend and have type-specific numerical acceptance gates.
The canonical workflow creates an exact 90-second clean control and measured J/S sweep,
runs numerical waveform gates, produces plots and checksums, and validates fixtures with
the dedicated automatic-channel GNSS-SDR configuration. It does not modify `gpssim.c` or
provide RF transmission instructions.

```sh
make jammergen iqmix
cd processing
uv run python cw_dataset.py create --profile canonical \
  --rinex /path/to/frozen-navigation.rnx \
  --start-time 2026/07/12,00:00:00 \
  --output-dir ../cw-dataset-output/run-20260712
```

Use `--jammer-type cw|narrowband|wideband|chirp|pulsed`; see the
[offline multi-waveform dataset guide](docs/cw-jamming-dataset.md) for type parameters,
the sample contract, fast test profile, receiver gates, and authentic-signal boundary.

For a bounded GNSS-SDR cross-type check, use `--profile verification` once per
jammer type. It creates a 90-second matched clean/+20 dB J/S pair with a
40-second clean prefix, GNSS-SDR tracking/PVT evidence, and post-correlation
PNG/PDF spectrum-spectrogram figures for every visible PRN. Aggregate completed
runs with `processing/jamming_campaign_report.py`; raw RF spectra remain
composite because GPS L1 C/A PRNs overlap in frequency.

For an authorized conducted or verified shielded hardware experiment, `jammertx`
generates the same selected source in memory and streams jammer-only SC16/Q11 through bladeRF. It
does not use authentic-GPS IQ, RINEX, ephemeris, GPS time, receiver location, or PRNs.
Build it explicitly with `make jammertx`; it is not part of `make all` because libbladeRF is
optional. The [real-time X300 multi-waveform guide](docs/realtime-cw-jammer-x300.md)
documents the shared signal model, calibration boundary, receiver evidence, and
controlled-RF safety requirements; use `jammertx --help` for bladeRF-specific options.

For simulator-aligned matched-code experiments, `x300tx` and `bladetx` use the
same navigation-data-free, PRN-targeted source and internal clean-GPS alignment
state. The clean simulator IQ is discarded: both hardware adapters transmit a
continuous jammer-only stream from sample zero until operator stop. The
bladeRF adapter converts canonical SC16/Q15 samples to its SC16 Q11 device
format at the final hardware boundary. See the [real-time matched-code X300
researcher guide](docs/realtime-code-aligned-matched-code-x300.md) for the
shared waveform and safety contract; build and test the bladeRF adapter with
`make test-bladetx-matched` and use the [bladeRF matched-code guide](docs/realtime-code-aligned-matched-code-bladerf.md)
for its Q11 boundary and live fail-closed command contract.

The user motion can be specified in either dynamic or static mode:

```
> gps-sdr-sim -e brdc3540.14n -u circle.csv
```

```
> gps-sdr-sim -e brdc3540.14n -x circle_llh.csv
```

```
> gps-sdr-sim -e brdc3540.14n -g triumphv3.txt
```

```
> gps-sdr-sim -e brdc3540.14n -l 30.286502,120.032669,100
```

### Transmitting the samples

The following are upstream file-playback examples, separate from the current
X300 runbook. Use a conducted path with known attenuation and suitable DC
isolation, or verified shielding. Determine attenuation from measured TX and
receiver-input levels; a bare coax or a software gain setting does not establish
a qualified path. Do not use these examples on the present exposed antennas.

#### BladeRF:

The simulated GPS signal file, named "gpssim.bin", can be loaded
into the bladeRF for playback as shown below:

```
set frequency 1575.42M
set samplerate 2.6M
set bandwidth 2.5M
set txvga1 -25
cal lms
cal dc tx
tx config file=gpssim.bin format=bin
tx start
```

You can also execute these commands via the `bladeRF-cli` script option as below:

```
> bladeRF-cli -s bladerf.script
```

For live stdin or FIFO streaming, `bladeplayer` uses timed TX start automatically.
Use the same `-r <lead_sec>` value on both tools when tuning stream alignment.
For untimed consumers such as `gps-sdr-sim-uhd.py`, set `-r 0` in
`gps-sdr-sim` to disable the lead bias.

#### HackRF:

The output data have to be 8-bit signed I/Q samples.

```
> gps-sdr-sim -e brdc0010.22n -b 8
```

You can use `hackrf_transfer` tool in the HackRF host software.

```
> hackrf_transfer -t gpssim.bin -f 1575420000 -s 2600000 -a 1 -x 0
```

#### UHD supported devices (tested with USRP2 only):

```
> gps-sdr-sim-uhd.py -t gpssim.bin -s 2500000 -x 0
```

You can also use `tx_samples_from_file` tool included in the UHD examples:

```
> tx_samples_from_file --file gpssim.bin --type short --rate 2500000 --freq 1575420000 --gain 0
```

#### LimeSDR (in case of 1 Msps 1-bit file, to get full BaseBand dynamic and low RF power):

```
> limeplayer -s 1000000 -b 1 -d 2047 -g 0.1 < ../circle.1b.1M.bin
```

#### ADALM-Pluto (PlutoSDR):

The ADALM-Pluto device is expected to have its network interface up and running and is accessible
via "pluto.local" by default.

Default settings:

```
> plutoplayer -t gpssim.bin
```

Set TX attenuation:

```
> plutoplayer -t gpssim.bin -a -30.0
```

Default -20.0dB. Applicable range 0.0dB to -80.0dB in 0.25dB steps.

Set RF bandwidth:

```
> plutoplayer -t gpssim.bin -b 3.0
```

Default 3.0MHz. Applicable range 1.0MHz to 5.0MHz.

### License

Copyright &copy; 2015-2025 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).

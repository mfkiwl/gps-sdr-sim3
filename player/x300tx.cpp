/*
 * x300tx.cpp — Integrated USRP X300 GPS signal generator and transmitter.
 *
 * Generates GPS L1 C/A IQ samples in-process and transmits them directly
 * via the UHD C++ API with timed TX metadata.  Eliminates the pipe-based
 * latency of the old gps-sdr-sim | gps-sdr-sim-uhd.py path.
 *
 * Build:  make x300tx   (from the project root)
 *
 * Copyright (c) 2024-2026  — same licence as gps-sdr-sim
 */

#include <uhd/device.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <cerrno>

#include "player/ubx_receiver.hpp"

extern "C" {
#include "gpssim.h"
#include "tools/matched_code_plan.h"
#include "tools/matched_code_source.h"
#include "tools/sha256.h"
}

#include "player/matched_code_alignment.h"
#include "player/x300_live_time.hpp"
#include "player/x300_timing.hpp"

////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define TX_FREQUENCY 1575420000.0 // GPS L1 (Hz)
#define TX_SAMPLERATE 2500000.0   // 2.5 Msps, exact from X300 200 MHz clock
#define PREBUFFER_DEFAULT 5       // 0.5 s pre-buffer
#define TX_START_LEAD_DEFAULT_NS 250000000LL // 250 ms startup lead
#define TX_START_LEAD_MAX_SEC 60.0
#define TX_START_LEAD_MIN_SEC 0.02
#define EPOCH_TARGET_SEC 0.1
#define MATCHED_RATE_TOLERANCE_HZ 0.5
#define MATCHED_DRY_RUN_VALIDATION_SEC 0.1
#define MATCHED_MIN_HEADROOM_DB 1.0
#define FNV1A64_OFFSET_BASIS UINT64_C(14695981039346656037)
#define FNV1A64_PRIME UINT64_C(1099511628211)

struct MatchedCodeOptions {
  bool enabled = false;
  bool dry_run = false;
  bool controlled_rf_confirmed = false;
  bool amplitude_set = false;
  bool phase_seed_set = false;
  bool address_set = false;
  bool channel_set = false;
  bool antenna_set = false;
  bool legacy_js_set = false;
  bool legacy_gain_boost_set = false;
  std::string target_prns;
  std::string manifest_path;
  std::string trajectory_path;
  std::string calibration_id;
  double amplitude = 0.0;
  uint64_t phase_seed = 0;
};

struct MatchedCodeRunResult {
  std::string status = "incomplete";
  std::string failure_reason;
  std::string device_type;
  std::string device_product;
  std::string device_serial;
  std::string device_address;
  std::string actual_antenna;
  std::string start_mode;
  size_t tx_channel_count = 0;
  double actual_rate_hz = 0.0;
  double actual_frequency_hz = 0.0;
  double actual_gain_db = 0.0;
  double uhd_start_time_seconds = 0.0;
  bool start_margin_met = false;
  bool pps_verified = false;
  bool target_allocation_passed = false;
  uint64_t internal_alignment_samples = 0;
  uint64_t rendered_jammer_samples = 0;
  uint64_t quantized_samples = 0;
  uint64_t sent_samples = 0;
  x300::TxStats transport;
  uint64_t underflows = 0;
  uint64_t sequence_errors = 0;
  uint64_t time_errors = 0;
  bool interrupted = false;
  matched_code_source_metrics_t source_metrics{};
  uint64_t jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  std::string trajectory_sha256;
  int exit_status = 1;
};

struct LiveRun {
  std::string endpoint, recording, replay_path;
  std::string device_address, tx_subdevices;
  size_t tx_channel = 0;
  std::unique_ptr<ubx::Receiver> receiver;
  ubx::Snapshot initial;
  x300::LiveStartPlan plan;
  std::vector<int> prns;
  bool check_receiver = false, check_start = false, check_time = false;
  bool check_pps = false;
  bool time_only = false;
  bool position_from_receiver = false;
  bool uhd_opened = false;
  double host_epoch_monotonic = 0;
  double warmup = 60, delivery = 0, tx_path = 0, sky_path = 0;
  double model_time_offset = 0;
  bool model_time_offset_set = false;
  bool gps_pps = false;
  bool pps_bounds_set = false;
  x300::PpsOptions pps_options;
  std::unique_ptr<x300::PpsAssociation> pps_association;
  bool enabled() const { return !endpoint.empty() || !replay_path.empty(); }
  bool navigation_from_ubx() const { return enabled() && !time_only; }
  bool replaying() const { return !replay_path.empty(); }
  ubx::Snapshot snapshot() const {
    return receiver ? receiver->snapshot() : initial;
  }
  void checkHealth(double now, gpstime_t epoch) const {
    const auto current = snapshot();
    // Snapshot first, then query time: a receiver-thread update between the
    // caller's timestamp and snapshot must not look like a future arrival.
    if (receiver)
      now = ubx::monotonicSeconds();
    x300::checkLiveHealth(current, plan, now, epoch, prns,
                          navigation_from_ubx());
  }
  // Called only from the sender/control thread, never by the IQ producer.
  void checkPps(x300::Radio &radio) {
    if (pps_association) {
      const auto current = snapshot();
      pps_association->observe(current, x300::observePps(radio));
    }
  }
};

////////////////////////////////////////////////////////////
// Signal handling
////////////////////////////////////////////////////////////

static void sighandler(int) {
  stop_requested = 1; // gpssim.c global
}

static void installSignalHandlers(void) {
  signal(SIGINT, sighandler);
  signal(SIGTERM, sighandler);
  signal(SIGPIPE, SIG_IGN);
}

static double getMonotonicSeconds(void) {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class UhdRadio final : public x300::Radio {
public:
  UhdRadio(uhd::usrp::multi_usrp::sptr device,
           uhd::tx_streamer::sptr stream = {}, size_t channel = 0)
      : device_(std::move(device)), stream_(std::move(stream)),
        channel_(channel) {}
  uhd::time_spec_t now() override { return device_->get_time_now(); }
  uhd::time_spec_t lastPps() override { return device_->get_time_last_pps(); }
  void latchNextPps() override {
    device_->set_time_next_pps(uhd::time_spec_t(0.0));
  }
  void checkLocks() override {
    if (!device_->get_mboard_sensor("ref_locked").to_bool())
      throw std::runtime_error("external frequency reference is not locked");
    if (stream_ && !device_->get_tx_sensor("lo_locked", channel_).to_bool())
      throw std::runtime_error("TX local oscillator is not locked");
  }
  double monotonic() override { return getMonotonicSeconds(); }
  void sleep(double seconds) override {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
  size_t send(const int16_t *iq, size_t count,
              const uhd::tx_metadata_t &metadata, double timeout) override {
    static const int16_t empty[2] = {0, 0};
    return stream_->send(iq ? iq : empty, count, metadata, timeout);
  }
  bool receiveEvent(uhd::async_metadata_t &event, double timeout) override {
    return stream_->recv_async_msg(event, timeout);
  }

private:
  uhd::usrp::multi_usrp::sptr device_;
  uhd::tx_streamer::sptr stream_;
  size_t channel_;
};

static void configureClock(const uhd::usrp::multi_usrp::sptr &device,
                           const char *clock_source, const char *time_source) {
  device->set_clock_source(clock_source);
  device->set_time_source(time_source);
  if (device->get_clock_source(0) != clock_source ||
      device->get_time_source(0) != time_source)
    throw std::runtime_error("hardware clock/time source readback mismatch");
  fprintf(stderr, "[SYNC] clock=%s time=%s; awaiting external PPS\n",
          clock_source, time_source);
}

// Navigation is deliberately independent of the time source. A missing live
// ephemeris for a revived PRN must not prevent a time-only hardware start.
static gpstime_t prepareLiveStart(LiveRun &live, x300::Radio *radio,
                                  double lead) {
  live.initial = live.snapshot();
  if (live.gps_pps) {
    if (!radio || live.delivery != 0 || live.model_time_offset != 0)
      throw std::runtime_error("--gps-pps needs hardware and rejects "
                               "arrival/model-time compensation");
    const double begin = radio->monotonic();
    live.pps_association =
        std::make_unique<x300::PpsAssociation>(live.pps_options, begin);
    // Acquire AFTER the local-zero latch, from newly received labels. Warmup
    // history can refer to the old hardware counter and is not reusable.
    while (!live.pps_association->reference().verified) {
      if (stop_requested || radio->monotonic() - begin > 12)
        throw std::runtime_error("PPS/GPS epoch acquisition stopped or timed "
                                 "out; need bridge --pps");
      radio->checkLocks();
      const auto current = live.snapshot();
      if (current.pulse_config.present && current.leap_info.present)
        live.pps_association->observe(current, x300::observePps(*radio));
      radio->sleep(.01);
    }
    live.initial = live.snapshot();
    live.plan = x300::planPpsStart(live.initial, x300::observeClock(*radio),
                                   live.pps_association->reference(), lead,
                                   live.tx_path, live.sky_path);
    fprintf(stderr,
            "[GPS-PPS] Verified %u labeled edges: H_ref=%.9f -> GPS %d:%.9f; "
            "H_start=%.9f -> model %d:%.9f. Host UTC bound assumption %.3f s; "
            "RF path uncalibrated.\n",
            live.plan.pps.confirmed_edges, live.plan.pps.hardware,
            live.plan.pps.gps.week, live.plan.pps.gps.sec,
            live.plan.hardware_start, live.plan.gps_zero.week,
            live.plan.gps_zero.sec, live.pps_options.host_utc_bound);
  } else if (radio) {
    live.plan = x300::planLiveStart(live.initial, x300::observeClock(*radio),
                                    lead, live.delivery, live.tx_path,
                                    live.sky_path, live.model_time_offset);
    fprintf(stderr,
            "[LIVE] Fixed hardware deadline %.9f s -> estimated GPS %d:%.9f; "
            "arrival correction %.6f s, variation %.6f s, query width %.6f s; "
            "delivery assumption %.6f s; model offset %+.9f s; "
            "absolute uncertainty unknown.\n",
            live.plan.hardware_start, live.plan.gps_zero.week,
            live.plan.gps_zero.sec, live.plan.estimate.arrival_correction,
            live.plan.estimate.arrival_variation, live.plan.correlation.width(),
            live.delivery, live.model_time_offset);
  } else {
    live.plan = {};
    live.plan.estimate =
        ubx::estimateGpsNow(live.initial, getMonotonicSeconds(), live.delivery);
    live.host_epoch_monotonic = live.plan.estimate.host_monotonic + lead;
    live.plan.gps_zero = live.plan.estimate.gps;
    live.plan.gps_zero.sec += lead;
    live.plan.gps_zero = x300::sampleTime(live.plan.gps_zero, 0, 1);
    live.plan.gps_zero_before_model_offset = live.plan.gps_zero;
    live.plan.model_time_offset = live.model_time_offset;
    live.plan.gps_zero =
        x300::offsetModelEpoch(live.plan.gps_zero, live.model_time_offset);
    live.plan.anchor = live.initial.time;
    live.plan.time_rejections = live.initial.time_rejections;
    live.plan.clock_resets = live.initial.clock_resets;
    fprintf(stderr,
            "[TIME-ONLY] Offline host epoch %.9f -> estimated GPS %d:%.9f; "
            "model offset %+.9f s; no RF deadline or hardware association.\n",
            live.host_epoch_monotonic, live.plan.gps_zero.week,
            live.plan.gps_zero.sec, live.model_time_offset);
  }
  live.checkHealth(radio ? radio->monotonic() : getMonotonicSeconds(),
                   live.plan.gps_zero);
  return live.plan.gps_zero;
}

typedef struct {
  int base_samples;
  double fractional_samples;
  double accumulator;
  int max_samples;
} epoch_plan_t;

static void initEpochPlan(epoch_plan_t *plan, double sample_rate_hz) {
  double exact_samples = sample_rate_hz * EPOCH_TARGET_SEC;

  plan->base_samples = (int)floor(exact_samples);
  plan->fractional_samples = exact_samples - (double)plan->base_samples;
  plan->accumulator = 0.0;
  plan->max_samples =
      plan->base_samples + (plan->fractional_samples > 1.0e-12 ? 1 : 0);
}

static int nextEpochSampleCount(epoch_plan_t *plan) {
  int sample_count = plan->base_samples;

  plan->accumulator += plan->fractional_samples;
  if (plan->accumulator >= 1.0 - 1.0e-12) {
    sample_count++;
    plan->accumulator -= 1.0;
  }

  return sample_count;
}

static gpstime_t getGpsTimeAtSampleOffset(gpstime_t zero, long long samples,
                                          double rate) {
  if (samples < 0)
    throw std::runtime_error("negative sample offset");
  return x300::sampleTime(zero, static_cast<uint64_t>(samples), rate);
}

static int hasCloneMode(const synth_config_t *cfg) {
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (cfg->mode[sv] == SYNTH_CLONE)
      return TRUE;
  }

  return FALSE;
}

static int hasReviveMode(const synth_config_t *cfg) {
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (cfg->mode[sv] == SYNTH_REVIVE)
      return TRUE;
  }

  return FALSE;
}

////////////////////////////////////////////////////////////
// Generate one 0.1-second epoch of SC16 IQ samples
////////////////////////////////////////////////////////////

static void prepareEpoch(channel_t chan[MAX_CHAN], int gain[MAX_CHAN],
                         ephem_t *active_eph, ionoutc_t *ionoutc, gpstime_t grx,
                         double epoch_duration, double delt,
                         int path_loss_enable, int fixed_gain,
                         double ant_pat[37], int attack_enabled,
                         const attack_config_t *attack_cfg) {
  int i;

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0) {
      range_t rho;
      int sv = chan[i].prn - 1;
      double path_loss;
      double ant_gain;
      int ibs;

      computeRange(&rho, active_eph[sv], ionoutc, grx, xyz[0]);
      chan[i].azel[0] = rho.azel[0];
      chan[i].azel[1] = rho.azel[1];
      double nav_seconds = subGpsTime(chan[i].rho0.g, chan[i].g0) + 6.0 -
                           chan[i].rho0.range / SPEED_OF_LIGHT;
      if (!std::isfinite(nav_seconds) || nav_seconds < 0.0 ||
          nav_seconds >= N_DWRD * 0.6)
        throw std::runtime_error(
            "navigation words unavailable for sample epoch");
      computeCodePhase(&chan[i], rho, epoch_duration);
#ifndef FLOAT_CARR_PHASE
      chan[i].carr_phasestep =
          (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
      path_loss = 20200000.0 / rho.d;
      ibs = (int)((90.0 - rho.azel[1] * R2D) / 5.0);
      ant_gain = ant_pat[ibs];
      gain[i] = path_loss_enable == TRUE ? (int)(path_loss * ant_gain * 128.0)
                                         : fixed_gain;
      if (attack_enabled == TRUE)
        applyGainAttack(attack_cfg, chan[i].prn, &gain[i]);
      if (attack_cfg != nullptr && attack_cfg->partial_mode &&
          attack_cfg->gain_boost_db != 0.0)
        gain[i] = (int)(gain[i] * pow(10.0, attack_cfg->gain_boost_db / 20.0));
    }
  }
}

static int cleanCarrierTableIndex(const channel_t *channel) {
#ifdef FLOAT_CARR_PHASE
  return (int)floor(channel->carr_phase * 512.0);
#else
  return (channel->carr_phase >> 16) & 0x1ff;
#endif
}

static void advanceCleanChannelSample(channel_t *channel, double delt) {
  channel->code_phase += channel->f_code * delt;
  if (channel->code_phase >= CA_SEQ_LEN) {
    channel->code_phase -= CA_SEQ_LEN;
    ++channel->icode;
    if (channel->icode >= 20) {
      channel->icode = 0;
      ++channel->ibit;
      if (channel->ibit >= 30) {
        channel->ibit = 0;
        ++channel->iword;
      }
      if (channel->iword < 0 || channel->iword >= N_DWRD)
        throw std::runtime_error(
            "navigation word index outside generated frame");
      channel->dataBit =
          (int)((channel->dwrd[channel->iword] >> (29 - channel->ibit)) &
                0x1UL) *
              2 -
          1;
    }
  }
  channel->codeCA = channel->ca[(int)channel->code_phase] * 2 - 1;
#ifdef FLOAT_CARR_PHASE
  channel->carr_phase += channel->f_carr * delt;
  if (channel->carr_phase >= 1.0)
    channel->carr_phase -= 1.0;
  else if (channel->carr_phase < 0.0)
    channel->carr_phase += 1.0;
#else
  channel->carr_phase += channel->carr_phasestep;
#endif
}

static void generateEpoch(short *iq_buff, int sample_count,
                          channel_t chan[MAX_CHAN], int gain[MAX_CHAN],
                          ephem_t *active_eph, ionoutc_t *ionoutc,
                          gpstime_t grx, int staticLocationMode,
                          double epoch_duration, double delt,
                          int path_loss_enable, int fixed_gain,
                          double ant_pat[37], int attack_enabled,
                          const attack_config_t *attack_cfg,
                          unsigned int attack_noise_state[MAX_SAT],
                          double jam_js_linear, uint64_t *clipped_components) {
  int i, isamp;
  int ip, qp, iTable;

  prepareEpoch(chan, gain, active_eph, ionoutc, grx, epoch_duration, delt,
               path_loss_enable, fixed_gain, ant_pat, attack_enabled,
               attack_cfg);

  // Per-sample: accumulate IQ from all channels
  for (isamp = 0; isamp < sample_count; isamp++) {
    int i_acc = 0;
    int q_acc = 0;

    for (i = 0; i < MAX_CHAN; i++) {
      if (chan[i].prn > 0) {
        attack_method_t attack_method = ATTACK_METHOD_NONE;
        if (attack_enabled == TRUE)
          attack_method = getAttackMethod(attack_cfg, chan[i].prn);

        iTable = cleanCarrierTableIndex(&chan[i]);
        if (attack_method == ATTACK_METHOD_JAM_NOISE) {
          unsigned int *state = &attack_noise_state[chan[i].prn - 1];
          double noise_amp = (double)gain[i] * jam_js_linear;
          double ni = (double)nextGaussianNoise(state) / JAM_NOISE_RMS_SCALE;
          double nq = (double)nextGaussianNoise(state) / JAM_NOISE_RMS_SCALE;

          ip = clipInt32FromDouble(ni * chan[i].codeCA * cosTable512[iTable] *
                                   noise_amp);
          qp = clipInt32FromDouble(nq * chan[i].codeCA * sinTable512[iTable] *
                                   noise_amp);
        } else {
          ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
          qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];
        }

        i_acc += ip;
        q_acc += qp;

        advanceCleanChannelSample(&chan[i], delt);
      }
    }

    i_acc = (i_acc + 64) >> 7;
    q_acc = (q_acc + 64) >> 7;

    *clipped_components +=
        (i_acc < -32768 || i_acc > 32767) + (q_acc < -32768 || q_acc > 32767);
    iq_buff[isamp * 2] = clipInt16(i_acc);
    iq_buff[isamp * 2 + 1] = clipInt16(q_acc);
  }
}

static void renderCleanEpochWide(double *iq_buff, int sample_count,
                                 channel_t chan[MAX_CHAN], int gain[MAX_CHAN],
                                 double delt) {
  int isamp;

  for (isamp = 0; isamp < sample_count; ++isamp) {
    int i_acc = 0;
    int q_acc = 0;
    int i;

    for (i = 0; i < MAX_CHAN; ++i) {
      int i_table;
      int ip;
      int qp;

      if (chan[i].prn <= 0)
        continue;
      i_table = cleanCarrierTableIndex(&chan[i]);
      ip = chan[i].dataBit * chan[i].codeCA * cosTable512[i_table] * gain[i];
      qp = chan[i].dataBit * chan[i].codeCA * sinTable512[i_table] * gain[i];
      i_acc += ip;
      q_acc += qp;

      advanceCleanChannelSample(&chan[i], delt);
    }

    iq_buff[2 * isamp] = (double)i_acc / 128.0;
    iq_buff[2 * isamp + 1] = (double)q_acc / 128.0;
  }
}

static bool captureMatchedTargetStates(
    const matched_code_plan_t &plan, const channel_t chan[MAX_CHAN],
    const int gain[MAX_CHAN], uint64_t sample_offset,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int channel = allocatedSat[prn - 1];

    if (channel < 0 || channel >= MAX_CHAN || chan[channel].prn != prn ||
        gain[channel] <= 0) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not allocated with positive clean gain at sample "
              << sample_offset;
      *error = message.str();
      return false;
    }
    matched_code_capture_channel_state(&chan[channel], gain[channel],
                                       sample_offset, &states[index]);
  }
  return true;
}

static bool validateMatchedTargetUsability(
    const matched_code_plan_t &plan, const ephem_t active_ephemeris[MAX_SAT],
    const synth_config_t *synth_config, gpstime_t boundary_time,
    double elevation_mask, uint64_t sample_offset, std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int satellite = prn - 1;
    bool synthetic = synth_config != nullptr && synth_config->enabled &&
                     synth_config->mode[satellite] != SYNTH_NONE;
    double azimuth_elevation[2];
    int usable =
        synthetic
            ? (active_ephemeris[satellite].vflg == 1 ? TRUE : FALSE)
            : checkSatVisibility(active_ephemeris[satellite], boundary_time,
                                 xyz[0], elevation_mask, azimuth_elevation);

    if (usable != TRUE) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not geometrically usable at sample " << sample_offset;
      *error = message.str();
      return false;
    }
  }
  return true;
}

static void writeTrajectoryHeader(std::ofstream &trajectory,
                                  const matched_code_plan_t &plan,
                                  gpstime_t start_time) {
  trajectory << std::setprecision(17);
  trajectory << "# schema=gps-sdr-sim.target-trajectory.v1\n";
  trajectory << "# sample_rate_hz=" << plan.sample_rate_hz << "\n";
  trajectory << "# epoch_cadence_samples="
             << static_cast<uint64_t>(
                    std::llround(plan.sample_rate_hz * EPOCH_TARGET_SEC))
             << "\n";
  trajectory << "# gps_week=" << start_time.week << "\n";
  trajectory << "# gps_tow=" << start_time.sec << "\n";
  trajectory << "# boundary=first_sample\n";
  trajectory << "sample_offset,prn,code_phase_chips,carrier_doppler_hz,"
                "code_rate_chips_per_s,clean_gain\n";
}

static void writeTrajectoryStates(std::ofstream &trajectory,
                                  const matched_code_target_state_t *states,
                                  size_t state_count) {
  for (size_t index = 0; index < state_count; ++index) {
    const matched_code_target_state_t &state = states[index];
    trajectory << state.sample_offset << ',' << state.prn << ','
               << state.code_phase_chips << ',' << state.carrier_doppler_hz
               << ',' << state.code_rate_chips_per_s << ',' << state.clean_gain
               << '\n';
  }
}

static bool rendersOnlyRevivedPrns(const attack_config_t &attack,
                                   const synth_config_t &synth) {
  if (!attack.partial_mode || !synth.enabled)
    return false;
  bool selected = false;
  for (int sv = 0; sv < MAX_SAT; ++sv) {
    if (!attack.prn_select[sv])
      continue;
    if (synth.mode[sv] != SYNTH_REVIVE)
      return false;
    selected = true;
  }
  return selected;
}

static void
refreshNavState(channel_t chan[MAX_CHAN], ephem_t eph[][MAX_SAT],
                const ephem_t synth_source[][MAX_SAT], int neph, int *ieph,
                ephem_t *active_eph, synth_ephem_store_t *synth_eph,
                const synth_config_t *synth_cfg, const ionoutc_t *ionoutc,
                gpstime_t grx, double elvmask, bool live_updated,
                const attack_config_t *attack_cfg, const int *required_prns) {
  int i;
  int eph_changed = live_updated;

  if (*ieph + 1 < neph) {
    gpstime_t next_toc;

    if (getSetReferenceToc(eph[*ieph + 1], &next_toc) == TRUE &&
        shouldAdvanceEphSet(next_toc, grx) == TRUE) {
      (*ieph)++;
      eph_changed = TRUE;
    }
  }

  if (synth_cfg->enabled) {
    gpstime_t synth_ref = quantizeSynthReferenceTime(grx);

    if (refreshSyntheticEphemerisSet(synth_eph, synth_source, neph, eph[*ieph],
                                     ionoutc, synth_cfg, xyz[0],
                                     synth_ref) == TRUE)
      eph_changed = TRUE;
  }

  if (eph_changed == TRUE) {
    overlaySyntheticEphemerisSet(active_eph, eph[*ieph], synth_cfg, synth_eph);

    for (i = 0; i < MAX_CHAN; i++) {
      if (chan[i].prn != 0)
        eph2sbf(active_eph[chan[i].prn - 1], *ionoutc, chan[i].sbf);
    }
  }

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0)
      generateNavMsg(grx, &chan[i], 0);
  }

  allocateChannel(chan, active_eph, *ionoutc, grx, xyz[0], elvmask, attack_cfg,
                  synth_cfg, required_prns);
}

static std::string jsonEscape(const std::string &value) {
  std::ostringstream escaped;

  for (unsigned char character : value) {
    switch (character) {
    case '"':
      escaped << "\\\"";
      break;
    case '\\':
      escaped << "\\\\";
      break;
    case '\b':
      escaped << "\\b";
      break;
    case '\f':
      escaped << "\\f";
      break;
    case '\n':
      escaped << "\\n";
      break;
    case '\r':
      escaped << "\\r";
      break;
    case '\t':
      escaped << "\\t";
      break;
    default:
      if (character < 0x20) {
        escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(character) << std::dec;
      } else {
        escaped << static_cast<char>(character);
      }
    }
  }
  return escaped.str();
}

static int checkGpsNow(LiveRun &live, const std::string &manifest) {
  ubx::GpsNowEstimate estimate;
  ubx::Snapshot state;
  std::string problem;
  bool ready = false;
  double now = getMonotonicSeconds();
  try {
    live.receiver =
        std::make_unique<ubx::Receiver>(live.endpoint, live.recording);
    const double deadline = now + live.warmup;
    for (;;) {
      state = live.snapshot();
      now = getMonotonicSeconds();
      if (!state.failure.empty()) {
        problem = state.failure;
        break;
      }
      if (stop_requested) {
        problem = "GPS time check interrupted";
        break;
      }
      try {
        estimate = ubx::estimateGpsNow(state, now, live.delivery);
        ready = true;
        problem.clear();
        break;
      } catch (const std::exception &e) {
        problem = e.what();
      }
      if (now >= deadline) {
        problem = "GPS time warmup timed out: " + problem;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
  } catch (const std::exception &e) {
    problem = e.what();
  }
  std::ostringstream report;
  report << std::setprecision(17)
         << "{\"schema\":\"gps-sdr-sim.ublox-gps-now.v1\",\"status\":\""
         << (ready ? "estimated" : "unavailable")
         << "\",\"epoch_association\":\"ublox_receiver_time_estimate\""
         << ",\"host_monotonic_s\":" << now << ",\"gps_now\":";
  if (ready)
    report << "{\"week\":" << estimate.gps.week
           << ",\"tow\":" << estimate.gps.sec
           << ",\"solution_age_s\":" << estimate.solution_age << '}';
  else
    report << "null";
  report << ",\"delivery_delay_estimate_s\":" << live.delivery
         << ",\"time_estimate\":"
         << (ready ? ubx::timeEstimateJson(estimate) : "null")
         << ",\"delays_calibrated\":false,\"absolute_gps_uncertainty_s\":null"
         << ",\"gps_alignment_verified\":false"
         << ",\"hardware_edge_association_verified\":false"
         << ",\"uhd_opened\":false,\"generated_samples\":0,\"send_attempted\":"
            "false"
         << ",\"receiver_endpoint\":\"" << jsonEscape(live.endpoint)
         << "\",\"ubx_recording\":\"" << jsonEscape(live.recording)
         << "\",\"failure_reason\":\"" << jsonEscape(problem)
         << "\",\"receiver\":" << ubx::snapshotJson(state, now) << '}';
  const std::string json = report.str();
  std::ofstream output(manifest);
  output << json << '\n';
  output.close();
  if (!output) {
    fprintf(stderr,
            "[GPS-NOW] Cannot write time report. UHD was never opened.\n");
    return 1;
  }
  printf("%s\n", json.c_str());
  fprintf(stderr,
          "[GPS-NOW] Receive-only time estimate; UHD was never opened.\n");
  if (!ready)
    fprintf(stderr, "[GPS-NOW] ERROR: %s\n", problem.c_str());
  return ready ? 0 : 1;
}

static uint64_t updateFnv1a64Sc16(uint64_t hash, const int16_t *samples,
                                  size_t sample_count) {
  for (size_t index = 0; index < sample_count * 2; ++index) {
    uint16_t value = static_cast<uint16_t>(samples[index]);
    hash ^= static_cast<uint8_t>(value & 0xffU);
    hash *= FNV1A64_PRIME;
    hash ^= static_cast<uint8_t>((value >> 8) & 0xffU);
    hash *= FNV1A64_PRIME;
  }
  return hash;
}

static std::string fnv1a64Hex(uint64_t value) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << value;
  return output.str();
}

static bool initializeMatchedJammerPlan(matched_code_plan_t *plan,
                                        double sample_rate_hz,
                                        const std::string &target_prns,
                                        double amplitude, uint64_t phase_seed,
                                        char *error, size_t error_size) {
  long double validation_samples_exact;
  long double validation_samples_rounded;

  if (plan == nullptr || !std::isfinite(sample_rate_hz) ||
      sample_rate_hz <= 0.0 || !std::isfinite(amplitude) || amplitude <= 0.0 ||
      amplitude > 1.0) {
    snprintf(error, error_size,
             "sample rate and matched-code amplitude in (0, 1] are required");
    return false;
  }

  memset(plan, 0, sizeof(*plan));
  plan->sample_rate_hz = sample_rate_hz;
  plan->phase_seed = phase_seed;
  plan->jammer_scale = amplitude;
  if (matched_code_parse_targets(target_prns.c_str(), plan->target_prns,
                                 &plan->target_count, error, error_size) != 0)
    return false;

  validation_samples_exact =
      static_cast<long double>(sample_rate_hz) *
      static_cast<long double>(MATCHED_DRY_RUN_VALIDATION_SEC);
  validation_samples_rounded = roundl(validation_samples_exact);
  if (validation_samples_rounded <= 0.0L ||
      validation_samples_rounded > static_cast<long double>(UINT64_MAX) ||
      fabsl(validation_samples_exact - validation_samples_rounded) > 1.0e-6L) {
    snprintf(error, error_size,
             "the 100 ms validation window is not an integral sample count");
    return false;
  }

  plan->total_samples = static_cast<uint64_t>(validation_samples_rounded);
  plan->onset_sample = 0;
  plan->offset_sample = plan->total_samples;
  plan->ramp_samples = 0;
  plan->reference_start_sample = 0;
  plan->reference_end_sample = plan->total_samples;
  plan->jammer_component_bound =
      amplitude * sqrt(static_cast<double>(plan->target_count));
  plan->predicted_composite_bound = plan->jammer_component_bound;
  plan->predicted_headroom_db =
      20.0 * log10(1.0 / plan->predicted_composite_bound);
  if (plan->predicted_headroom_db + 1.0e-12 < MATCHED_MIN_HEADROOM_DB) {
    snprintf(error, error_size,
             "matched-code amplitude leaves %.3f dB predicted headroom; at "
             "least %.1f dB is required for %zu target(s)",
             plan->predicted_headroom_db, MATCHED_MIN_HEADROOM_DB,
             plan->target_count);
    return false;
  }
  return true;
}

static bool artifactPathsAreDistinct(const char *navigation,
                                     const MatchedCodeOptions &options,
                                     std::vector<std::string> paths = {}) {
  if (navigation[0])
    paths.emplace_back(navigation);
  for (const auto &path : {options.manifest_path, options.trajectory_path}) {
    if (!path.empty()) {
      paths.push_back(path);
      paths.push_back(path + ".tmp");
    }
  }
  if (options.enabled)
    paths.push_back(options.manifest_path + ".probe");
  try {
    for (size_t i = 0; i < paths.size(); ++i) {
      auto resolved = std::filesystem::weakly_canonical(
          std::filesystem::absolute(paths[i]));
      for (size_t j = 0; j < i; ++j) {
        std::error_code error;
        if (resolved == std::filesystem::weakly_canonical(
                            std::filesystem::absolute(paths[j])) ||
            std::filesystem::equivalent(paths[i], paths[j], error)) {
          fprintf(stderr,
                  "ERROR: navigation input and artifacts must use different "
                  "paths: %s, %s\n",
                  paths[i].c_str(), paths[j].c_str());
          return false;
        }
      }
    }
  } catch (const std::filesystem::filesystem_error &e) {
    fprintf(stderr, "ERROR: cannot resolve artifact paths: %s\n", e.what());
    return false;
  }
  return true;
}

static void writeTransportJson(std::ostream &out, const x300::TxStats &stats) {
  out << "{\"generated_samples\": " << stats.generated
      << ", \"accepted_samples\": " << stats.accepted
      << ", \"underflows\": " << stats.underflows
      << ", \"sequence_errors\": " << stats.sequence_errors
      << ", \"time_errors\": " << stats.time_errors
      << ", \"other_errors\": " << stats.other_errors
      << ", \"data_send_calls\": " << stats.data_send_calls
      << ", \"partial_sample_sends\": " << stats.partial_sample_sends
      << ", \"max_samples_per_send\": " << stats.max_samples_per_send
      << ", \"maximum_send_s\": " << stats.maximum_send_seconds
      << ", \"zero_sample_sends\": " << stats.zero_sample_sends
      << ", \"prestart_backpressure_waits\": "
      << stats.prestart_backpressure_waits
      << ", \"burst_ack\": " << (stats.burst_ack ? "true" : "false")
      << ", \"start_margin_met\": "
      << (stats.start_margin_met ? "true" : "false")
      << ", \"operator_stopped\": " << (stats.interrupted ? "true" : "false")
      << ", \"maximum_render_s\": " << stats.maximum_render_seconds
      << ", \"prebuffer_s\": " << stats.prebuffer_seconds
      << ", \"send_attempted\": " << (stats.send_attempted ? "true" : "false")
      << ", \"first_send_hardware_s\": " << stats.first_send_hardware_seconds
      << ", \"generated_iq_fnv1a64\": \""
      << fnv1a64Hex(stats.generated_iq_fnv1a64) << "\""
      << ", \"minimum_queued_lead_s\": ";
  if (std::isfinite(stats.minimum_lead_seconds))
    out << stats.minimum_lead_seconds;
  else
    out << "null";
  out << ", \"first_event\": \"" << jsonEscape(stats.first_event)
      << "\", \"first_event_hardware_s\": ";
  if (stats.first_event_has_time)
    out << stats.first_event_seconds;
  else
    out << "null";
  out << ", \"first_event_observed_host_monotonic_s\": ";
  if (stats.first_event_observed_host_monotonic_seconds)
    out << *stats.first_event_observed_host_monotonic_seconds;
  else
    out << "null";
  out << ", \"accepted_samples_at_first_error\": "
      << stats.accepted_at_first_error << ", \"failure_reason\": \""
      << jsonEscape(stats.failure) << "\"}";
}

static bool writeRunManifest(const std::string &path, const std::string &status,
                             gpstime_t zero, double rate,
                             const char *clock_source, const char *time_source,
                             const x300::SyncState &sync,
                             const x300::TxStats &stats, uint64_t clipped,
                             const char *navfile, const std::string &nav_digest,
                             const double reference_xyz[3],
                             const LiveRun *live = nullptr) {
  const std::string temporary = path + ".tmp";
  std::ofstream out(temporary, std::ios::out | std::ios::trunc);
  if (!out)
    return false;
  out << std::setprecision(17)
      << "{\"schema\": \"gps-sdr-sim.x300tx-hardware-time.v1\", \"status\": \""
      << jsonEscape(status)
      << "\", \"gps_alignment_verified\": false, "
         "\"epoch_association\": \""
      << (live && live->enabled() && !live->replaying()
              ? (live->plan.pps.verified
                     ? "ublox_tim_tp_hardware_pps"
                     : (live->gps_pps ? "pps_association_pending"
                                      : (live->host_epoch_monotonic > 0
                                             ? "ublox_host_epoch_estimate"
                                             : "ublox_receiver_time_estimate")))
              : "scenario_at_sample_zero")
      << "\", \"pps_epoch_association_verified\": "
      << (live && live->plan.pps.verified ? "true" : "false")
      << ", \"rf_alignment_verified\": false"
      << ", \"pps_latch_verified\": " << (sync.verified ? "true" : "false")
      << ", \"hardware_pps_origin_s\": 0, \"hardware_start_s\": ";
  if (live && live->enabled() && !live->plan.valid)
    out << "null";
  else
    out << stats.start_seconds;
  out << ", \"sample_zero_gps_week\": " << zero.week
      << ", \"sample_zero_gps_tow\": " << zero.sec
      << ", \"sample_rate_hz\": " << rate << ", \"clock_source\": \""
      << jsonEscape(clock_source) << "\", \"time_source\": \""
      << jsonEscape(time_source) << "\", \"ephemeris_path\": \""
      << jsonEscape(navfile) << "\", \"ephemeris_sha256\": \"" << nav_digest
      << "\", \"reference_ecef_m\": [" << reference_xyz[0] << ','
      << reference_xyz[1] << ',' << reference_xyz[2] << ']'
      << ", \"clipped_components\": " << clipped << ", \"transport\": ";
  writeTransportJson(out, stats);
  if (live && live->enabled()) {
    out << ", \"navigation_source\": \""
        << (live->check_pps
                ? "none"
                : (live->time_only ? "frozen_rinex" : "ublox_gps_l1_sfrbx"))
        << "\", \"navigation_waveform\": \""
        << (live->check_pps
                ? "none"
                : (live->time_only ? "rinex_scenario_with_synthetic_overlays"
                                   : "reconstructed_lnav_not_bit_identical"))
        << "\", \"gps_time_source\": \""
        << (live->gps_pps ? "ublox_tim_tp_hardware_pps" : "ublox_nav_timegps")
        << "\", "
           "\"absolute_gps_uncertainty_s\": null, "
           "\"physical_rf_start_measured\": false, "
           "\"receiver_endpoint\": \""
        << jsonEscape(live->endpoint) << "\", \"ubx_recording\": \""
        << jsonEscape(live->recording) << "\", \"ubx_replay\": \""
        << jsonEscape(live->replay_path) << "\", \"device_address\": \""
        << jsonEscape(live->device_address) << "\", \"tx_subdevices\": \""
        << jsonEscape(live->tx_subdevices)
        << "\", \"tx_channel\": " << live->tx_channel
        << ", \"uhd_opened\": " << (live->uhd_opened ? "true" : "false")
        << ", \"receiver_configuration_changed\": false"
        << ", \"reference_position_source\": \""
        << (live->position_from_receiver ? "frozen_nav_pvt" : "explicit")
        << "\", \"live_start_plan\": "
        << (live->plan.valid ? x300::livePlanJson(live->plan) : "null")
        << ", \"pps_monitor\": "
        << (live->pps_association
                ? x300::ppsReferenceJson(live->pps_association->reference())
                : "null")
        << ", \"time_estimate\": "
        << (live->plan.estimate.observations
                ? ubx::timeEstimateJson(live->plan.estimate)
                : "null")
        << ", \"model_time_offset_s\": " << live->model_time_offset
        << ", \"model_time_offset_source\": \""
        << (live->model_time_offset_set ? "operator_supplied" : "none") << "\"";
    if (live->time_only) {
      out << ", \"time_reference_only\": true, \"host_epoch_monotonic_s\": ";
      if (live->host_epoch_monotonic > 0)
        out << live->host_epoch_monotonic;
      else
        out << "null";
      out << ", \"delivery_delay_estimate_s\": " << live->delivery
          << ", \"delays_calibrated\": false";
    }
    out << ", \"receiver_initial\": "
        << ubx::snapshotJson(live->initial, live->initial.last_received)
        << ", \"receiver_final\": ";
    auto current = live->snapshot();
    out << ubx::snapshotJson(current, live->replaying()
                                          ? current.last_received
                                          : getMonotonicSeconds());
    out << ", \"selected_prns\": [";
    for (size_t i = 0; i < live->prns.size(); ++i)
      out << (i ? "," : "") << live->prns[i];
    out << ']';
  }
  out << "}\n";
  out.close();
  if (!out || std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    return false;
  }
  return true;
}

static bool writeMatchedManifestAtomic(
    const MatchedCodeOptions &options, const matched_code_plan_t &plan,
    const MatchedCodeRunResult &result, const char *navfile,
    const std::string &ephemeris_sha256, const std::string &scenario_sha256,
    gpstime_t sample_zero, const double reference_xyz[3],
    double requested_rate_hz, const char *device_address, size_t tx_channel,
    const char *tx_antenna, double requested_gain_db, const char *clock_source,
    const char *time_source, int prebuffer_count) {
  std::string temporary_path = options.manifest_path + ".tmp";
  std::ofstream manifest(temporary_path, std::ios::out | std::ios::trunc);
  matched_code_source_config_t source_config{};
  matched_code_source_t phase_source;
  char source_error[128];

  if (!manifest)
    return false;
  source_config.sample_rate_hz = plan.sample_rate_hz;
  source_config.total_samples = plan.total_samples;
  source_config.onset_sample = plan.onset_sample;
  source_config.offset_sample = plan.offset_sample;
  source_config.ramp_samples = plan.ramp_samples;
  source_config.amplitude = plan.jammer_scale;
  source_config.phase_seed = plan.phase_seed;
  source_config.target_count = plan.target_count;
  for (size_t index = 0; index < plan.target_count; ++index)
    source_config.target_prns[index] = plan.target_prns[index];
  bool phases_available =
      matched_code_source_init(&phase_source, &source_config, source_error,
                               sizeof(source_error)) == 0;

  manifest << std::setprecision(17);
  manifest << "{\n";
  manifest << "  \"schema\": \"gps-sdr-sim.x300tx-matched-code.v3\",\n";
  manifest << "  \"tool\": \"x300tx\",\n";
  manifest << "  \"status\": \"" << jsonEscape(result.status) << "\",\n";
  manifest << "  \"failure_reason\": ";
  if (result.failure_reason.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(result.failure_reason) << "\"";
  manifest << ",\n";
  manifest << "  \"exit_status\": " << result.exit_status << ",\n";
  manifest << "  \"acceptance_scope\": "
              "\"transmitter-only; RF waveform fidelity, propagation, "
              "receiver-input power, RF code alignment, and receiver behavior "
              "were not measured\",\n";
  manifest << "  \"rf_output\": {\"contains\": "
              "\"matched_code_interference_only\", "
              "\"clean_gps_transmitted\": false},\n";
  manifest << "  \"safety\": {\"controlled_rf_only\": true, "
              "\"controlled_rf_confirmed\": "
           << (options.controlled_rf_confirmed ? "true" : "false")
           << ", \"calibration_id\": ";
  if (options.calibration_id.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(options.calibration_id) << "\"";
  manifest << "},\n";
  manifest << "  \"scenario\": {\"schema\": "
              "\"gps-sdr-sim.x300tx-frozen-scenario.v1\", "
              "\"sha256\": \""
           << jsonEscape(scenario_sha256) << "\", \"ephemeris_path\": \""
           << jsonEscape(navfile != nullptr ? navfile : "")
           << "\", \"ephemeris_sha256\": \"" << jsonEscape(ephemeris_sha256)
           << "\", \"reference_ecef_m\": [" << reference_xyz[0] << ", "
           << reference_xyz[1] << ", " << reference_xyz[2]
           << "], \"sample_zero_gps_week\": " << sample_zero.week
           << ", \"sample_zero_gps_tow\": " << sample_zero.sec
           << ", \"internal_alignment_source\": "
              "\"synthetic_clean_state_discarded_not_transmitted\""
           << ", \"requested_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"selected_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"startup_target_allocation_passed\": "
           << (result.target_allocation_passed ? "true" : "false") << "},\n";
  manifest << "  \"trajectory\": {\"schema\": "
              "\"gps-sdr-sim.target-trajectory.v1\", \"path\": \""
           << jsonEscape(options.trajectory_path) << "\", \"sha256\": \""
           << jsonEscape(result.trajectory_sha256) << "\"},\n";
  manifest << "  \"waveform\": {\"taxonomy\": "
              "\"navigation-data-free-gps-l1-ca-matched-code\", "
              "\"data_symbol_policy\": \"constant_positive\", "
              "\"carrier_phase_policy\": \"independent_deterministic\", "
              "\"phase_seed\": "
           << plan.phase_seed << ", \"equal_component_weight\": "
           << 1.0 / sqrt((double)plan.target_count)
           << ", \"target_count_normalization\": \"sqrt_n\", "
              "\"initial_carrier_phases_rad\": [";
  for (size_t index = 0; index < plan.target_count; ++index) {
    double phase =
        phases_available
            ? matched_code_source_initial_phase_rad(&phase_source, index)
            : 0.0;
    manifest << (index == 0 ? "" : ", ") << phase;
  }
  manifest << "], \"output_amplitude_full_scale\": " << options.amplitude
           << ", \"predicted_peak_full_scale\": "
           << plan.predicted_composite_bound
           << ", \"predicted_headroom_db\": " << plan.predicted_headroom_db
           << ", \"jammer_iq_fnv1a64\": ";
  if (result.quantized_samples == 0)
    manifest << "null";
  else
    manifest << "\"" << fnv1a64Hex(result.jammer_iq_fnv1a64) << "\"";
  manifest << "},\n";
  manifest << "  \"sample_contract\": {\"requested_rate_hz\": "
           << requested_rate_hz
           << ", \"actual_rate_hz\": " << result.actual_rate_hz
           << ", \"format\": \"sc16_le\", \"iq_order\": \"IQ\", "
              "\"continuous\": true, \"planned_samples\": null, "
              "\"dry_run_validation_samples\": "
           << plan.total_samples << ", \"internal_alignment_samples\": "
           << result.internal_alignment_samples
           << ", \"rendered_jammer_samples\": "
           << result.rendered_jammer_samples
           << ", \"quantized_jammer_samples\": " << result.quantized_samples
           << ", \"sent_jammer_samples\": " << result.sent_samples << "},\n";
  manifest << "  \"activation\": {\"start_sample\": 0, "
              "\"stop_condition\": \"SIGINT_or_SIGTERM\""
           << "},\n";
  manifest << "  \"timing\": {\"start_mode\": \""
           << jsonEscape(result.start_mode)
           << "\", \"epoch_association\": \"scenario_at_sample_zero\", "
              "\"gps_alignment_verified\": false, \"gps_week\": "
           << sample_zero.week << ", \"gps_tow\": " << sample_zero.sec
           << ", \"prebuffer_epochs\": " << prebuffer_count
           << ", \"pps_latch_verified\": "
           << (result.pps_verified ? "true" : "false")
           << ", \"uhd_start_time_s\": " << result.uhd_start_time_seconds
           << ", \"start_margin_met\": "
           << (result.start_margin_met ? "true" : "false") << "},\n";
  manifest << "  \"hardware\": {\"device_type\": \""
           << jsonEscape(result.device_type) << "\", \"product\": \""
           << jsonEscape(result.device_product) << "\", \"serial\": \""
           << jsonEscape(result.device_serial) << "\", \"address\": \""
           << jsonEscape(device_address)
           << "\", \"tx_channel_count\": " << result.tx_channel_count
           << ", \"selected_channel\": " << tx_channel
           << ", \"requested_antenna\": \"" << jsonEscape(tx_antenna)
           << "\", \"actual_antenna\": \"" << jsonEscape(result.actual_antenna)
           << "\", \"requested_center_frequency_hz\": " << TX_FREQUENCY
           << ", \"actual_center_frequency_hz\": " << result.actual_frequency_hz
           << ", \"clock_source\": \"" << jsonEscape(clock_source)
           << "\", \"time_source\": \"" << jsonEscape(time_source)
           << "\", \"requested_gain_db\": " << requested_gain_db
           << ", \"actual_gain_db\": " << result.actual_gain_db << "},\n";
  manifest << "  \"measurements\": {\"jammer_rms_full_scale\": "
           << result.source_metrics.active_plateau_rms
           << ", \"jammer_peak_full_scale\": "
           << result.source_metrics.peak_component
           << ", \"clipped_components\": "
           << result.source_metrics.clipped_components
           << ", \"underflows\": " << result.underflows
           << ", \"sequence_errors\": " << result.sequence_errors
           << ", \"time_errors\": " << result.time_errors
           << ", \"operator_stopped\": "
           << (result.interrupted ? "true" : "false") << "}\n";
  manifest << ", \"transport\": ";
  writeTransportJson(manifest, result.transport);
  manifest << "}\n";
  manifest.close();
  if (!manifest ||
      std::rename(temporary_path.c_str(), options.manifest_path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return false;
  }
  return true;
}

struct MatchedSimulationState {
  channel_t channels[MAX_CHAN];
  int gains[MAX_CHAN];
  ephem_t active_ephemeris[MAX_SAT];
  synth_ephem_store_t synthetic_ephemeris;
  int ephemeris_index = 0;
  epoch_plan_t epoch_plan{};
  uint64_t sample_offset = 0;
  gpstime_t receiver_time{};
  int64_t nav_frame = 0;
};

static void initializeMatchedSimulationState(
    MatchedSimulationState *state, const channel_t channels[MAX_CHAN],
    const int gains[MAX_CHAN], const ephem_t active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *synthetic_ephemeris, int ephemeris_index,
    const epoch_plan_t *epoch_plan, gpstime_t start_time) {
  memcpy(state->channels, channels, sizeof(state->channels));
  memcpy(state->gains, gains, sizeof(state->gains));
  memcpy(state->active_ephemeris, active_ephemeris,
         sizeof(state->active_ephemeris));
  state->synthetic_ephemeris = *synthetic_ephemeris;
  state->ephemeris_index = ephemeris_index;
  state->epoch_plan = *epoch_plan;
  state->sample_offset = 0;
  state->receiver_time = start_time;
  state->nav_frame = x300::navFrame(start_time);
}

static int nextMatchedFrameSampleCount(MatchedSimulationState *state,
                                       uint64_t total_samples) {
  uint64_t remaining = total_samples - state->sample_offset;
  int epoch_samples = nextEpochSampleCount(&state->epoch_plan);
  return remaining < (uint64_t)epoch_samples ? (int)remaining : epoch_samples;
}

static bool prepareMatchedFrame(
    MatchedSimulationState *state, const matched_code_plan_t &plan,
    uint64_t sample_limit, gpstime_t sample_zero, ionoutc_t *ionoutc,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    const synth_config_t *synth_config, double elevation_mask,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    int *sample_count, std::string *error) {
  gpstime_t block_start = getGpsTimeAtSampleOffset(
      sample_zero, (long long)state->sample_offset, plan.sample_rate_hz);
  gpstime_t block_end;

  *sample_count = nextMatchedFrameSampleCount(state, sample_limit);
  *sample_count = static_cast<int>(
      x300::capAtNavBoundary(block_start, plan.sample_rate_hz, *sample_count));
  if (!validateMatchedTargetUsability(plan, state->active_ephemeris,
                                      synth_config, block_start, elevation_mask,
                                      state->sample_offset, error))
    return false;
  block_end = getGpsTimeAtSampleOffset(
      sample_zero, (long long)(state->sample_offset + *sample_count),
      plan.sample_rate_hz);
  state->receiver_time = block_end;
  prepareEpoch(state->channels, state->gains, state->active_ephemeris, ionoutc,
               block_end, subGpsTime(block_end, block_start), delt,
               path_loss_enable, fixed_gain, ant_pat, FALSE, nullptr);
  return captureMatchedTargetStates(plan, state->channels, state->gains,
                                    state->sample_offset, states, error);
}

static void refreshMatchedStateIfNeeded(
    MatchedSimulationState *state, ephem_t eph[][MAX_SAT],
    const ephem_t synth_source[][MAX_SAT], int neph, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns) {
  int64_t frame = x300::navFrame(state->receiver_time);
  if (frame == state->nav_frame)
    return;
  if (frame != state->nav_frame + 1)
    throw std::runtime_error("navigation refresh skipped a GPS frame");
  state->nav_frame = frame;
  refreshNavState(state->channels, eph, synth_source, neph,
                  &state->ephemeris_index, state->active_ephemeris,
                  &state->synthetic_ephemeris, synth_config, ionoutc,
                  state->receiver_time, elevation_mask, false, attack_config,
                  required_prns);
}

static matched_code_source_config_t
matchedSourceConfig(const matched_code_plan_t &plan, uint64_t sample_limit) {
  matched_code_source_config_t config{};

  config.sample_rate_hz = plan.sample_rate_hz;
  config.total_samples = sample_limit;
  config.onset_sample = 0;
  config.offset_sample = sample_limit;
  config.ramp_samples = 0;
  config.amplitude = plan.jammer_scale;
  config.phase_seed = plan.phase_seed;
  config.target_count = plan.target_count;
  for (size_t index = 0; index < plan.target_count; ++index)
    config.target_prns[index] = plan.target_prns[index];
  return config;
}

static bool
renderMatchedFrame(MatchedSimulationState *state, matched_code_source_t *source,
                   const matched_code_plan_t &plan, uint64_t sample_limit,
                   gpstime_t sample_zero, ionoutc_t *ionoutc, double delt,
                   int path_loss_enable, int fixed_gain, double ant_pat[37],
                   const synth_config_t *synth_config, double elevation_mask,
                   std::vector<double> *alignment_discard,
                   std::vector<int16_t> *jammer_output,
                   std::ofstream *trajectory, MatchedCodeRunResult *result,
                   int *sample_count, std::string *error) {
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  char source_error[256];

  if (!prepareMatchedFrame(state, plan, sample_limit, sample_zero, ionoutc,
                           delt, path_loss_enable, fixed_gain, ant_pat,
                           synth_config, elevation_mask, states, sample_count,
                           error))
    return false;
  if (trajectory != nullptr)
    writeTrajectoryStates(*trajectory, states, plan.target_count);
  if (matched_code_source_set_epoch(source, states, plan.target_count,
                                    source_error, sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  alignment_discard->resize((size_t)*sample_count * 2);
  jammer_output->resize((size_t)*sample_count * 2);
  renderCleanEpochWide(alignment_discard->data(), *sample_count,
                       state->channels, state->gains, delt);
  if (matched_code_source_render_sc16(source, jammer_output->data(),
                                      (size_t)*sample_count) !=
      (size_t)*sample_count) {
    *error = "shared matched-code renderer stopped before the epoch ended";
    return false;
  }
  result->jammer_iq_fnv1a64 = updateFnv1a64Sc16(
      result->jammer_iq_fnv1a64, jammer_output->data(), (size_t)*sample_count);
  result->internal_alignment_samples += (uint64_t)*sample_count;
  result->rendered_jammer_samples += (uint64_t)*sample_count;
  result->quantized_samples += (uint64_t)*sample_count;
  matched_code_source_get_metrics(source, &result->source_metrics);
  state->sample_offset += (uint64_t)*sample_count;
  return true;
}

static bool runMatchedPreflight(
    matched_code_plan_t *plan, MatchedCodeRunResult *result,
    const MatchedCodeOptions &options,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double delt,
    int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  int saved_allocated[MAX_SAT];
  std::vector<double> alignment_discard;
  std::string trajectory_temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(trajectory_temporary,
                           std::ios::out | std::ios::trunc);

  if (!trajectory) {
    *error = "cannot create target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, *plan, sample_zero);
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);

  while (state.sample_offset < plan->total_samples) {
    int sample_count;
    if (!prepareMatchedFrame(&state, *plan, plan->total_samples, sample_zero,
                             ionoutc, delt, path_loss_enable, fixed_gain,
                             ant_pat, synth_config, elevation_mask, states,
                             &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    writeTrajectoryStates(trajectory, states, plan->target_count);
    alignment_discard.resize((size_t)sample_count * 2);
    renderCleanEpochWide(alignment_discard.data(), sample_count, state.channels,
                         state.gains, delt);
    state.sample_offset += (uint64_t)sample_count;
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns);
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory || std::rename(trajectory_temporary.c_str(),
                                 options.trajectory_path.c_str()) != 0) {
    std::remove(trajectory_temporary.c_str());
    *error = "cannot finalize target trajectory artifact";
    return false;
  }
  char trajectory_hash[SHA256_HEX_SIZE];
  if (sha256_file_hex(options.trajectory_path.c_str(), trajectory_hash) != 0) {
    *error = "cannot checksum target trajectory artifact";
    return false;
  }
  result->trajectory_sha256 = trajectory_hash;
  result->target_allocation_passed = true;
  return true;
}

static bool runMatchedDryRender(
    const matched_code_plan_t &plan, MatchedCodeRunResult *result,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double delt,
    int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  matched_code_source_config_t source_config =
      matchedSourceConfig(plan, plan.total_samples);
  matched_code_source_t source;
  char source_error[256];
  std::vector<double> alignment_discard;
  std::vector<int16_t> jammer_output;

  if (matched_code_source_init(&source, &source_config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);

  while (state.sample_offset < plan.total_samples) {
    int sample_count;
    if (!renderMatchedFrame(&state, &source, plan, plan.total_samples,
                            sample_zero, ionoutc, delt, path_loss_enable,
                            fixed_gain, ant_pat, synth_config, elevation_mask,
                            &alignment_discard, &jammer_output, nullptr, result,
                            &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns);
    if (result->source_metrics.clipped_components > 0) {
      *error = "unexpected SC16 clipping during dry-run render";
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  matched_code_source_get_metrics(&source, &result->source_metrics);
  return matched_code_source_done(&source) &&
         result->internal_alignment_samples == plan.total_samples &&
         result->rendered_jammer_samples == plan.total_samples &&
         result->quantized_samples == plan.total_samples;
}

static bool runMatchedTransmitter(
    const matched_code_plan_t &plan, const MatchedCodeOptions &options,
    MatchedCodeRunResult *result, const uhd::usrp::multi_usrp::sptr &usrp,
    const uhd::tx_streamer::sptr &stream, size_t max_send_samples,
    double requested_start_delay_seconds, const x300::SyncState &sync,
    size_t tx_channel, const std::function<void()> &on_start,
    int prebuffer_count, const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double delt,
    int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  const uint64_t limit = std::numeric_limits<uint64_t>::max();
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  auto config = matchedSourceConfig(plan, limit);
  matched_code_source_t source;
  char source_error[256];
  if (matched_code_source_init(&source, &config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  std::vector<double> scratch;
  std::vector<int16_t> output;
  const std::string temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(temporary, std::ios::out | std::ios::trunc);
  if (!trajectory) {
    *error = "cannot create live target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, plan, sample_zero);
  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);
  installSignalHandlers();
  auto render = [&](x300::Frame &frame) {
    int count;
    std::string failure;
    if (!renderMatchedFrame(&state, &source, plan, limit, sample_zero, ionoutc,
                            delt, path_loss_enable, fixed_gain, ant_pat,
                            synth_config, elevation_mask, &scratch, &output,
                            &trajectory, result, &count, &failure))
      throw std::runtime_error(failure);
    if (!trajectory)
      throw std::runtime_error("live target trajectory write failed");
    if (result->source_metrics.clipped_components)
      throw std::runtime_error("matched-code IQ clipped");
    frame.iq = output;
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns);
    return true;
  };
  UhdRadio radio(usrp, stream, tx_channel);
  bool success = x300::transmit(
      radio, sync, plan.sample_rate_hz, requested_start_delay_seconds,
      max_send_samples, prebuffer_count, render,
      [] { return stop_requested != 0; }, result->transport,
      [&] {
        result->uhd_start_time_seconds = result->transport.start_seconds;
        result->pps_verified = sync.verified;
        on_start();
        fprintf(stderr,
                "[TX] Local PPS start %.9f s maps to scenario %d:%.9f; "
                "GPS alignment unverified.\n",
                result->uhd_start_time_seconds, sample_zero.week,
                sample_zero.sec);
      });
  result->sent_samples = result->transport.accepted;
  result->start_margin_met = result->transport.start_margin_met;
  result->underflows = result->transport.underflows;
  result->sequence_errors = result->transport.sequence_errors;
  result->time_errors = result->transport.time_errors;
  result->interrupted = result->transport.interrupted;
  *error = result->transport.failure;
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory ||
      std::rename(temporary.c_str(), options.trajectory_path.c_str()) != 0) {
    *error = "cannot finalize live trajectory";
    return false;
  }
  char digest[SHA256_HEX_SIZE];
  if (sha256_file_hex(options.trajectory_path.c_str(), digest) != 0) {
    *error = "cannot checksum live trajectory";
    return false;
  }
  result->trajectory_sha256 = digest;
  return success && result->interrupted;
}

////////////////////////////////////////////////////////////
// Usage
////////////////////////////////////////////////////////////

static void x300_usage(void) {
  fprintf(
      stderr,
      "Usage: x300tx [options]\n\n"
      "Hardware timing (external 10 MHz and PPS required by default):\n"
      "  --addr <ip>                 X300 address (default 192.168.10.2)\n"
      "  --clock-source <src>        external/gpsdo (default external)\n"
      "  --time-source <src>         external/gpsdo (default external)\n"
      "  --check-sync                Check locks and latch PPS; no RF "
      "transmission\n"
      "  --start-lead-sec <seconds>  Minimum future margin, 0.02..60 (default "
      "0.25)\n"
      "  -r <seconds>               Alias for --start-lead-sec\n"
      "  --tx-advance-ns <ns>        Same margin in nanoseconds\n"
      "  --prebuffer <N>            Bounded render queue capacity in epochs "
      "(1..50)\n"
      "  --rate <Hz>                Requested sample rate (default 2500000)\n"
      "  --gain <dB>                TX gain (default 0)\n"
      "  --channel <N>              TX channel (default 0)\n"
      "  --antenna <name>           TX antenna (default TX/RX)\n\n"
      "Scenario generation:\n"
      "  -e <rinex_nav>             Navigation RINEX file\n"
      "  -l <lat,lon,alt>           Static location (degrees, degrees, "
      "metres)\n"
      "  -c <x,y,z>                 Static ECEF position (metres)\n"
      "  -t <YYYY/MM/DD,hh:mm:ss>  Scenario GPS calendar epoch\n"
      "  --gps-week <N> --gps-tow <s>  Scenario epoch at sample zero\n"
      "  -d <seconds>               Finite duration (default 300; dry-run "
      "0.1)\n"
      "  --stream                   Continue until SIGINT/SIGTERM\n"
      "  --dry-run                  Render offline; never open UHD\n"
      "  --manifest <file>          Run JSON (normal mode defaults to a unique "
      "file)\n"
      "  -P <list>                  Select clean constellation PRNs\n"
      "  -S <spec>                  Synthetic satellites: "
      "force/overhead/az-el/clone/revive\n"
      "  -A <spec> -J <dB> -G <dB>  Legacy scenario controls\n"
      "  -p [gain] -i -v           Fixed gain, disable ionosphere, verbose\n\n"
      "Matched-code interference (independent phase, no navigation data):\n"
      "  --matched-code-target-prns <list>  Sole transmitted PRN selector\n"
      "  --matched-code-amplitude <value>   Amplitude in (0, 1]\n"
      "  --matched-code-phase-seed <N>      Deterministic phase seed\n"
      "  --manifest <file>                  Required run JSON\n"
      "  --trajectory <file>                Per-PRN state artifact\n"
      "  --calibration-id <text> --confirm-controlled-rf  Live setup controls\n"
      "  Live mode requires explicit address/channel/antenna/gain and scenario "
      "epoch.\n"
      "  Continuous until interrupted; -P, -d, -n, -A, -J, -G are "
      "incompatible.\n\n"
      "ZED-F9P/F9K time and navigation (raw UBX over TCP):\n"
      "  --ublox-tcp <host:port>     Estimated live GPS startup, no Trimble\n"
      "  --ublox-time-tcp <host:port> F9P/F9K time with frozen -e; revive "
      "TX/stream/dry-run\n"
      "  --gps-pps                  Associate direct F9P/F9K PPS with TIM-TP; "
      "external 10 MHz\n"
      "  --check-pps                Verify PPS/GPS epoch only; resets local "
      "counter, no RF\n"
      "  --pps-host-utc-bound-sec <s> Required independent host UTC error "
      "bound (0..0.25]\n"
      "  --pps-max-delivery-sec <s> Label receipt bound after preceding PPS "
      "(default 0.8)\n"
      "  --check-ublox              Receiver-only probe (-d seconds, default "
      "10)\n"
      "  --check-time               Report estimated GPS now; receiver only, "
      "no IQ/UHD\n"
      "  --check-start              Render/prebuffer and check X300 deadline; "
      "no TX streamer\n"
      "  --ublox-warmup-sec <s>     Readiness timeout (default 60)\n"
      "  --ublox-record <file.ubx>  New raw UBX + .rx.csv files (default "
      "manifest.ubx)\n"
      "  --ublox-replay <file.ubx>  Recorded arrivals; only --dry-run or "
      "--check-ublox\n"
      "  --delivery-delay-sec <s>   Remaining delay of fastest time arrival "
      "(default 0)\n"
      "  --model-time-offset-sec <s> Signed empirical model epoch correction "
      "(+ advances; -1..1)\n"
      "  --tx-path-delay-sec <s> --sky-path-delay-sec <s>  Declared path "
      "delays\n"
      "  Live start lead defaults to 2 seconds; -P fixes selected GPS PRNs.\n"
      "  PVT position is frozen unless -l/-c is provided. TCP GPS offset is "
      "uncalibrated.\n\n"
      "Sample zero is assigned to a future PPS-referenced hardware second.\n"
      "Scenario time advances by N / actual configured sample rate. Ephemeris\n"
      "TOE/TOC are preserved. Local rubidium PPS does NOT label live GPS "
      "time.\n"
      "Host-clock -n, ppm scaling and estimated Trimble time-tag starts were "
      "removed.\n\n"
      "Examples:\n"
      "  x300tx --check-sync --addr 192.168.10.2\n"
      "  x300tx -e tests/fixtures/brdc0030.25n --dry-run -d 1 --manifest "
      "run.json\n"
      "  x300tx -e navigation.rnx -l 21.0047844,105.8460541,22 --stream\n\n");
}

////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  int sv, i;
  int neph, ieph;
  ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ephem_t revive_scan_eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ephem_t active_eph[MAX_SAT];
  synth_ephem_store_t synth_eph;
  gpstime_t g0;

  double llh[3];

  channel_t chan[MAX_CHAN];
  double elvmask = 0.0;

  int gain[MAX_CHAN];

  gpstime_t grx;
  double delt;

  int staticLocationMode = TRUE;
  int location_specified = FALSE;

  char navfile[MAX_CHAR];

  double samp_freq;

  int result;

  int fixed_gain = 128;
  double ant_pat[37];

  datetime_t t0{}, tmin;
  gpstime_t gmin;

  double duration;
  int iduration;
  int duration_specified = FALSE;
  int stream_mode = FALSE;
  int stream_forever = FALSE;

  int scenario_time_set = FALSE;
  int has_revive_mode = FALSE;
  int attack_enabled = FALSE;
  int partial_prns_set = FALSE;
  unsigned int attack_noise_state[MAX_SAT];
  double jam_js_linear = 10.0;

  ionoutc_t ionoutc;
  attack_config_t attack_cfg;
  synth_config_t synth_cfg;
  int path_loss_enable = TRUE;

  // X300-specific options
  int gps_week_set = FALSE;
  int gps_tow_set = FALSE;
  int explicit_gps_week = -1;
  double explicit_gps_tow = 0.0;
  long long tx_advance_ns = TX_START_LEAD_DEFAULT_NS;
  char usrp_addr[64] = "192.168.10.2";
  double requested_samp_freq;
  double tx_gain = 0.0;
  int tx_gain_set = FALSE;
  int compat_txvga1_set = FALSE;
  int compat_txvga2_set = FALSE;
  int compat_txvga1 = 0;
  int compat_txvga2 = 0;
  char clock_source[32] = "external";
  char time_source[32] = "external";
  bool check_sync = false;
  x300::SyncState sync_state;
  x300::TxStats tx_stats;
  int prebuffer_count = PREBUFFER_DEFAULT;
  size_t tx_channel = 0;
  char tx_antenna[32] = "TX/RX";
  MatchedCodeOptions matched_options;
  matched_code_plan_t matched_plan{};
  MatchedCodeRunResult matched_result;
  int matched_required_prns[MAX_SAT] = {0};
  std::string matched_scenario_sha256;
  std::string matched_ephemeris_sha256;

  LiveRun live;
  bool start_lead_set = false;

  epoch_plan_t epoch_plan;
  size_t max_samps = 0;
  uhd::usrp::multi_usrp::sptr usrp;
  uhd::tx_streamer::sptr tx_stream;

  ////////////////////////////////////////////////////////////
  // Parse options
  ////////////////////////////////////////////////////////////

  navfile[0] = 0;
  samp_freq = TX_SAMPLERATE;
  requested_samp_freq = TX_SAMPLERATE;
  g0.week = -1;
  g0.sec = 0.0;
  iduration = USER_MOTION_SIZE;
  duration = (double)iduration / 10.0;
  ionoutc.enable = TRUE;
  ionoutc.leapen = FALSE;
  initAttackConfig(&attack_cfg);
  initAttackNoiseState(attack_noise_state);
  initSynthConfig(&synth_cfg);
  initSynthEphemStore(&synth_eph);

  if (argc == 2 &&
      (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
    x300_usage();
    return 0;
  }

  if (argc < 2) {
    x300_usage();
    return 1;
  }

  std::vector<char *> short_argv;
  short_argv.push_back(argv[0]);

  for (i = 1; i < argc; i++) {
    if (strncmp(argv[i], "--", 2) != 0) {
      short_argv.push_back(argv[i]);
      continue;
    }

    const char *opt = argv[i] + 2;

    if (strcmp(opt, "help") == 0) {
      x300_usage();
      return 0;
    }
    if (strcmp(opt, "check-sync") == 0) {
      check_sync = true;
      continue;
    }
    if (strcmp(opt, "check-ublox") == 0) {
      live.check_receiver = true;
      continue;
    }
    if (strcmp(opt, "check-time") == 0) {
      live.check_time = true;
      continue;
    }
    if (strcmp(opt, "check-start") == 0) {
      live.check_start = true;
      continue;
    }
    if (strcmp(opt, "gps-pps") == 0) {
      live.gps_pps = true;
      continue;
    }
    if (strcmp(opt, "check-pps") == 0) {
      live.check_pps = true;
      live.gps_pps = true;
      continue;
    }
    if (strncmp(opt, "trimble-rtcm-", 13) == 0) {
      fprintf(stderr,
              "ERROR: --%s was removed from X300; use --ublox-tcp host:port "
              "(raw UBX).\n",
              opt);
      return 1;
    }
    if (strcmp(opt, "stream") == 0) {
      stream_mode = TRUE;
      continue;
    }
    if (strcmp(opt, "gps-time-ppm") == 0 ||
        strcmp(opt, "tx-time-scale-ppm") == 0 ||
        strncmp(opt, "trimble-time-tag-", 17) == 0 ||
        strcmp(opt, "trimble-start-offset-sec") == 0 ||
        strcmp(opt, "trimble-tag-lead-ms") == 0 ||
        strcmp(opt, "trimble-timeout-ms") == 0 ||
        strcmp(opt, "trimble-leap-sec") == 0 ||
        strcmp(opt, "trimble-tx-cal-ns") == 0) {
      fprintf(stderr,
              "ERROR: --%s was removed. X300 now uses external 10 MHz, "
              "PPS-latched hardware time and sample counts. Use an "
              "explicit scenario epoch and --stream if needed.\n",
              opt);
      return 1;
    }
    if (strcmp(opt, "dry-run") == 0) {
      matched_options.dry_run = true;
      continue;
    }
    if (strcmp(opt, "confirm-controlled-rf") == 0) {
      matched_options.controlled_rf_confirmed = true;
      continue;
    }

    if (i + 1 >= argc) {
      fprintf(stderr, "ERROR: Missing value for option --%s.\n", opt);
      return 1;
    }

    const char *val = argv[++i];

    if (strcmp(opt, "pps-host-utc-bound-sec") == 0 ||
        strcmp(opt, "pps-max-delivery-sec") == 0) {
      char *end = nullptr;
      errno = 0;
      const double value = strtod(val, &end);
      const bool host = strcmp(opt, "pps-host-utc-bound-sec") == 0;
      if (!*val || *end || errno == ERANGE || !std::isfinite(value) ||
          value <= (host ? 0 : .02) || value > (host ? .25 : .9)) {
        fprintf(stderr, "ERROR: invalid --%s.\n", opt);
        return 1;
      }
      if (host)
        live.pps_options.host_utc_bound = value;
      else
        live.pps_options.max_delivery = value;
      live.pps_bounds_set = true;
      continue;
    }

    if (strcmp(opt, "ublox-tcp") == 0 || strcmp(opt, "ublox-time-tcp") == 0) {
      const bool time_only = strcmp(opt, "ublox-time-tcp") == 0;
      if (!live.endpoint.empty() && live.time_only != time_only) {
        fprintf(stderr, "ERROR: --ublox-tcp and --ublox-time-tcp are mutually "
                        "exclusive.\n");
        return 1;
      }
      live.endpoint = val;
      live.time_only = time_only;
      continue;
    }
    if (strcmp(opt, "ublox-record") == 0) {
      live.recording = val;
      continue;
    }
    if (strcmp(opt, "ublox-replay") == 0) {
      live.replay_path = val;
      continue;
    }
    if (strcmp(opt, "model-time-offset-sec") == 0) {
      char *end = nullptr;
      errno = 0;
      double value = strtod(val, &end);
      if (!*val || *end || errno == ERANGE || !std::isfinite(value) ||
          std::fabs(value) > 1) {
        fprintf(stderr, "ERROR: invalid --model-time-offset-sec (allowed -1..1 "
                        "seconds).\n");
        return 1;
      }
      live.model_time_offset = value;
      live.model_time_offset_set = true;
      continue;
    }
    if (strcmp(opt, "ublox-warmup-sec") == 0 ||
        strcmp(opt, "delivery-delay-sec") == 0 ||
        strcmp(opt, "tx-path-delay-sec") == 0 ||
        strcmp(opt, "sky-path-delay-sec") == 0) {
      char *end = nullptr;
      errno = 0;
      double value = strtod(val, &end);
      const bool warmup = strcmp(opt, "ublox-warmup-sec") == 0;
      const double maximum =
          warmup ? 300 : (strcmp(opt, "delivery-delay-sec") == 0 ? 10 : 1);
      if (!*val || *end || errno == ERANGE || !std::isfinite(value) ||
          value < (warmup ? 1 : 0) || value > maximum) {
        fprintf(stderr, "ERROR: invalid --%s (allowed %g..%g seconds).\n", opt,
                warmup ? 1. : 0., maximum);
        return 1;
      }
      if (warmup)
        live.warmup = value;
      else if (strcmp(opt, "delivery-delay-sec") == 0)
        live.delivery = value;
      else if (strcmp(opt, "tx-path-delay-sec") == 0)
        live.tx_path = value;
      else
        live.sky_path = value;
      continue;
    }

    if (strcmp(opt, "gps-week") == 0) {
      char *end = NULL;
      errno = 0;
      long parsed = strtol(val, &end, 10);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          parsed < 0 || parsed > 65535) {
        fprintf(stderr, "ERROR: --gps-week must be an integer in 0..65535.\n");
        return 1;
      }
      explicit_gps_week = (int)parsed;
      gps_week_set = TRUE;
      continue;
    }
    if (strcmp(opt, "gps-tow") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed) || parsed < 0.0 || parsed >= 604800.0) {
        fprintf(stderr,
                "ERROR: --gps-tow must be a finite value in [0, 604800).\n");
        return 1;
      }
      explicit_gps_tow = parsed;
      gps_tow_set = TRUE;
      continue;
    }
    if (strcmp(opt, "start-lead-sec") == 0) {
      char *end = nullptr;
      errno = 0;
      double lead = strtod(val, &end);
      if (!val[0] || errno == ERANGE || *end || !std::isfinite(lead) ||
          lead < TX_START_LEAD_MIN_SEC || lead > TX_START_LEAD_MAX_SEC) {
        fprintf(stderr, "ERROR: --start-lead-sec must be 0.02..60 seconds.\n");
        return 1;
      }
      tx_advance_ns = llround(lead * 1e9);
      start_lead_set = true;
      continue;
    }
    if (strcmp(opt, "tx-advance-ns") == 0) {
      char *end = NULL;
      errno = 0;
      long long parsed = strtoll(val, &end, 10);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          parsed < 0 || parsed > (long long)(TX_START_LEAD_MAX_SEC * 1.0e9)) {
        fprintf(stderr, "ERROR: --tx-advance-ns must be 0-%.0f seconds.\n",
                TX_START_LEAD_MAX_SEC);
        return 1;
      }
      tx_advance_ns = parsed;
      start_lead_set = true;
      continue;
    }
    if (strcmp(opt, "addr") == 0) {
      strncpy(usrp_addr, val, sizeof(usrp_addr) - 1);
      usrp_addr[sizeof(usrp_addr) - 1] = '\0';
      matched_options.address_set = true;
      continue;
    }
    if (strcmp(opt, "rate") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed) || parsed < 1000000.0 ||
          parsed > 200000000.0) {
        fprintf(stderr, "ERROR: --rate must be in 1000000..200000000 Hz.\n");
        return 1;
      }
      samp_freq = parsed;
      requested_samp_freq = parsed;
      continue;
    }
    if (strcmp(opt, "gain") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed)) {
        fprintf(stderr, "ERROR: --gain must be a finite number of dB.\n");
        return 1;
      }
      tx_gain = parsed;
      tx_gain_set = TRUE;
      continue;
    }
    if (strcmp(opt, "txvga1") == 0) {
      compat_txvga1 = atoi(val);
      compat_txvga1_set = TRUE;
      if (compat_txvga1 < -35 || compat_txvga1 > -4) {
        fprintf(stderr, "ERROR: --txvga1 must be -35..-4 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "txvga2") == 0) {
      compat_txvga2 = atoi(val);
      compat_txvga2_set = TRUE;
      if (compat_txvga2 < 0 || compat_txvga2 > 25) {
        fprintf(stderr, "ERROR: --txvga2 must be 0..25 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "clock-source") == 0) {
      strncpy(clock_source, val, sizeof(clock_source) - 1);
      clock_source[sizeof(clock_source) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "time-source") == 0) {
      strncpy(time_source, val, sizeof(time_source) - 1);
      time_source[sizeof(time_source) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "prebuffer") == 0) {
      char *end = nullptr;
      errno = 0;
      long parsed = strtol(val, &end, 10);
      if (!val[0] || *end || errno == ERANGE || parsed < 1 || parsed > 50) {
        fprintf(stderr, "ERROR: prebuffer must be 1-50.\n");
        return 1;
      }
      prebuffer_count = static_cast<int>(parsed);
      continue;
    }
    if (strcmp(opt, "channel") == 0) {
      char *end = NULL;
      errno = 0;
      unsigned long parsed = strtoul(val, &end, 10);
      if (val[0] == '\0' || val[0] == '-' || errno == ERANGE || end == NULL ||
          *end != '\0') {
        fprintf(stderr, "ERROR: --channel must be a non-negative integer.\n");
        return 1;
      }
      tx_channel = (size_t)parsed;
      matched_options.channel_set = true;
      continue;
    }
    if (strcmp(opt, "antenna") == 0) {
      strncpy(tx_antenna, val, sizeof(tx_antenna) - 1);
      tx_antenna[sizeof(tx_antenna) - 1] = '\0';
      matched_options.antenna_set = true;
      continue;
    }
    if (strcmp(opt, "matched-code-target-prns") == 0) {
      matched_options.enabled = true;
      matched_options.target_prns = val;
      continue;
    }
    if (strcmp(opt, "matched-code-amplitude") == 0) {
      char *end = NULL;
      errno = 0;
      matched_options.amplitude = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(matched_options.amplitude) ||
          matched_options.amplitude <= 0.0 || matched_options.amplitude > 1.0) {
        fprintf(stderr, "ERROR: --matched-code-amplitude must be in (0, 1].\n");
        return 1;
      }
      matched_options.amplitude_set = true;
      continue;
    }
    if (strcmp(opt, "matched-code-phase-seed") == 0) {
      char *end = NULL;
      unsigned long long parsed;
      errno = 0;
      parsed = strtoull(val, &end, 10);
      if (val[0] == '\0' || val[0] == '-' || errno == ERANGE || end == NULL ||
          *end != '\0') {
        fprintf(
            stderr,
            "ERROR: --matched-code-phase-seed must be an unsigned integer.\n");
        return 1;
      }
      matched_options.phase_seed = (uint64_t)parsed;
      matched_options.phase_seed_set = true;
      continue;
    }
    if (strcmp(opt, "manifest") == 0) {
      matched_options.manifest_path = val;
      continue;
    }
    if (strcmp(opt, "trajectory") == 0) {
      matched_options.trajectory_path = val;
      continue;
    }
    if (strcmp(opt, "calibration-id") == 0) {
      matched_options.calibration_id = val;
      continue;
    }

    fprintf(stderr, "ERROR: Unknown option --%s.\n", opt);
    return 1;
  }

  short_argv.push_back(nullptr);
  optind = 1;

  while ((result = getopt((int)short_argv.size() - 1, short_argv.data(),
                          "e:l:c:t:d:P:S:A:J:G:r:ipvnh")) != -1) {
    switch (result) {
    case 'e':
      strncpy(navfile, optarg, MAX_CHAR - 1);
      navfile[MAX_CHAR - 1] = '\0';
      break;
    case 'l':
      location_specified = TRUE;
      staticLocationMode = TRUE;
      sscanf(optarg, "%lf,%lf,%lf", &llh[0], &llh[1], &llh[2]);
      llh[0] = llh[0] / R2D;
      llh[1] = llh[1] / R2D;
      llh2xyz(llh, xyz[0]);
      break;
    case 'c':
      location_specified = TRUE;
      staticLocationMode = TRUE;
      sscanf(optarg, "%lf,%lf,%lf", &xyz[0][0], &xyz[0][1], &xyz[0][2]);
      break;
    case 't':
      if (sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh,
                 &t0.mm, &t0.sec) != 6 ||
          !std::isfinite(t0.sec) || t0.y <= 1980 || t0.m < 1 || t0.m > 12 ||
          t0.d < 1 || t0.d > 31 || t0.hh < 0 || t0.hh > 23 || t0.mm < 0 ||
          t0.mm > 59 || t0.sec < 0.0 || t0.sec >= 60.0) {
        fprintf(stderr, "ERROR: Invalid date and time.\n");
        return 1;
      }
      date2gps(&t0, &g0);
      scenario_time_set = TRUE;
      break;
    case 'd': {
      duration_specified = TRUE;
      char *end = nullptr;
      errno = 0;
      duration = strtod(optarg, &end);
      if (!optarg[0] || *end || errno == ERANGE || !std::isfinite(duration) ||
          duration <= 0) {
        fprintf(stderr, "ERROR: Invalid duration.\n");
        return 1;
      }
      break;
    }
    case 'P':
      partial_prns_set = TRUE;
      if (parsePartialPrns(&attack_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid PRN list.\n");
        return 1;
      }
      break;
    case 'S':
      if (parseSynthConfig(&synth_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid synthetic spec.\n");
        return 1;
      }
      break;
    case 'A':
      if (parseAttackConfig(&attack_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid attack spec.\n");
        return 1;
      }
      attack_enabled = TRUE;
      break;
    case 'J':
      matched_options.legacy_js_set = true;
      attack_cfg.jam_js_db = atof(optarg);
      jam_js_linear = pow(10.0, attack_cfg.jam_js_db / 20.0);
      break;
    case 'G':
      matched_options.legacy_gain_boost_set = true;
      attack_cfg.gain_boost_db = atof(optarg);
      break;
    case 'r': {
      char *end = nullptr;
      double lead_sec = strtod(optarg, &end);
      if (!optarg[0] || *end || !std::isfinite(lead_sec) ||
          lead_sec < TX_START_LEAD_MIN_SEC ||
          lead_sec > TX_START_LEAD_MAX_SEC) {
        fprintf(stderr, "ERROR: TX start lead must be 0.02-%.0f seconds.\n",
                TX_START_LEAD_MAX_SEC);
        return 1;
      }
      tx_advance_ns = (long long)llround(lead_sec * 1.0e9);
      start_lead_set = true;
      break;
    }
    case 'p':
      if (optind < static_cast<int>(short_argv.size()) - 1 &&
          short_argv[optind][0] != '-') {
        fixed_gain = atoi(short_argv[optind]);
        if (fixed_gain < 1 || fixed_gain > 128) {
          fprintf(stderr, "ERROR: Fixed gain must be between 1 and 128.\n");
          return 1;
        }
        optind++;
      }
      path_loss_enable = FALSE;
      break;
    case 'i':
      ionoutc.enable = FALSE;
      break;
    case 'v':
      // Accepted for CLI compatibility; channel and timing details are always
      // logged.
      break;
    case 'n':
      fprintf(stderr,
              "ERROR: hardware-timed X300 does not accept -n; use "
              "--stream and a scenario epoch (-t or --gps-week/--gps-tow).\n");
      return 1;
    case 'h':
      x300_usage();
      return 0;
    case ':':
    case '?':
      x300_usage();
      return 1;
    default:
      break;
    }
  }

  installSignalHandlers();
  if (live.check_pps &&
      (live.check_start || navfile[0] || synth_cfg.enabled ||
       partial_prns_set || attack_enabled || gps_week_set || gps_tow_set ||
       scenario_time_set || stream_mode || duration_specified ||
       location_specified || tx_gain_set ||
       matched_options.controlled_rf_confirmed ||
       matched_options.amplitude_set || matched_options.phase_seed_set ||
       matched_options.legacy_js_set || matched_options.legacy_gain_boost_set ||
       !matched_options.trajectory_path.empty() ||
       !matched_options.calibration_id.empty() || live.tx_path ||
       live.sky_path)) {
    fprintf(stderr, "ERROR: --check-pps is a timing-only hardware check; "
                    "waveform options are incompatible.\n");
    return 1;
  }
  if (live.gps_pps &&
      (live.endpoint.empty() || live.replaying() || live.check_receiver ||
       live.check_time || check_sync || matched_options.dry_run ||
       matched_options.enabled || strcmp(clock_source, "external") != 0 ||
       strcmp(time_source, "external") != 0 ||
       live.pps_options.host_utc_bound <= 0 || live.delivery != 0 ||
       live.model_time_offset != 0)) {
    fprintf(stderr, "ERROR: --gps-pps requires live UBX, external clock/PPS "
                    "and explicit --pps-host-utc-bound-sec (0,0.25]; "
                    "arrival/model offsets, replay and offline checks are "
                    "incompatible. Use --check-start for no-TX validation.\n");
    return 1;
  }
  if (!live.gps_pps && live.pps_bounds_set) {
    fprintf(stderr, "ERROR: PPS bounds require --gps-pps.\n");
    return 1;
  }
  // Reject ignored corrections before receiver access, replay or UHD setup.
  if (live.model_time_offset_set &&
      (live.endpoint.empty() || live.replaying() || live.check_time ||
       live.check_receiver || check_sync || matched_options.enabled)) {
    fprintf(
        stderr,
        "ERROR: --model-time-offset-sec requires a live UBX waveform or "
        "--check-start; receiver-only checks and replay cannot apply it.\n");
    return 1;
  }
  // Time-only input supplies an epoch; target navigation stays in frozen RINEX.
  if (live.time_only && !live.check_pps) {
    if (live.check_receiver || live.check_time || check_sync ||
        live.replaying() || matched_options.enabled) {
      fprintf(stderr,
              "ERROR: --ublox-time-tcp supports TX, --stream, --check-start or "
              "--dry-run; replay and other checks are incompatible.\n");
      return 1;
    }
    if (!navfile[0] || !location_specified || !partial_prns_set) {
      fprintf(stderr, "ERROR: --ublox-time-tcp requires frozen -e, explicit "
                      "-l/-c and -P.\n");
      return 1;
    }
    if (hasCloneMode(&synth_cfg)) {
      fprintf(stderr,
              "ERROR: time-only mode does not supply live clone navigation.\n");
      return 1;
    }
    if (matched_options.dry_run &&
        (stream_mode || live.tx_path || live.sky_path)) {
      fprintf(stderr, "ERROR: time-only --dry-run requires finite rendering "
                      "without RF-path delay options.\n");
      return 1;
    }
  }
  if (live.check_time && (live.endpoint.empty() || live.replaying())) {
    fprintf(stderr, "ERROR: --check-time requires live --ublox-tcp; recordings "
                    "are not current time.\n");
    return 1;
  }
  if (live.check_time &&
      (live.check_receiver || live.check_start || check_sync ||
       matched_options.dry_run || matched_options.enabled || navfile[0] ||
       synth_cfg.enabled || partial_prns_set || attack_enabled ||
       gps_week_set || gps_tow_set || scenario_time_set || stream_mode ||
       duration_specified || location_specified || start_lead_set ||
       tx_gain_set || matched_options.controlled_rf_confirmed ||
       matched_options.amplitude_set || matched_options.phase_seed_set ||
       matched_options.legacy_js_set || matched_options.legacy_gain_boost_set ||
       !matched_options.trajectory_path.empty() ||
       !matched_options.calibration_id.empty() || live.tx_path ||
       live.sky_path)) {
    fprintf(stderr, "ERROR: --check-time is receiver-only; waveform/epoch "
                    "options and other checks are incompatible.\n");
    return 1;
  }
  if ((live.check_receiver || live.check_start || !live.recording.empty()) &&
      !live.enabled()) {
    fprintf(stderr, "ERROR: u-blox checks/recording require --ublox-tcp or "
                    "--ublox-replay.\n");
    return 1;
  }
  if ((!live.endpoint.empty() && live.replaying()) ||
      (live.check_receiver && live.check_start) ||
      (live.enabled() && check_sync) ||
      (live.check_start && (matched_options.dry_run || live.replaying()))) {
    fprintf(stderr,
            "ERROR: incompatible u-blox diagnostic/live/replay modes.\n");
    return 1;
  }
  if (live.enabled() && !live.replaying() &&
      (gps_week_set || gps_tow_set || scenario_time_set ||
       (matched_options.dry_run && !live.time_only))) {
    if (live.time_only)
      fprintf(stderr, "ERROR: UBX time-only mode chooses its future GPS epoch; "
                      "do not set -t or --gps-week/--gps-tow.\n");
    else
      fprintf(stderr, "ERROR: live UBX chooses its future GPS epoch; explicit "
                      "epochs/--dry-run require --ublox-replay.\n");
    return 1;
  }
  if (live.replaying() && !live.check_receiver &&
      (!matched_options.dry_run || !gps_week_set || !gps_tow_set)) {
    fprintf(stderr, "ERROR: recorded UBX requires --dry-run and explicit "
                    "--gps-week/--gps-tow; it is never fresh live time.\n");
    return 1;
  }
  if (live.enabled()) {
    live.device_address = usrp_addr;
    live.tx_channel = tx_channel;
    if (matched_options.manifest_path.empty())
      matched_options.manifest_path = "x300tx-ublox-" +
                                      std::to_string(time(nullptr)) + "-" +
                                      std::to_string(getpid()) + ".json";
    if (!live.replaying() && live.recording.empty())
      live.recording = matched_options.manifest_path + ".ubx";
    std::vector<std::string> paths;
    if (!live.recording.empty()) {
      paths.push_back(live.recording);
      paths.push_back(live.recording + ".rx.csv");
    }
    if (live.replaying()) {
      paths.push_back(live.replay_path);
      paths.push_back(live.replay_path + ".rx.csv");
    }
    if (!artifactPathsAreDistinct(navfile, matched_options, paths))
      return 1;
  }
  if (live.enabled() && !start_lead_set)
    tx_advance_ns = 2000000000LL;
  // This diagnostic returns before any ephemeris, renderer or UHD access.
  if (live.check_time)
    return checkGpsNow(live, matched_options.manifest_path);
  if (live.check_receiver) {
    try {
      if (live.replaying())
        live.initial = ubx::replay(live.replay_path);
      else {
        if (live.recording.empty())
          live.recording =
              "ublox-probe-" + std::to_string(time(nullptr)) + ".ubx";
        live.receiver =
            std::make_unique<ubx::Receiver>(live.endpoint, live.recording);
        double deadline =
            getMonotonicSeconds() + (duration_specified ? duration : 10);
        while (!stop_requested && getMonotonicSeconds() < deadline) {
          live.initial = live.snapshot();
          if (!live.initial.failure.empty())
            throw std::runtime_error(live.initial.failure);
          std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        live.initial = live.snapshot();
      }
      auto &s = live.initial;
      double now = live.replaying() ? s.last_received : getMonotonicSeconds();
      std::string report = ubx::snapshotJson(s, now);
      if (!matched_options.manifest_path.empty()) {
        std::ofstream output(matched_options.manifest_path);
        output << report << '\n';
        output.close();
        if (!output)
          throw std::runtime_error("cannot write receiver probe report");
      }
      printf("%s\n", report.c_str());
      fprintf(stderr, "[UBX] Receiver-only probe; UHD was never opened.\n");
      auto problem = ubx::readiness(s, now, s.time.gps, {}, false);
      if (!problem.empty())
        throw std::runtime_error(problem);
      return 0;
    } catch (const std::exception &e) {
      fprintf(stderr, "[UBX] ERROR: %s\n", e.what());
      return 1;
    }
  }

  if ((strcmp(clock_source, "external") != 0 &&
       strcmp(clock_source, "gpsdo") != 0) ||
      (strcmp(time_source, "external") != 0 &&
       strcmp(time_source, "gpsdo") != 0)) {
    fprintf(stderr, "ERROR: hardware sync requires external or gpsdo "
                    "clock/time sources.\n");
    return 1;
  }
  if (tx_advance_ns < (long long)(TX_START_LEAD_MIN_SEC * 1e9)) {
    fprintf(stderr, "ERROR: TX start lead must be at least 20 ms.\n");
    return 1;
  }
  if (check_sync) {
    if (matched_options.dry_run || matched_options.enabled) {
      fprintf(stderr,
              "ERROR: --check-sync cannot be combined with waveform modes.\n");
      return 1;
    }
    try {
      uhd::device_addr_t address;
      address["type"] = "x300";
      address["addr"] = usrp_addr;
      auto device = uhd::usrp::multi_usrp::make(address);
      configureClock(device, clock_source, time_source);
      UhdRadio radio(device);
      auto sync = x300::synchronize(radio);
      fprintf(stderr,
              "[SYNC] PASS: ref_locked=true, PPS latch verified, "
              "last PPS=%.9f s. No TX streamer created; no RF transmitted.\n"
              "[SYNC] Local hardware epoch only; GPS alignment unverified.\n",
              sync.last_pps_seconds);
      return 0;
    } catch (const std::exception &e) {
      fprintf(stderr, "[SYNC] FAIL: %s\n", e.what());
      return 1;
    }
  }

  if (live.check_pps) {
    x300::SyncState sync;
    x300::TxStats stats;
    gpstime_t zero{-1, 0};
    live.time_only = true;
    bool success = false;
    try {
      live.receiver =
          std::make_unique<ubx::Receiver>(live.endpoint, live.recording);
      const double deadline = getMonotonicSeconds() + live.warmup;
      for (;;) {
        live.initial = live.snapshot();
        const auto problem = ubx::readiness(live.initial, getMonotonicSeconds(),
                                            live.initial.time.gps, {}, false);
        if (problem.empty())
          break;
        if (stop_requested || !live.initial.failure.empty() ||
            getMonotonicSeconds() >= deadline)
          throw std::runtime_error("UBX warmup: " + problem);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      uhd::device_addr_t address;
      address["type"] = "x300";
      address["addr"] = usrp_addr;
      auto device = uhd::usrp::multi_usrp::make(address);
      live.uhd_opened = true;
      configureClock(device, clock_source, time_source);
      UhdRadio radio(device);
      sync = x300::synchronize(radio);
      zero = prepareLiveStart(live, &radio, tx_advance_ns * 1e-9);
      stats.start_seconds = live.plan.hardware_start;
      x300::checkHealth(radio, sync);
      live.checkPps(radio);
      success = true;
      fprintf(stderr, "[PPS-CHECK] PASS: GPS epoch associated; no TX streamer "
                      "or RF transmission.\n");
    } catch (const std::exception &e) {
      stats.failure = e.what();
      fprintf(stderr, "[PPS-CHECK] FAIL: %s\n", e.what());
    }
    if (!writeRunManifest(matched_options.manifest_path,
                          success ? "pps_checked" : "failed", zero, 0,
                          clock_source, time_source, sync, stats, 0, "", "",
                          xyz[0], &live)) {
      fprintf(stderr, "ERROR: cannot write PPS check manifest.\n");
      return 1;
    }
    return success ? 0 : 1;
  }

  // Explicit GPS week/TOW overrides the scenario calendar epoch from -t.
  if (gps_week_set && gps_tow_set) {
    g0.week = explicit_gps_week;
    g0.sec = explicit_gps_tow;
    gps2date(&g0, &t0);
    scenario_time_set = TRUE;
    fprintf(stderr, "[TIMING] Using explicit GPS epoch: week %d  tow %.3f\n",
            g0.week, g0.sec);
  } else if (gps_week_set || gps_tow_set) {
    fprintf(stderr,
            "ERROR: --gps-week and --gps-tow must both be specified.\n");
    return 1;
  }

  if (hasCloneMode(&synth_cfg) == TRUE && !live.navigation_from_ubx()) {
    fprintf(stderr, "ERROR: Clone mode requires --ublox-tcp.\n");
    return 1;
  }
  has_revive_mode = hasReviveMode(&synth_cfg);
  if (has_revive_mode && (navfile[0] == 0 || live.navigation_from_ubx())) {
    fprintf(stderr, "ERROR: Revive mode requires frozen -e ephemeris.\n");
    return 1;
  }
  if (!navfile[0] && !live.enabled()) {
    fprintf(stderr, "ERROR: Navigation RINEX file is required (-e) unless "
                    "--ublox-tcp or --ublox-replay is set.\n");
    return 1;
  }
  if (navfile[0] && live.navigation_from_ubx()) {
    fprintf(stderr,
            "ERROR: -e and u-blox navigation input are mutually exclusive.\n");
    return 1;
  }

  // Default location if none specified
  if (!location_specified && !live.enabled()) {
    llh[0] = 35.681298 / R2D;
    llh[1] = 139.766247 / R2D;
    llh[2] = 10.0;
    llh2xyz(llh, xyz[0]);
  }

  stream_forever = (stream_mode == TRUE && staticLocationMode == TRUE &&
                    duration_specified == FALSE);

  if (!matched_options.enabled &&
      (matched_options.controlled_rf_confirmed ||
       matched_options.amplitude_set || matched_options.phase_seed_set ||
       !matched_options.trajectory_path.empty() ||
       !matched_options.calibration_id.empty())) {
    fprintf(stderr, "ERROR: matched-code-only options require "
                    "--matched-code-target-prns.\n");
    return 1;
  }

  if (matched_options.enabled) {
    char plan_error[256] = "";

    if (!matched_options.amplitude_set || !matched_options.phase_seed_set ||
        matched_options.manifest_path.empty()) {
      fprintf(stderr, "ERROR: matched-code mode requires target PRNs, output "
                      "amplitude, phase seed, and --manifest.\n");
      return 1;
    }
    if (duration_specified) {
      fprintf(stderr,
              "ERROR: continuous matched-code mode does not accept -d.\n");
      return 1;
    }
    if (partial_prns_set == TRUE) {
      fprintf(stderr,
              "ERROR: matched-code jammer-only mode rejects -P; "
              "--matched-code-target-prns is the sole jammer selector.\n");
      return 1;
    }
    if (attack_enabled || matched_options.legacy_js_set ||
        matched_options.legacy_gain_boost_set) {
      fprintf(stderr,
              "ERROR: matched-code mode is mutually exclusive with -A, -J, "
              "and -G legacy attack controls.\n");
      return 1;
    }
    if (live.enabled()) {
      fprintf(stderr,
              "ERROR: matched-code mode requires frozen RINEX ephemeris; "
              "u-blox live/replay navigation is not supported in this mode.\n");
      return 1;
    }
    if (fabs(samp_freq * EPOCH_TARGET_SEC -
             round(samp_freq * EPOCH_TARGET_SEC)) > 1.0e-6) {
      fprintf(stderr,
              "ERROR: matched-code mode requires a rate with an integer "
              "100 ms epoch sample count.\n");
      return 1;
    }
    if (!initializeMatchedJammerPlan(
            &matched_plan, samp_freq, matched_options.target_prns,
            matched_options.amplitude, matched_options.phase_seed, plan_error,
            sizeof(plan_error))) {
      fprintf(stderr, "ERROR: invalid matched-code plan: %s\n", plan_error);
      return 1;
    }
    for (size_t target = 0; target < matched_plan.target_count; ++target) {
      int prn = matched_plan.target_prns[target];
      matched_required_prns[prn - 1] = 1;
    }
    if (matched_options.trajectory_path.empty()) {
      std::string base = matched_options.manifest_path;
      if (base.size() >= 5 && base.substr(base.size() - 5) == ".json")
        base.resize(base.size() - 5);
      matched_options.trajectory_path = base + ".trajectory.csv";
    }
    if (!artifactPathsAreDistinct(navfile, matched_options))
      return 1;
    {
      std::error_code manifest_path_error;
      std::error_code trajectory_path_error;
      std::filesystem::path manifest_absolute = std::filesystem::absolute(
          matched_options.manifest_path, manifest_path_error);
      std::filesystem::path trajectory_absolute = std::filesystem::absolute(
          matched_options.trajectory_path, trajectory_path_error);
      if (manifest_path_error || trajectory_path_error) {
        fprintf(stderr,
                "ERROR: matched-code artifact paths cannot be resolved.\n");
        return 1;
      }
      if (manifest_absolute.lexically_normal() ==
          trajectory_absolute.lexically_normal()) {
        fprintf(stderr, "ERROR: --manifest and --trajectory must use different "
                        "paths.\n");
        return 1;
      }
    }
    if (!matched_options.dry_run) {
      if (!matched_options.address_set || !matched_options.channel_set ||
          !matched_options.antenna_set || !tx_gain_set ||
          !std::isfinite(tx_gain) || matched_options.calibration_id.empty() ||
          !matched_options.controlled_rf_confirmed) {
        fprintf(stderr, "ERROR: live matched-code TX requires explicit --addr, "
                        "--channel, --antenna, --gain, --calibration-id, and "
                        "--confirm-controlled-rf.\n");
        return 1;
      }
      if (!(gps_week_set && gps_tow_set)) {
        fprintf(stderr,
                "ERROR: live matched-code TX requires an explicit scenario "
                "--gps-week/--gps-tow (not a live GPS time reference).\n");
        return 1;
      }
    }
    {
      std::string probe_path = matched_options.manifest_path + ".probe";
      std::ofstream probe(probe_path, std::ios::out | std::ios::trunc);
      if (!probe) {
        fprintf(stderr, "ERROR: manifest path is not writable: %s\n",
                matched_options.manifest_path.c_str());
        return 1;
      }
      probe.close();
      if (!probe || std::remove(probe_path.c_str()) != 0) {
        fprintf(stderr, "ERROR: manifest path cannot be finalized: %s\n",
                matched_options.manifest_path.c_str());
        return 1;
      }
    }
  }

  if (!matched_options.enabled) {
    if (matched_options.manifest_path.empty())
      matched_options.manifest_path = "x300tx-run-" +
                                      std::to_string(time(nullptr)) + "-" +
                                      std::to_string(getpid()) + ".json";
    if (!artifactPathsAreDistinct(navfile, matched_options))
      return 1;
  }

  if (matched_options.dry_run && !matched_options.enabled &&
      !duration_specified) {
    duration = EPOCH_TARGET_SEC;
    stream_forever = FALSE;
  }
  if (matched_options.dry_run && stream_forever) {
    fprintf(stderr, "ERROR: --dry-run requires a finite duration.\n");
    return 1;
  }
  if (!std::isfinite(duration) || (!stream_forever && duration <= 0.0)) {
    fprintf(stderr, "ERROR: Invalid duration.\n");
    return 1;
  }
  if (!stream_forever && duration > STATIC_MAX_DURATION && staticLocationMode) {
    fprintf(stderr, "ERROR: Invalid duration.\n");
    return 1;
  }

  if (!live.enabled() || location_specified) {
    xyz2llh(xyz[0], llh);
    fprintf(stderr, "xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1],
            xyz[0][2]);
    fprintf(stderr, "llh = %11.6f, %11.6f, %11.1f\n", llh[0] * R2D,
            llh[1] * R2D, llh[2]);
  }

  if (matched_options.enabled) {
    matched_result.status = "incomplete";
    matched_result.exit_status = 1;
    matched_result.device_address = usrp_addr;
    if (gps_week_set && gps_tow_set)
      matched_result.start_mode = "explicit_gps_time";
    else if (scenario_time_set)
      matched_result.start_mode = "explicit_gps_calendar_time";
    else
      matched_result.start_mode = "frozen_rinex_epoch";
    if (matched_options.dry_run) {
      matched_result.device_type = "not_opened";
      matched_result.device_product = "not_opened";
      matched_result.actual_rate_hz = requested_samp_freq;
      matched_result.actual_frequency_hz = TX_FREQUENCY;
      matched_result.actual_gain_db = tx_gain_set ? tx_gain : 0.0;
      matched_result.actual_antenna = tx_antenna;
    }
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count)) {
      fprintf(stderr,
              "ERROR: cannot write initial matched-code manifest before "
              "preflight.\n");
      return 1;
    }
  }

  auto finish_matched_early_failure = [&](const char *status,
                                          const std::string &reason) -> int {
    if (matched_options.enabled) {
      matched_result.status = status;
      matched_result.failure_reason = reason;
      matched_result.exit_status = 1;
      if (!writeMatchedManifestAtomic(
              matched_options, matched_plan, matched_result, navfile,
              matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
              requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
              clock_source, time_source, prebuffer_count)) {
        fprintf(stderr,
                "ERROR: cannot finalize failed matched-code manifest.\n");
      }
    } else {
      tx_stats.failure = reason;
      if (!writeRunManifest(matched_options.manifest_path, status, g0,
                            samp_freq, clock_source, time_source, sync_state,
                            tx_stats, 0, navfile, matched_ephemeris_sha256,
                            xyz[0], &live))
        fprintf(stderr, "ERROR: cannot persist failed run manifest.\n");
    }
    return 1;
  };

  ////////////////////////////////////////////////////////////
  // Read ephemeris
  ////////////////////////////////////////////////////////////

  if (live.enabled()) {
    try {
      if (live.replaying()) {
        live.initial = ubx::replay(live.replay_path);
      } else {
        if (live.recording.empty())
          live.recording = matched_options.manifest_path + ".ubx";
        live.receiver =
            std::make_unique<ubx::Receiver>(live.endpoint, live.recording);
      }
      const double deadline = getMonotonicSeconds() + live.warmup;
      std::string problem;
      do {
        live.initial = live.snapshot();
        live.prns.clear();
        for (int prn = 1; prn <= MAX_SAT; ++prn)
          if (partial_prns_set && attack_cfg.prn_select[prn - 1])
            live.prns.push_back(synth_cfg.mode[prn - 1] == SYNTH_CLONE
                                    ? synth_cfg.source_prn[prn - 1]
                                    : prn);
        if (!partial_prns_set)
          live.prns = ubx::observedPrns(live.initial);
        for (int sv = 0; sv < MAX_SAT; ++sv)
          if (synth_cfg.mode[sv] == SYNTH_CLONE)
            live.prns.push_back(synth_cfg.source_prn[sv]);
        std::sort(live.prns.begin(), live.prns.end());
        live.prns.erase(std::unique(live.prns.begin(), live.prns.end()),
                        live.prns.end());
        double now = live.replaying() ? live.initial.last_received
                                      : getMonotonicSeconds();
        gpstime_t epoch = live.replaying() ? g0 : live.initial.time.gps;
        problem = ubx::readiness(live.initial, now, epoch, live.prns,
                                 live.navigation_from_ubx());
        if (problem.empty())
          break;
        if (!live.initial.failure.empty() || live.replaying() || stop_requested)
          throw std::runtime_error(problem);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      } while (getMonotonicSeconds() < deadline);
      if (!problem.empty())
        throw std::runtime_error("UBX warmup: " + problem);
      if (live.navigation_from_ubx()) {
        memset(eph, 0, sizeof(eph));
        for (int prn : live.prns)
          eph[0][prn - 1] = live.initial.navigation[prn - 1].eph;
        neph = 1;
        gmin = live.initial.time.gps;
        gps2date(&gmin, &tmin);
        // Missing ionosphere/UTC pages stay explicitly unavailable. Do not
        // invent a leap offset or rebase the received ephemeris epochs.
        ionoutc = {};
        if (live.initial.time.leap_valid)
          ionoutc.dtls = live.initial.time.leap_seconds;
      }
      if (!location_specified) {
        llh[0] = live.initial.position.latitude / R2D;
        llh[1] = live.initial.position.longitude / R2D;
        llh[2] = live.initial.position.height;
        llh2xyz(llh, xyz[0]);
        live.position_from_receiver = true;
      }
      fprintf(stderr,
              "[UBX] Frozen model location %.9f,%.9f,%.3f m; receiver reports "
              "%.3f m/s.\n",
              llh[0] * R2D, llh[1] * R2D, llh[2], live.initial.position.speed);
      if (live.time_only)
        fprintf(stderr,
                "[UBX] Time reference ready: %s; target navigation comes from "
                "frozen RINEX.\n",
                live.initial.identity.c_str());
      else
        fprintf(stderr,
                "[UBX] Ready: %zu GPS L1 C/A ephemerides, %s; position %s.\n",
                live.prns.size(), live.initial.identity.c_str(),
                live.position_from_receiver ? "frozen from NAV-PVT"
                                            : "explicit");
    } catch (const std::exception &e) {
      fprintf(stderr, "[UBX] ERROR: %s\n", e.what());
      return finish_matched_early_failure("receiver_error", e.what());
    }
  }
  if (!live.navigation_from_ubx()) {
    neph = readRinexNavAll(eph, &ionoutc, navfile);
    if (neph == 0) {
      fprintf(stderr, "ERROR: No ephemeris available.\n");
      return finish_matched_early_failure(
          "preflight_error", "frozen navigation file contains no ephemeris");
    } else if (neph == -1) {
      fprintf(stderr, "ERROR: Ephemeris file not found.\n");
      return finish_matched_early_failure(
          "preflight_error", "frozen navigation file was not found");
    }

    if (has_revive_mode == TRUE)
      memcpy(revive_scan_eph, eph, sizeof(revive_scan_eph));

    for (sv = 0; sv < MAX_SAT; sv++) {
      if (eph[0][sv].vflg == 1) {
        gmin = eph[0][sv].toc;
        tmin = eph[0][sv].t;
        break;
      }
    }
  }

  if (!live.navigation_from_ubx()) {
    char digest[SHA256_HEX_SIZE];
    if (sha256_file_hex(navfile, digest) != 0) {
      fprintf(stderr, "ERROR: cannot checksum frozen navigation file '%s'.\n",
              navfile);
      return finish_matched_early_failure(
          "preflight_error", "cannot checksum frozen navigation file");
    }
    matched_ephemeris_sha256 = digest;
  }

  // Select the scenario label before opening UHD. This never rebases TOE/TOC.
  if (g0.week < 0) {
    g0 = gmin;
    t0 = tmin;
  }
  if (!matched_options.enabled &&
      !writeRunManifest(matched_options.manifest_path, "incomplete", g0,
                        samp_freq, clock_source, time_source, sync_state,
                        tx_stats, 0, navfile, matched_ephemeris_sha256, xyz[0],
                        &live)) {
    fprintf(stderr, "ERROR: cannot write run manifest before opening UHD.\n");
    return 1;
  }

  ////////////////////////////////////////////////////////////
  // Configure USRP X300
  ////////////////////////////////////////////////////////////

  if (matched_options.dry_run) {
    fprintf(stderr,
            "\n[DRY-RUN] UHD discovery/open/configuration is intentionally "
            "skipped.\n");
    matched_result.device_type = "not_opened";
    matched_result.device_product = "not_opened";
    matched_result.device_address = usrp_addr;
    matched_result.actual_rate_hz = samp_freq;
    matched_result.actual_frequency_hz = TX_FREQUENCY;
    matched_result.actual_gain_db = tx_gain_set ? tx_gain : 0.0;
    matched_result.actual_antenna = tx_antenna;
  } else {
    fprintf(stderr, "\n[UHD] Configuring USRP X300 at %s ...\n", usrp_addr);

    try {
      uhd::device_addr_t dev_addr;
      dev_addr["addr"] = usrp_addr;
      dev_addr["type"] = "x300";
      if (matched_options.enabled) {
        uhd::device_addrs_t discovered = uhd::device::find(dev_addr);
        if (discovered.size() != 1) {
          fprintf(stderr,
                  "ERROR: matched-code preflight found %zu UHD devices at "
                  "explicit address %s; exactly one is required.\n",
                  discovered.size(), usrp_addr);
          return finish_matched_early_failure(
              "device_error",
              "device discovery did not return exactly one device");
        }
        std::string discovered_type =
            discovered[0].has_key("type") ? discovered[0]["type"] : "";
        std::string discovered_product =
            discovered[0].has_key("product") ? discovered[0]["product"] : "";
        if (discovered_type != "x300" ||
            (discovered_product != "X300" && discovered_product != "X310")) {
          fprintf(stderr,
                  "ERROR: matched-code mode requires an X300/X310, discovered "
                  "type='%s' product='%s'.\n",
                  discovered_type.c_str(), discovered_product.c_str());
          return finish_matched_early_failure(
              "device_error", "discovered device is not an X300/X310");
        }
        matched_result.device_type = discovered_type;
        matched_result.device_product = discovered_product;
        matched_result.device_serial =
            discovered[0].has_key("serial") ? discovered[0]["serial"] : "";
        matched_result.device_address =
            discovered[0].has_key("addr") ? discovered[0]["addr"] : usrp_addr;
        fprintf(stderr, "[UHD] Discovered %s serial %s at %s\n",
                discovered_product.c_str(),
                matched_result.device_serial.c_str(),
                matched_result.device_address.c_str());
      }

      usrp = uhd::usrp::multi_usrp::make(dev_addr);
      live.uhd_opened = true;
      if (live.enabled())
        live.tx_subdevices = usrp->get_tx_subdev_spec().to_string();

      configureClock(usrp, clock_source, time_source);

      {
        size_t tx_channels = usrp->get_tx_num_channels();
        matched_result.tx_channel_count = tx_channels;
        if (tx_channel >= tx_channels) {
          fprintf(stderr,
                  "ERROR: --channel %zu is invalid; device reports %zu TX "
                  "channel(s).\n",
                  tx_channel, tx_channels);
          return finish_matched_early_failure(
              "device_error", "selected TX channel does not exist");
        }
      }

      fprintf(stderr, "[UHD] TX channel: %zu\n", tx_channel);

      usrp->set_tx_rate(samp_freq, tx_channel);
      fprintf(stderr, "[UHD] TX rate:    %.0f Hz (actual %.0f Hz)\n", samp_freq,
              usrp->get_tx_rate(tx_channel));

      if (!live.check_start) {
        uhd::tune_request_t tune_req(TX_FREQUENCY);
        usrp->set_tx_freq(tune_req, tx_channel);
        fprintf(stderr, "[UHD] TX freq:    %.0f Hz (actual %.0f Hz)\n",
                TX_FREQUENCY, usrp->get_tx_freq(tx_channel));

        if (compat_txvga1_set || compat_txvga2_set) {
          fprintf(
              stderr,
              "[UHD] BladeRF gain option(s) accepted for CLI compatibility:");
          if (compat_txvga1_set)
            fprintf(stderr, " txvga1=%d", compat_txvga1);
          if (compat_txvga2_set)
            fprintf(stderr, " txvga2=%d", compat_txvga2);
          fprintf(stderr, ". X300 RF gain is controlled by --gain");
          if (tx_gain_set == FALSE)
            fprintf(stderr, " (using default %.1f dB)", tx_gain);
          fprintf(stderr, ".\n");
        }

        {
          uhd::gain_range_t gain_range = usrp->get_tx_gain_range(tx_channel);
          double min_gain = gain_range.start();
          double max_gain = gain_range.stop();

          if (matched_options.enabled &&
              (tx_gain < min_gain || tx_gain > max_gain)) {
            fprintf(
                stderr,
                "ERROR: matched-code TX gain %.1f dB is outside device range "
                "[%.1f, %.1f] dB.\n",
                tx_gain, min_gain, max_gain);
            return finish_matched_early_failure(
                "device_error",
                "requested TX gain is outside the device range");
          }
          if (tx_gain < min_gain) {
            fprintf(stderr,
                    "[UHD] WARNING: requested TX gain %.1f dB is below device "
                    "minimum %.1f dB; clamping.\n",
                    tx_gain, min_gain);
            tx_gain = min_gain;
          } else if (tx_gain > max_gain) {
            fprintf(stderr,
                    "[UHD] WARNING: requested TX gain %.1f dB is above device "
                    "maximum %.1f dB; clamping.\n",
                    tx_gain, max_gain);
            tx_gain = max_gain;
          }
        }

        usrp->set_tx_gain(tx_gain, tx_channel);
        fprintf(stderr, "[UHD] TX gain:    %.1f dB (actual %.1f dB)\n", tx_gain,
                usrp->get_tx_gain(tx_channel));

        usrp->set_tx_antenna(tx_antenna, tx_channel);
        fprintf(stderr, "[UHD] TX antenna: %s\n",
                usrp->get_tx_antenna(tx_channel).c_str());
        fprintf(stderr, "[UHD] Clock src:  %s\n", clock_source);
        fprintf(stderr, "[UHD] Time src:   %s\n", time_source);

        // Create TX streamer — SC16 on both CPU and wire side
        uhd::stream_args_t stream_args("sc16", "sc16");
        stream_args.channels = {tx_channel};
        tx_stream = usrp->get_tx_stream(stream_args);

        max_samps = tx_stream->get_max_num_samps();
        fprintf(stderr, "[UHD] Max samples per packet: %zu\n", max_samps);
      }
      samp_freq = usrp->get_tx_rate(tx_channel);
      if (matched_options.enabled &&
          fabs(samp_freq - matched_plan.sample_rate_hz) >
              MATCHED_RATE_TOLERANCE_HZ) {
        fprintf(stderr,
                "ERROR: actual TX rate %.9f differs from requested %.9f by "
                "more than %.1f Hz.\n",
                samp_freq, matched_plan.sample_rate_hz,
                MATCHED_RATE_TOLERANCE_HZ);
        return finish_matched_early_failure(
            "device_error", "actual TX rate differs from the requested rate");
      }
      UhdRadio radio(usrp, tx_stream, tx_channel);
      sync_state = x300::synchronize(radio);
      matched_result.pps_verified = sync_state.verified;
      fprintf(stderr,
              "[SYNC] PPS time-zero latch and subsequent second verified. "
              "GPS alignment is unverified.\n");
      matched_result.actual_rate_hz = samp_freq;
      matched_result.actual_frequency_hz = usrp->get_tx_freq(tx_channel);
      matched_result.actual_gain_db = usrp->get_tx_gain(tx_channel);
      matched_result.actual_antenna = usrp->get_tx_antenna(tx_channel);
    } catch (const std::exception &e) {
      fprintf(stderr, "ERROR: Failed to configure USRP: %s\n", e.what());
      return finish_matched_early_failure("device_error", e.what());
    }
  }

  delt = 1.0 / samp_freq;
  initEpochPlan(&epoch_plan, samp_freq);
  if (matched_options.enabled) {
    char plan_error[256] = "";
    if (fabs(samp_freq * EPOCH_TARGET_SEC -
             round(samp_freq * EPOCH_TARGET_SEC)) > 1.0e-6 ||
        !initializeMatchedJammerPlan(
            &matched_plan, samp_freq, matched_options.target_prns,
            matched_options.amplitude, matched_options.phase_seed, plan_error,
            sizeof(plan_error))) {
      fprintf(stderr,
              "ERROR: actual-rate matched-code plan is unsupported: %s\n",
              plan_error[0] != '\0' ? plan_error : "non-integral 100 ms epoch");
      return finish_matched_early_failure(
          "preflight_error", "actual-rate matched-code plan is unsupported");
    }
  }

  fprintf(stderr,
          "[TIMING] Generator sample rate uses %s: "
          "%.6f Hz\n",
          matched_options.dry_run ? "requested offline rate" : "actual TX rate",
          samp_freq);
  if (epoch_plan.max_samples == epoch_plan.base_samples) {
    fprintf(stderr, "[TIMING] Epoch sample count: %d samples every %.1f ms\n",
            epoch_plan.base_samples, EPOCH_TARGET_SEC * 1000.0);
  } else {
    fprintf(stderr,
            "[TIMING] Epoch sample count dithers between %d and %d samples "
            "to match the actual TX rate.\n",
            epoch_plan.base_samples, epoch_plan.max_samples);
  }

  ////////////////////////////////////////////////////////////
  // Resolve GPS start time
  ////////////////////////////////////////////////////////////

  if (live.enabled() && !live.replaying()) {
    try {
      if (matched_options.dry_run) {
        g0 = prepareLiveStart(live, nullptr, tx_advance_ns * 1e-9);
      } else {
        UhdRadio radio(usrp, {}, tx_channel);
        g0 = prepareLiveStart(live, &radio, tx_advance_ns * 1e-9);
        tx_stats.start_seconds = live.plan.hardware_start;
      }
      if (live.navigation_from_ubx())
        for (int prn : live.prns)
          eph[0][prn - 1] = live.initial.navigation[prn - 1].eph;
    } catch (const std::exception &e) {
      return finish_matched_early_failure("start_plan_error", e.what());
    }
  }
  gps2date(&g0, &t0);

  fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%09.6f (%d:%.9f)\n",
          t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
  if (matched_options.enabled)
    fprintf(stderr, "Duration = continuous until SIGINT/SIGTERM\n");
  else if (stream_forever)
    fprintf(stderr, "Duration = streaming until interrupted\n");
  else
    fprintf(stderr, "Duration = %.1f [sec]\n", duration);

  if (matched_options.enabled) {
    std::ostringstream scenario;
    scenario << std::setprecision(17) << matched_ephemeris_sha256 << '|'
             << xyz[0][0] << '|' << xyz[0][1] << '|' << xyz[0][2] << '|'
             << g0.week << '|' << g0.sec << '|' << matched_plan.sample_rate_hz
             << "|continuous|" << matched_options.target_prns << '|'
             << matched_options.amplitude << '|' << matched_options.phase_seed
             << '|';
    for (sv = 0; sv < MAX_SAT; ++sv) {
      scenario << (int)synth_cfg.mode[sv] << ':' << synth_cfg.source_prn[sv]
               << ':' << synth_cfg.azimuth[sv] << ':' << synth_cfg.elevation[sv]
               << ';';
    }
    std::string canonical = scenario.str();
    char digest[SHA256_HEX_SIZE];
    sha256_bytes_hex(canonical.data(), canonical.size(), digest);
    matched_scenario_sha256 = digest;
  }

  ////////////////////////////////////////////////////////////
  // Select ephemeris set
  ////////////////////////////////////////////////////////////

  if (live.navigation_from_ubx()) {
    ieph = 0;
  } else if (rendersOnlyRevivedPrns(attack_cfg, synth_cfg)) {
    // Every selected output is replaced by its validated historical template
    // below. The base set is only backing storage for the overlay; no authentic
    // ephemeris from it is transmitted, so its TOC need not match the live
    // epoch. Revive still scans the complete frozen archive, independently of
    // ieph.
    ieph = 0;
    fprintf(stderr,
            "[NAV] All selected PRNs use revive; current-set freshness "
            "is not required. Historical templates will be validated.\n");
  } else {
    ieph = -1;
    for (i = 0; i < neph; i++) {
      gpstime_t ref_toc;
      if (getSetReferenceToc(eph[i], &ref_toc) == TRUE &&
          shouldAdvanceEphSet(ref_toc, g0) == TRUE) {
        ieph = i;
        break;
      }
    }
    if (ieph == -1) {
      fprintf(stderr, "ERROR: No current ephemeris set found.\n");
      return finish_matched_early_failure(
          "preflight_error", "no current frozen ephemeris set was found");
    }
  }

  ////////////////////////////////////////////////////////////
  // Build synthetic satellite overlay
  ////////////////////////////////////////////////////////////

  if (synth_cfg.enabled) {
    gpstime_t synth_ref = quantizeSynthReferenceTime(g0);
    const ephem_t(*synth_source)[MAX_SAT] =
        has_revive_mode == TRUE ? revive_scan_eph : eph;

    for (sv = 0; sv < MAX_SAT; sv++) {
      if (synth_cfg.mode[sv] == SYNTH_REVIVE) {
        ephem_t revive_template;
        gpstime_t template_toe;
        double delta_sec;
        double elev_deg;
        int found_ephem = FALSE;

        if (scanEphemerisForRevive(
                synth_source, neph, sv + 1, synth_ref, xyz[0], &revive_template,
                &template_toe, &delta_sec, &elev_deg, &found_ephem) == FALSE) {
          if (found_ephem == TRUE) {
            fprintf(stderr,
                    "ERROR: Revive PRN %d: not above %.1f deg at any point "
                    "in lookback window.\n",
                    sv + 1, SYNTH_REVIVE_MIN_ELEVATION_DEG);
          } else {
            fprintf(stderr,
                    "ERROR: Revive PRN %d: no ephemeris found within %.1fh "
                    "lookback.\n",
                    sv + 1, SYNTH_REVIVE_MAX_LOOKBACK_SEC / 3600.0);
          }
          return finish_matched_early_failure(
              "preflight_error", "revive target scenario is not usable");
        }
      } else if (synth_cfg.mode[sv] == SYNTH_OVERHEAD ||
                 synth_cfg.mode[sv] == SYNTH_AZEL) {
        double az, el;

        if (synth_cfg.mode[sv] == SYNTH_OVERHEAD) {
          az = 0.0;
          el = PI / 2.0;
        } else {
          az = synth_cfg.azimuth[sv];
          el = synth_cfg.elevation[sv];
        }

        {
          double test_sat[3];

          if (synthAzelReachable(xyz[0], az, el, test_sat) == FALSE) {
            fprintf(stderr,
                    "WARNING: PRN %d az=%.1f el=%.1f deg lies outside the "
                    "synthetic GPS inclination envelope at this location. "
                    "Skipping.\n",
                    sv + 1, az * R2D, el * R2D);
            synth_cfg.mode[sv] = SYNTH_NONE;
            continue;
          }
        }

        fprintf(stderr, "Synthetic PRN %02d: az=%.1f el=%.1f deg\n", sv + 1,
                az * R2D, el * R2D);
      } else if (synth_cfg.mode[sv] == SYNTH_FORCE) {
        if (eph[ieph][sv].vflg != 1) {
          fprintf(stderr,
                  "WARNING: PRN %d forced but no ephemeris. Skipping.\n",
                  sv + 1);
          synth_cfg.mode[sv] = SYNTH_NONE;
        } else {
          fprintf(stderr, "Forced PRN %02d (below-horizon bypass)\n", sv + 1);
        }
      }
    }

    refreshSyntheticEphemerisSet(&synth_eph, synth_source, neph, eph[ieph],
                                 &ionoutc, &synth_cfg, xyz[0], synth_ref);
  }

  overlaySyntheticEphemerisSet(active_eph, eph[ieph], &synth_cfg, &synth_eph);

  ////////////////////////////////////////////////////////////
  // Allocate channels (each producer frame owns its IQ storage).
  ////////////////////////////////////////////////////////////

  for (i = 0; i < MAX_CHAN; i++)
    chan[i].prn = 0;
  for (sv = 0; sv < MAX_SAT; sv++)
    allocatedSat[sv] = -1;

  grx = g0;
  allocateChannel(chan, active_eph, ionoutc, grx, xyz[0], elvmask, &attack_cfg,
                  &synth_cfg,
                  matched_options.enabled ? matched_required_prns : nullptr);

  if (live.time_only) {
    for (int prn : live.prns) {
      if (allocatedSat[prn - 1] < 0)
        return finish_matched_early_failure(
            "preflight_error", "selected RINEX target PRN " +
                                   std::to_string(prn) + " was not allocated");
    }
  }

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0)
      fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
              chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d,
              chan[i].rho0.iono_delay);
  }

  // Antenna gain pattern
  for (i = 0; i < 37; i++)
    ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

  if (attack_enabled) {
    fprintf(stderr, "Attack config:\n");
    for (sv = 0; sv < MAX_SAT; sv++) {
      if (attack_cfg.method[sv] != ATTACK_METHOD_NONE)
        fprintf(stderr, "  PRN %02d -> %s\n", sv + 1,
                attackMethodName(attack_cfg.method[sv]));
    }
  }
  if (attack_cfg.partial_mode) {
    fprintf(stderr, "Partial constellation mode: rendering only PRNs");
    for (sv = 0; sv < MAX_SAT; sv++)
      if (attack_cfg.prn_select[sv])
        fprintf(stderr, " %d", sv + 1);
    fprintf(stderr, "\n");
    if (attack_cfg.gain_boost_db != 0.0)
      fprintf(stderr, "  Power boost: +%.1f dB\n", attack_cfg.gain_boost_db);
  }

  if (matched_options.enabled) {
    const ephem_t(*matched_synth_source)[MAX_SAT] =
        has_revive_mode == TRUE ? revive_scan_eph : eph;
    std::string matched_error;

    matched_result.status = "incomplete";
    matched_result.exit_status = 1;
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count)) {
      fprintf(stderr,
              "ERROR: cannot write incomplete matched-code manifest.\n");
      return 1;
    }

    fprintf(stderr,
            "\n[MATCHED] Jammer-only preflight: targets=%s validation-samples="
            "%llu amplitude=%.9f predicted-headroom=%.3f dB\n",
            matched_options.target_prns.c_str(),
            (unsigned long long)matched_plan.total_samples,
            matched_options.amplitude, matched_plan.predicted_headroom_db);
    if (!runMatchedPreflight(&matched_plan, &matched_result, matched_options,
                             chan, gain, active_eph, &synth_eph, ieph,
                             &epoch_plan, eph, matched_synth_source, neph, g0,
                             &ionoutc, &synth_cfg, &attack_cfg, elvmask,
                             matched_required_prns, delt, path_loss_enable,
                             fixed_gain, ant_pat, &matched_error)) {
      matched_result.status = "preflight_error";
      matched_result.failure_reason = matched_error;
      matched_result.exit_status = 1;
      writeMatchedManifestAtomic(
          matched_options, matched_plan, matched_result, navfile,
          matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
          requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
          clock_source, time_source, prebuffer_count);
      fprintf(stderr, "ERROR: matched-code preflight failed: %s\n",
              matched_error.c_str());
      return 1;
    }
    fprintf(stderr,
            "[MATCHED] Clean simulator IQ is internal alignment state only and "
            "will be discarded before the RF adapter.\n");

    if (matched_options.dry_run) {
      if (runMatchedDryRender(
              matched_plan, &matched_result, chan, gain, active_eph, &synth_eph,
              ieph, &epoch_plan, eph, matched_synth_source, neph, g0, &ionoutc,
              &synth_cfg, &attack_cfg, elvmask, matched_required_prns, delt,
              path_loss_enable, fixed_gain, ant_pat, &matched_error)) {
        matched_result.status = "dry_run";
        matched_result.exit_status = 0;
      } else {
        matched_result.status =
            matched_result.source_metrics.clipped_components > 0
                ? "clipping"
                : "preflight_error";
        matched_result.failure_reason = matched_error;
        matched_result.exit_status = 1;
      }
      matched_result.transport.generated = matched_result.quantized_samples;
      matched_result.transport.failure = matched_result.failure_reason;
      if (!writeMatchedManifestAtomic(
              matched_options, matched_plan, matched_result, navfile,
              matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
              requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
              clock_source, time_source, prebuffer_count)) {
        fprintf(stderr, "ERROR: cannot finalize dry-run manifest.\n");
        return 1;
      }
      fprintf(
          stderr,
          "[DRY-RUN] status=%s rendered=%llu clipping=%llu; UHD was "
          "never opened.\n",
          matched_result.status.c_str(),
          (unsigned long long)matched_result.quantized_samples,
          (unsigned long long)matched_result.source_metrics.clipped_components);
      return matched_result.exit_status;
    }

    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count)) {
      fprintf(stderr,
              "ERROR: cannot update matched-code manifest before arming.\n");
      return 1;
    }

    bool completed = false;
    try {
      completed = runMatchedTransmitter(
          matched_plan, matched_options, &matched_result, usrp, tx_stream,
          max_samps, (double)tx_advance_ns * 1.0e-9, sync_state, tx_channel,
          [&] {
            if (!writeMatchedManifestAtomic(
                    matched_options, matched_plan, matched_result, navfile,
                    matched_ephemeris_sha256, matched_scenario_sha256, g0,
                    xyz[0], requested_samp_freq, usrp_addr, tx_channel,
                    tx_antenna, tx_gain, clock_source, time_source,
                    prebuffer_count))
              throw std::runtime_error(
                  "cannot persist armed matched-code manifest");
          },
          prebuffer_count, chan, gain, active_eph, &synth_eph, ieph,
          &epoch_plan, eph, matched_synth_source, neph, g0, &ionoutc,
          &synth_cfg, &attack_cfg, elvmask, matched_required_prns, delt,
          path_loss_enable, fixed_gain, ant_pat, &matched_error);
    } catch (const std::exception &transmit_error) {
      matched_error = std::string("matched-code transmitter failed: ") +
                      transmit_error.what();
    } catch (...) {
      matched_error = "matched-code transmitter failed unexpectedly";
    }
    if (completed) {
      matched_result.status = "stopped";
      matched_result.exit_status = 0;
    } else {
      matched_result.failure_reason = matched_error;
      matched_result.exit_status = 1;
      if (matched_result.time_errors > 0)
        matched_result.status = "time_error";
      else if (matched_result.sequence_errors > 0)
        matched_result.status = "sequence_error";
      else if (matched_result.underflows > 0)
        matched_result.status = "underflow";
      else if (matched_result.source_metrics.clipped_components > 0)
        matched_result.status = "clipping";
      else if (matched_result.interrupted)
        matched_result.status = "interrupted";
      else if (matched_error.find("stale") != std::string::npos)
        matched_result.status = "preflight_error";
      else
        matched_result.status = "device_error";
    }
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count)) {
      fprintf(stderr, "ERROR: cannot finalize matched-code run manifest.\n");
      return 1;
    }
    fprintf(
        stderr,
        "[TX] Matched-code status=%s sent=%llu underflows=%llu "
        "sequence-errors=%llu time-errors=%llu clipping=%llu\n",
        matched_result.status.c_str(),
        (unsigned long long)matched_result.sent_samples,
        (unsigned long long)matched_result.underflows,
        (unsigned long long)matched_result.sequence_errors,
        (unsigned long long)matched_result.time_errors,
        (unsigned long long)matched_result.source_metrics.clipped_components);
    return matched_result.exit_status;
  }

  uint64_t generated = 0;
  uint64_t clipped = 0;
  const uint64_t sample_limit =
      stream_forever ? std::numeric_limits<uint64_t>::max()
                     : static_cast<uint64_t>(llround(duration * samp_freq));
  int64_t nav_frame = x300::navFrame(g0);
  if (!sample_limit) {
    fprintf(stderr, "ERROR: duration rounds to zero samples.\n");
    return 1;
  }
  auto save = [&](const std::string &status) {
    if (!writeRunManifest(matched_options.manifest_path, status, g0, samp_freq,
                          clock_source, time_source, sync_state, tx_stats,
                          clipped, navfile, matched_ephemeris_sha256, xyz[0],
                          &live))
      throw std::runtime_error("cannot finalize hardware-time run manifest");
  };
  auto render = [&](x300::Frame &frame) {
    if (generated >= sample_limit)
      return false;
    int count = static_cast<int>(std::min<uint64_t>(
        nextEpochSampleCount(&epoch_plan), sample_limit - generated));
    gpstime_t begin = x300::sampleTime(g0, generated, samp_freq);
    count = static_cast<int>(x300::capAtNavBoundary(begin, samp_freq, count));
    gpstime_t end = x300::sampleTime(g0, generated + count, samp_freq);
    if (live.enabled() && !live.replaying())
      live.checkHealth(getMonotonicSeconds(), end);
    frame.iq.resize(2 * static_cast<size_t>(count));
    generateEpoch(frame.iq.data(), count, chan, gain, active_eph, &ionoutc, end,
                  staticLocationMode, subGpsTime(end, begin), delt,
                  path_loss_enable, fixed_gain, ant_pat, attack_enabled,
                  &attack_cfg, attack_noise_state, jam_js_linear, &clipped);
    if (clipped)
      throw std::runtime_error("generated IQ clipped; reduce per-channel gain");
    generated += count;
    int64_t next_frame = x300::navFrame(end);
    if (next_frame != nav_frame) {
      if (next_frame != nav_frame + 1)
        throw std::runtime_error("navigation refresh skipped a GPS frame");
      if (live.navigation_from_ubx() && !live.replaying()) {
        auto snapshot = live.snapshot();
        x300::checkLiveHealth(snapshot, live.plan, getMonotonicSeconds(), end,
                              live.prns);
        for (int prn : live.prns)
          eph[0][prn - 1] = snapshot.navigation[prn - 1].eph;
      }
      refreshNavState(chan, eph, has_revive_mode ? revive_scan_eph : eph, neph,
                      &ieph, active_eph, &synth_eph, &synth_cfg, &ionoutc, end,
                      elvmask, live.navigation_from_ubx(), &attack_cfg,
                      nullptr);
      nav_frame = next_frame;
    }
    return true;
  };
  installSignalHandlers();
  bool success = false;
  try {
    save("incomplete");
    if (live.check_start) {
      UhdRadio radio(usrp, {}, tx_channel);
      double begin = radio.monotonic();
      x300::Frame frame;
      for (int n = 0; n < prebuffer_count && !stop_requested && render(frame);
           ++n)
        x300::hashIq(frame.iq, tx_stats);
      tx_stats.generated = generated;
      tx_stats.prebuffer_seconds = radio.monotonic() - begin;
      x300::checkHealth(radio, sync_state);
      live.checkPps(radio);
      live.checkHealth(radio.monotonic(), g0);
      save("start_plan_prepared");
      double margin = live.plan.hardware_start - radio.now().get_real_secs();
      if (stop_requested || margin < 0.02)
        throw std::runtime_error("fixed TX deadline missed before send; "
                                 "discard dated IQ and restart");
      tx_stats.minimum_lead_seconds = margin;
      tx_stats.start_margin_met = true;
      success = true;
      fprintf(stderr,
              "[START-CHECK] PASS: planned H=%.9f -> GPS %d:%.9f; %llu samples "
              "prepared in %.6f s, submission margin %.6f s. No TX streamer "
              "created; no RF transmitted.\n",
              live.plan.hardware_start, g0.week, g0.sec,
              (unsigned long long)generated, tx_stats.prebuffer_seconds,
              margin);
    } else if (matched_options.dry_run) {
      x300::Frame frame;
      while (!stop_requested && render(frame)) {
        x300::hashIq(frame.iq, tx_stats);
      }
      tx_stats.generated = generated;
      tx_stats.interrupted = stop_requested != 0;
      success = !stop_requested;
      if (!success)
        tx_stats.failure = "offline render interrupted";
      fprintf(stderr,
              "[DRY-RUN] Rendered %llu samples; UHD was never opened.\n",
              (unsigned long long)generated);
    } else {
      UhdRadio radio(usrp, tx_stream, tx_channel);
      success = x300::transmit(
          radio, sync_state, samp_freq, tx_advance_ns * 1e-9, max_samps,
          prebuffer_count, render, [] { return stop_requested != 0; }, tx_stats,
          [&] {
            save("armed");
            fprintf(stderr,
                    "[TX] Local PPS start %.9f s maps to model epoch %d:%.9f; "
                    "GPS alignment unverified.\n",
                    tx_stats.start_seconds, g0.week, g0.sec);
          },
          live.enabled() ? std::optional<double>(live.plan.hardware_start)
                         : std::nullopt,
          [&] {
            if (live.enabled()) {
              live.checkPps(radio);
              live.checkHealth(
                  getMonotonicSeconds(),
                  x300::sampleTime(g0, tx_stats.accepted, samp_freq));
            }
          });
    }
  } catch (const std::exception &e) {
    tx_stats.failure = e.what();
  }
  if (matched_options.dry_run || live.check_start)
    tx_stats.generated = generated;
  try {
    save(success
             ? (live.check_start
                    ? "start_checked"
                    : (matched_options.dry_run
                           ? "dry_run"
                           : (tx_stats.interrupted ? "stopped" : "completed")))
             : "failed");
  } catch (const std::exception &e) {
    tx_stats.failure = e.what();
    success = false;
  }
  if (!success)
    fprintf(stderr, "[TX] ERROR: %s\n", tx_stats.failure.c_str());
  fprintf(stderr,
          "[TX] status=%s generated=%llu accepted=%llu underflows=%llu "
          "sequence-errors=%llu time-errors=%llu; manifest=%s\n",
          success ? "success" : "failed",
          (unsigned long long)tx_stats.generated,
          (unsigned long long)tx_stats.accepted,
          (unsigned long long)tx_stats.underflows,
          (unsigned long long)tx_stats.sequence_errors,
          (unsigned long long)tx_stats.time_errors,
          matched_options.manifest_path.c_str());
  return success ? 0 : 1;
}

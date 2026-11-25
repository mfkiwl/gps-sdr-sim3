#ifndef X300_PPS_TIME_HPP
#define X300_PPS_TIME_HPP

#include "ubx_receiver.hpp"
#include "x300_timing.hpp"
#include <iomanip>
#include <sstream>

namespace x300 {
struct PpsOptions {
  // The operator must bound an independently synchronized, unsmeared UTC
  // clock. One-way UBX cannot establish its own whole-second delivery bound.
  double host_utc_bound = 0;
  double max_delivery = 0.8; // receipt since the pulse preceding the label
};
struct PpsObservation {
  double before = 0, hardware = 0, pps = 0, utc = 0, after = 0;
  double width() const { return after - before; }
};
inline PpsObservation observePps(Radio &radio) {
  PpsObservation o;
  o.before = radio.monotonic();
  o.hardware = radio.now().get_real_secs();
  o.pps = radio.lastPps().get_real_secs();
  o.utc = radio.utcNow();
  o.after = radio.monotonic();
  return o;
}
inline bool samePulseConfig(const ubx::PulseConfig &a,
                            const ubx::PulseConfig &b) {
  return a.from_valget == b.from_valget && a.flags == b.flags &&
      a.period == b.period && a.locked_period == b.locked_period &&
      a.length == b.length &&
      a.locked_length == b.locked_length &&
      a.antenna_delay_ns == b.antenna_delay_ns &&
      a.rf_delay_ns == b.rf_delay_ns && a.user_delay_ns == b.user_delay_ns;
}
inline void checkPpsMetadata(const ubx::Snapshot &s, double now) {
  const auto &c = s.pulse_config;
  const auto &l = s.leap_info;
  if (!c.present || now < c.received || now - c.received > 10)
    throw std::runtime_error("fresh CFG-TP5 or CFG-VALGET TIMEPULSE0 readback required (bridge --pps)");
  // Require rising, GNSS locked, TOW aligned, 1 Hz TIMEPULSE0. Do not infer
  // polarity or time grid from an observed integer PPS interval alone.
  const uint32_t required = 1 | 2 | 4 | 32 | 64;
  if ((c.flags & required) != required ||
      c.locked_period != ((c.flags & 8) ? 1u : 1000000u) ||
      !c.locked_length || ((c.flags & 16) && c.locked_length >= 1000000) ||
      ((c.flags >> 7) & 15) > 1 || c.user_delay_ns != 0)
    throw std::runtime_error("PPS requires 1 Hz rising GNSS/TOW-aligned UTC or GPS grid with zero user delay");
  if (!s.time.valid || !s.time.leap_valid || now < s.time.received ||
      now - s.time.received > 1.5 || !l.present || (l.valid & 3) != 3 ||
      now < l.received || now - l.received > 10 ||
      l.current != s.time.leap_seconds || std::abs(l.current) > 64)
    throw std::runtime_error("fresh consistent NAV-TIMEGPS/NAV-TIMELS leap information required");
  // POSIX UTC cannot unambiguously label an inserted/deleted second. Stop
  // before the event; reacquire with current leap metadata afterwards.
  if (std::fabs(l.seconds_to_event - (now - l.received)) < 10)
    throw std::runtime_error("leap-second transition: stop and reacquire PPS epoch afterwards");
}
inline gpstime_t pulseGps(const ubx::TimePulse &p, const ubx::Snapshot &s) {
  const bool utc = p.flags & 1;
  const unsigned grid = (s.pulse_config.flags >> 7) & 15;
  if (!p.present || p.week > 32767 || p.tow_ms >= 604800000 ||
      p.tow_ms % 1000 || p.tow_sub_ms || (p.flags & ~31u) ||
      ((p.flags >> 2) & 3) == 3 ||
      (utc ? (grid != 0 || !(p.flags & 2) ||
              (p.reference_info >> 4) < 1 || (p.reference_info >> 4) > 8)
           : (grid != 1 || (p.reference_info & 15) != 0)))
    throw std::runtime_error("invalid TIM-TP second label, time base or reference");
  gpstime_t gps{static_cast<int>(p.week), p.tow_ms * .001};
  if (utc) gps.sec += s.time.leap_seconds;
  // qErr is intentionally not applied (including when invalid). This maps
  // the nominal labeled boundary; calibrated edge/RF accuracy is separate.
  return sampleTime(gps, 0, 1);
}
struct PpsReference {
  bool verified = false;
  double hardware = 0;
  gpstime_t gps{-1, 0};
  double label_received = 0;
  double last_confirmed_hardware = 0;
  unsigned confirmed_edges = 0;
  PpsOptions options;
  ubx::TimePulse pulse;
  ubx::PulseConfig config;
  int leap_seconds = 0;
  double maximum_host_residual = 0, maximum_query_width = 0;
};
// Sender-thread state only. The immutable reference is copied into the start
// plan; neither subsequent labels nor a host clock adjustment can retime IQ.
class PpsAssociation {
public:
  explicit PpsAssociation(PpsOptions options = {}, double begin = 0)
      : begin_(begin) {
    reference_.options = options;
  }
  const PpsReference &reference() const { return reference_; }
  void observe(const ubx::Snapshot &s, const PpsObservation &o) {
    const auto &options = reference_.options;
    if (!(options.host_utc_bound > 0 && options.host_utc_bound <= .25) ||
        !std::isfinite(options.max_delivery) || options.max_delivery <= .02 ||
        options.max_delivery > .9)
      throw std::runtime_error("PPS requires --pps-host-utc-bound-sec in (0,0.25] and delivery bound in (0.02,0.9]");
    if (!std::isfinite(o.before) || !std::isfinite(o.after) ||
        !std::isfinite(o.hardware) || !std::isfinite(o.pps) ||
        !std::isfinite(o.utc) || o.width() < 0 || o.width() > .05 ||
        o.pps > o.hardware + o.width() || o.hardware - o.pps > 1.5 ||
        std::fabs(o.pps - std::round(o.pps)) > 1e-6)
      throw std::runtime_error("ambiguous PPS observation or PPS/sample-clock drift");
    if (have_observation_ &&
        (o.pps < last_observation_.pps ||
         o.after < last_observation_.after ||
         std::fabs((o.hardware - last_observation_.hardware) -
                   (o.after - last_observation_.after)) > .1 ||
         std::fabs((o.utc - last_observation_.utc) -
                   (o.after - last_observation_.after)) > .05))
      throw std::runtime_error("hardware or host UTC clock stepped; reacquire PPS epoch");
    auto problem = ubx::readiness(s, o.after, s.time.gps, {}, false);
    if (s.replay || !problem.empty())
      throw std::runtime_error("PPS needs fresh live UBX: " + problem);
    checkPpsMetadata(s, o.after);
    if (!have_observation_) {
      reference_.config = s.pulse_config;
      reference_.leap_seconds = s.time.leap_seconds;
      rejections_ = s.time_rejections;
      resets_ = s.clock_resets;
    } else if (!samePulseConfig(reference_.config, s.pulse_config) ||
               reference_.leap_seconds != s.time.leap_seconds ||
               rejections_ != s.time_rejections || resets_ != s.clock_resets)
      throw std::runtime_error("receiver reset, time rejection, leap or PPS configuration changed");
    last_observation_ = o;
    have_observation_ = true;
    reference_.maximum_query_width = std::max(reference_.maximum_query_width, o.width());
    for (const auto &pulse : s.time_pulses) {
      if (pulse.sequence <= sequence_) continue;
      if (pulse.received < begin_ && labels_.empty()) {
        sequence_ = pulse.sequence; continue;
      }
      if (sequence_ && pulse.sequence != sequence_ + 1)
        throw std::runtime_error("TIM-TP history overrun or skipped report");
      if (pulse.received > o.after || o.after - pulse.received > 2.5)
        throw std::runtime_error("stale TIM-TP label");
      const auto gps = pulseGps(pulse, s);
      const double utc = 315964800.0 + gps.week * 604800.0 + gps.sec -
                         s.time.leap_seconds;
      const double predicted = o.hardware + utc - o.utc;
      const double target = std::round(predicted);
      const double residual = std::fabs(predicted - target);
      if (residual > options.host_utc_bound + o.width())
        throw std::runtime_error("TIM-TP epoch disagrees with independent host UTC bound");
      // TIM-TP labels the NEXT pulse. Bracket its first receipt in the
      // preceding physical second, including the full hardware query RTT.
      const double received_low = o.hardware + pulse.received - o.after;
      const double received_high = o.hardware + pulse.received - o.before;
      if (received_low <= target - 1 || received_high >= target - .02 ||
          received_high - (target - 1) > options.max_delivery)
        throw std::runtime_error("TIM-TP next pulse ambiguous/late: delivery bound or edge margin violated");
      if (!labels_.empty() &&
          (std::fabs(subGpsTime(gps, labels_.back().gps) - 1) > 1e-9 ||
           target != labels_.back().hardware + 1 ||
           pulse.received <= labels_.back().pulse.received))
        throw std::runtime_error("TIM-TP duplicate, skipped, reordered or inconsistent label");
      if (reference_.confirmed_edges &&
          std::fabs(subGpsTime(gps, reference_.gps) -
                    (target - reference_.hardware)) > 1e-6)
        throw std::runtime_error("TIM-TP departed from frozen PPS/GPS mapping");
      labels_.push_back({target, gps, pulse});
      if (labels_.size() > 16) labels_.pop_front();
      sequence_ = pulse.sequence;
      reference_.maximum_host_residual = std::max(reference_.maximum_host_residual, residual);
    }
    if (labels_.empty()) {
      if (o.after - begin_ > 2.5)
        throw std::runtime_error("no fresh TIM-TP pulse stream (bridge --pps required)");
      return;
    }
    if (o.after - labels_.back().pulse.received > 1.5)
      throw std::runtime_error("TIM-TP pulse-label stream stopped");
    for (const auto &label : labels_) {
      if (std::fabs(label.hardware - o.pps) > 1e-6 ||
          (reference_.confirmed_edges && o.pps <= reference_.last_confirmed_hardware))
        continue;
      if (!reference_.confirmed_edges) {
        reference_.hardware = o.pps;
        reference_.gps = label.gps;
        reference_.pulse = label.pulse;
        reference_.label_received = label.pulse.received;
      } else if (!reference_.verified &&
                 std::fabs(o.pps - reference_.last_confirmed_hardware - 1) > 1e-6)
        throw std::runtime_error("skipped hardware PPS during epoch acquisition");
      if (std::fabs(subGpsTime(label.gps, reference_.gps) -
                    (o.pps - reference_.hardware)) > 1e-6)
        throw std::runtime_error("PPS drift from frozen GPS epoch exceeds 1 us");
      reference_.last_confirmed_hardware = o.pps;
      ++reference_.confirmed_edges;
      reference_.verified = reference_.confirmed_edges >= 3;
    }
    if (reference_.verified && o.hardware - reference_.last_confirmed_hardware > 1.5)
      throw std::runtime_error("GPS-labeled hardware PPS missing");
  }

private:
  struct Label { double hardware; gpstime_t gps; ubx::TimePulse pulse; };
  std::deque<Label> labels_;
  PpsReference reference_;
  double begin_ = 0;
  uint64_t sequence_ = 0, rejections_ = 0, resets_ = 0;
  bool have_observation_ = false;
  PpsObservation last_observation_;
};

inline std::string ppsReferenceJson(const PpsReference &r) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"verified\":" << (r.verified ? "true" : "false")
      << ",\"hardware_ref_s\":" << r.hardware
      << ",\"gps_ref_week\":" << r.gps.week << ",\"gps_ref_tow\":" << r.gps.sec
      << ",\"confirmed_edges\":" << r.confirmed_edges
      << ",\"last_confirmed_hardware_s\":" << r.last_confirmed_hardware
      << ",\"label_first_received_monotonic_s\":" << r.label_received
      << ",\"label_semantics\":\"next_pulse\",\"time_base\":\""
      << ((r.pulse.flags & 1) ? "UTC" : "GPS") << '"'
      << ",\"gps_minus_utc_s\":" << r.leap_seconds
      << ",\"host_utc_bound_assumed_s\":" << r.options.host_utc_bound
      << ",\"max_label_delivery_s\":" << r.options.max_delivery
      << ",\"max_host_epoch_residual_s\":" << r.maximum_host_residual
      << ",\"max_query_width_s\":" << r.maximum_query_width
      << ",\"pulse_config_source\":\""
      << (r.config.from_valget ? "CFG-VALGET" : "CFG-TP5") << '"'
      << ",\"cfg_tp5_flags\":" << r.config.flags
      << ",\"antenna_cable_delay_setting_ns\":" << r.config.antenna_delay_ns
      << ",\"receiver_rf_delay_setting_ns\":";
  // CFG-VALGET has no RF group delay item; do not report an unread zero.
  if (r.config.from_valget)
    out << "null";
  else
    out << r.config.rf_delay_ns;
  out << ",\"qerr_valid\":" << ((r.pulse.flags & 16) ? "false" : "true")
      << ",\"qerr_reported_ps\":" << r.pulse.quantization_error_ps
      << ",\"qerr_applied\":false,\"edge_path_calibrated\":false,\"rf_alignment_verified\":false}";
  return out.str();
}
} // namespace x300
#endif

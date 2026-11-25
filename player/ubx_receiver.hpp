#ifndef UBX_RECEIVER_HPP
#define UBX_RECEIVER_HPP

#include "gps_lnav.hpp"
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ubx {
double monotonicSeconds();

struct TimeSolution {
  gpstime_t gps{-1, 0};
  uint32_t itow_ms = 0;
  int32_t ftow_ns = 0;
  uint32_t accuracy_ns = 0;
  int leap_seconds = 0;
  bool leap_valid = false;
  bool valid = false;
  double received = 0;
};
struct Position {
  bool valid = false;
  uint32_t itow_ms = 0;
  double latitude = 0, longitude = 0, height = 0;
  double horizontal_accuracy = 0, vertical_accuracy = 0, speed = 0;
  int satellites = 0;
  double received = 0;
};
struct Observation {
  unsigned gnss = 0, prn = 0, signal = 0, flags = 0, lock_ms = 0, cno = 0;
  double pseudorange = 0, carrier_cycles = 0, doppler = 0;
};
struct RfBlock {
  unsigned id = 0, jamming_state = 0, antenna_status = 0, antenna_power = 0;
  unsigned noise_per_ms = 0, agc_count = 0, cw_suppression = 0;
};
// Labels retain their reported time base. A USB report does not associate a
// pulse or EXTINT event with an X300 edge; none of these fields arms the radio.
struct TimePulse {
  bool present = false;
  uint32_t tow_ms = 0, tow_sub_ms = 0;
  int32_t quantization_error_ps = 0;
  unsigned week = 0, flags = 0, reference_info = 0;
  double received = 0;
  uint64_t sequence = 0;
};
// TIMEPULSE0 settings in CFG-TP5 layout. Receivers that NAK CFG-TP5 (ZED-F9K)
// are read back through CFG-VALGET CFG-TP-*; that path has no RF group delay.
struct PulseConfig {
  bool present = false, from_valget = false;
  uint32_t period = 0, locked_period = 0, length = 0, locked_length = 0;
  int antenna_delay_ns = 0, rf_delay_ns = 0, user_delay_ns = 0;
  uint32_t flags = 0;
  double received = 0;
};
struct LeapInfo {
  bool present = false;
  int current = 0, change = 0, seconds_to_event = 0;
  unsigned valid = 0;
  double received = 0;
};
struct TimeMark {
  bool present = false;
  unsigned channel = 0, flags = 0, count = 0, rising_week = 0, falling_week = 0;
  uint32_t rising_ms = 0, rising_sub_ns = 0, falling_ms = 0, falling_sub_ns = 0;
  uint32_t accuracy_ns = 0;
  double received = 0;
};
struct Snapshot {
  bool connected = false, replay = false;
  std::string identity, unique_id, failure;
  TimeSolution time;
  std::deque<TimeSolution> times;
  Position position;
  std::array<Navigation, MAX_SAT> navigation{};
  std::array<bool, MAX_SAT> gps_l1_observed{};
  gpstime_t raw_epoch{-1, 0};
  double raw_received = 0;
  std::vector<Observation> observations;
  uint32_t clock_itow_ms = 0;
  int32_t clock_bias_ns = 0, clock_drift_ns_s = 0;
  double clock_received = 0;
  uint64_t bytes = 0, frames = 0, checksum_errors = 0, malformed = 0;
  uint64_t discarded_bytes = 0, time_rejections = 0, clock_resets = 0;
  uint64_t pvt_epochs = 0, valid_3d_epochs = 0;
  uint64_t rawx_epochs = 0, nonempty_rawx_epochs = 0;
  bool rf_present = false;
  double rf_received = 0;
  std::vector<RfBlock> rf_blocks;
  TimePulse time_pulse;
  std::deque<TimePulse> time_pulses;
  uint64_t pulse_duplicates = 0;
  bool tp5_unsupported = false;
  PulseConfig pulse_config;
  LeapInfo leap_info;
  TimeMark time_mark;
  double last_received = 0;
};

// Receive-only diagnostic at a host query instant. This estimate has no
// physical PPS association and must not be treated as a radio start time.
struct GpsNowEstimate {
  gpstime_t gps{-1, 0};
  double host_monotonic = 0;
  double solution_age = 0;
  double delivery_delay = 0;
  // Unit-slope fit over the last five seconds: choose the largest GPS-minus-
  // arrival offset (least observed delivery delay). Constant bias is unknown.
  double arrival_correction = 0, arrival_variation = 0, window_seconds = 0;
  size_t observations = 0;
};
GpsNowEstimate estimateGpsNow(const Snapshot &s, double now,
                              double delivery_delay = 0);
std::string timeEstimateJson(const GpsNowEstimate &estimate);

// Parser is independent of transport and deterministic under recorded arrivals.
class Decoder {
public:
  Snapshot state;
  void feed(const uint8_t *data, size_t size, double received);
  size_t buffered() const { return buffer_.size(); }

private:
  std::vector<uint8_t> buffer_;
  void message(uint8_t cls, uint8_t id, const uint8_t *p, size_t n, double at);
};

class Receiver {
public:
  Receiver(const std::string &endpoint, const std::string &record_path);
  ~Receiver();
  Receiver(const Receiver &) = delete;
  Receiver &operator=(const Receiver &) = delete;
  Snapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  Decoder decoder_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  void run(std::string endpoint, std::string record_path);
};

Snapshot replay(const std::string &path);
// Returns a reason when mandatory time/position/selected ephemerides are
// unusable.
std::string readiness(const Snapshot &s, double now, gpstime_t epoch,
                      const std::vector<int> &prns, bool require_navigation);
std::vector<int> observedPrns(const Snapshot &s);
// ZED-F9P or ZED-F9K MON-VER module string; other receivers are not qualified.
bool supportedIdentity(const std::string &identity);
std::string snapshotJson(const Snapshot &s, double now);

} // namespace ubx
#endif

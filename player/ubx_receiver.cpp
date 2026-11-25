#include "ubx_receiver.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <netdb.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace ubx {
namespace {
uint16_t u16(const uint8_t *p) { return p[0] | uint16_t(p[1]) << 8; }
uint32_t u32(const uint8_t *p) { return u16(p) | uint32_t(u16(p + 2)) << 16; }
int16_t i16(const uint8_t *p) { return static_cast<int16_t>(u16(p)); }
int32_t i32(const uint8_t *p) { return static_cast<int32_t>(u32(p)); }
template <typename T> T real(const uint8_t *p) {
  uint64_t value = 0;
  for (size_t i = 0; i < sizeof(T); ++i)
    value |= uint64_t(p[i]) << (8 * i);
  T result;
  if constexpr (sizeof(T) == 4) {
    uint32_t small = value;
    std::memcpy(&result, &small, sizeof(T));
  } else
    std::memcpy(&result, &value, sizeof(T));
  return result;
}
std::string ascii(const uint8_t *p, size_t n) {
  std::string result;
  for (size_t i = 0; i < n && p[i]; ++i)
    result += p[i] >= 32 && p[i] < 127 && p[i] != '"' && p[i] != '\\'
                  ? static_cast<char>(p[i])
                  : '?';
  return result;
}
struct Fd {
  int value = -1;
  ~Fd() {
    if (value >= 0)
      ::close(value);
  }
};
void writeAll(int fd, const void *data, size_t n) {
  auto p = static_cast<const uint8_t *>(data);
  while (n) {
    ssize_t done = ::write(fd, p, n);
    if (done < 0 && errno == EINTR)
      continue;
    if (done <= 0)
      throw std::runtime_error("UBX recording write failed");
    p += done;
    n -= done;
  }
}
int connectTcp(const std::string &endpoint) {
  auto colon = endpoint.rfind(':');
  if (colon == std::string::npos || !colon || colon + 1 == endpoint.size())
    throw std::runtime_error("--ublox-tcp expects host:port");
  std::string host = endpoint.substr(0, colon),
              port = endpoint.substr(colon + 1);
  if (host.front() == '[' && host.back() == ']')
    host = host.substr(1, host.size() - 2);
  if (port.find_first_not_of("0123456789") != std::string::npos ||
      port.size() > 5 || std::stoul(port) < 1 || std::stoul(port) > 65535)
    throw std::runtime_error("invalid UBX TCP port");
  addrinfo hints{}, *addresses = nullptr;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_family = AF_UNSPEC;
  int error = getaddrinfo(host.c_str(), port.c_str(), &hints, &addresses);
  if (error)
    throw std::runtime_error(std::string("UBX DNS: ") + gai_strerror(error));
  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> owner(addresses,
                                                           freeaddrinfo);
  for (auto a = addresses; a; a = a->ai_next) {
    Fd fd{socket(a->ai_family, a->ai_socktype, a->ai_protocol)};
    if (fd.value < 0)
      continue;
    fcntl(fd.value, F_SETFL, O_NONBLOCK);
    int status = connect(fd.value, a->ai_addr, a->ai_addrlen);
    if (status < 0 && errno != EINPROGRESS)
      continue;
    pollfd p{fd.value, POLLOUT, 0};
    if (status < 0 && poll(&p, 1, 3000) <= 0)
      continue;
    int so_error = 0;
    socklen_t length = sizeof(so_error);
    if (getsockopt(fd.value, SOL_SOCKET, SO_ERROR, &so_error, &length) ||
        so_error)
      continue;
    int result = fd.value;
    fd.value = -1;
    return result;
  }
  throw std::runtime_error("cannot connect to UBX TCP endpoint");
}
// CFG-TP-* TIMEPULSE0 keys (u-blox F9 interface description), as polled by
// tools/ubx_tcp_bridge.py --pps.
enum : uint32_t {
  TP_CABLE_DELAY = 0x30050001, TP_PERIOD = 0x40050002,
  TP_PERIOD_LOCK = 0x40050003, TP_LEN = 0x40050004, TP_LEN_LOCK = 0x40050005,
  TP_USER_DELAY = 0x40050006, TP_ENA = 0x10050007, TP_SYNC_GNSS = 0x10050008,
  TP_USE_LOCKED = 0x10050009, TP_ALIGN_TOW = 0x1005000a,
  TP_POLARITY = 0x1005000b, TP_TIMEGRID = 0x2005000c,
  TP_PULSE_DEF = 0x20050023, TP_FREQ = 0x40050024, TP_FREQ_LOCK = 0x40050025,
  TP_DUTY = 0x5005002a, TP_DUTY_LOCK = 0x5005002b, TP_LENGTH_DEF = 0x20050030
};
using ValgetItems = std::map<uint32_t, const uint8_t *>;
// Returns false for a partial readback. Throws on values CFG-TP5 cannot hold.
bool valgetPulseConfig(const ValgetItems &v, double at, PulseConfig &c) {
  for (uint32_t key :
       {TP_CABLE_DELAY, TP_PERIOD, TP_PERIOD_LOCK, TP_LEN, TP_LEN_LOCK,
        TP_USER_DELAY, TP_ENA, TP_SYNC_GNSS, TP_USE_LOCKED, TP_ALIGN_TOW,
        TP_POLARITY, TP_TIMEGRID, TP_PULSE_DEF, TP_FREQ, TP_FREQ_LOCK, TP_DUTY,
        TP_DUTY_LOCK, TP_LENGTH_DEF})
    if (!v.count(key))
      return false;
  auto bit = [&](uint32_t key) { return uint32_t(v.at(key)[0] & 1); };
  const unsigned pulse_def = v.at(TP_PULSE_DEF)[0],
                 length_def = v.at(TP_LENGTH_DEF)[0],
                 grid = v.at(TP_TIMEGRID)[0];
  if (pulse_def > 1 || length_def > 1 || grid > 15)
    throw std::invalid_argument("CFG-TP enumeration");
  // CFG-TP5 ratio lengths are duty cycles in units of 2^-32.
  auto length = [&](uint32_t microseconds, uint32_t percent) -> uint32_t {
    if (length_def)
      return u32(v.at(microseconds));
    double duty = real<double>(v.at(percent));
    if (!std::isfinite(duty) || duty < 0 || duty > 100)
      throw std::invalid_argument("CFG-TP duty");
    return static_cast<uint32_t>(
        std::min(duty / 100 * 4294967296.0, 4294967295.0));
  };
  c = {};
  c.present = c.from_valget = true;
  c.flags = bit(TP_ENA) | bit(TP_SYNC_GNSS) << 1 | bit(TP_USE_LOCKED) << 2 |
            pulse_def << 3 | length_def << 4 | bit(TP_ALIGN_TOW) << 5 |
            bit(TP_POLARITY) << 6 | grid << 7;
  c.period = u32(v.at(pulse_def ? TP_FREQ : TP_PERIOD));
  c.locked_period = u32(v.at(pulse_def ? TP_FREQ_LOCK : TP_PERIOD_LOCK));
  c.length = length(TP_LEN, TP_DUTY);
  c.locked_length = length(TP_LEN_LOCK, TP_DUTY_LOCK);
  c.antenna_delay_ns = i16(v.at(TP_CABLE_DELAY));
  c.user_delay_ns = i32(v.at(TP_USER_DELAY));
  c.received = at;
  return true;
}
} // namespace

double monotonicSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void Decoder::feed(const uint8_t *data, size_t size, double at) {
  if (!std::isfinite(at))
    throw std::invalid_argument("invalid receive timestamp");
  state.bytes += size;
  state.last_received = at;
  // Process incrementally so even a huge/coalesced input cannot grow the
  // parser.
  for (size_t i = 0; i < size; ++i) {
    buffer_.push_back(data[i]);
    for (;;) {
      if (buffer_.size() < 2)
        break;
      if (buffer_[0] != 0xb5 || buffer_[1] != 0x62) {
        buffer_.erase(buffer_.begin());
        ++state.discarded_bytes;
        continue;
      }
      if (buffer_.size() < 6)
        break;
      size_t n = u16(buffer_.data() + 4);
      if (n > 8192) {
        buffer_.erase(buffer_.begin());
        ++state.malformed;
        continue;
      }
      if (buffer_.size() < n + 8)
        break;
      uint8_t a = 0, b = 0;
      for (size_t j = 2; j < n + 6; ++j) {
        a += buffer_[j];
        b += a;
      }
      if (a != buffer_[n + 6] || b != buffer_[n + 7]) {
        ++state.checksum_errors;
        buffer_.erase(buffer_.begin());
        continue;
      }
      ++state.frames;
      message(buffer_[2], buffer_[3], buffer_.data() + 6, n, at);
      buffer_.erase(buffer_.begin(), buffer_.begin() + n + 8);
    }
  }
}

void Decoder::message(uint8_t cls, uint8_t id, const uint8_t *p, size_t n,
                      double at) {
  if (cls == 0x0a && id == 4) {
    if (n < 40 || (n - 40) % 30) {
      ++state.malformed;
      return;
    }
    state.identity = ascii(p, 30) + "; HW=" + ascii(p + 30, 10);
    for (size_t j = 40; j < n; j += 30)
      state.identity += "; " + ascii(p + j, 30);
  } else if (cls == 0x27 && id == 3) {
    if (n != 9 || p[0] != 1) {
      ++state.malformed;
      return;
    }
    std::ostringstream hex;
    hex << std::hex << std::setfill('0');
    for (size_t j = 4; j < 9; ++j)
      hex << std::setw(2) << unsigned(p[j]);
    state.unique_id = hex.str();
  } else if (cls == 10 && id == 0x38) {
    if (n < 4 || p[0] != 0 || n != size_t(4 + 24 * p[1])) {
      ++state.malformed;
      return;
    }
    std::vector<RfBlock> blocks;
    for (size_t j = 4; j < n; j += 24)
      blocks.push_back({p[j], unsigned(p[j + 1] & 3), p[j + 2], p[j + 3],
                        u16(p + j + 12), u16(p + j + 14), p[j + 16]});
    state.rf_blocks = std::move(blocks);
    state.rf_received = at;
    state.rf_present = true;
  } else if (cls == 0x0d && id == 1) {
    if (n != 16 || u32(p) >= 604800000) {
      ++state.malformed;
      return;
    }
    TimePulse next{true, u32(p), u32(p + 4), i32(p + 8),
                   u16(p + 12), p[14], p[15], at};
    const auto &last = state.time_pulse;
    // Poll replies may repeat the same next pulse. Keep its FIRST arrival:
    // duplicates must never refresh the freshness watchdog or hide a stall.
    if (last.present && next.week == last.week && next.tow_ms == last.tow_ms &&
        next.tow_sub_ms == last.tow_sub_ms && next.flags == last.flags &&
        next.reference_info == last.reference_info &&
        next.quantization_error_ps == last.quantization_error_ps) {
      ++state.pulse_duplicates;
      return;
    }
    next.sequence = last.sequence + 1;
    state.time_pulse = next;
    state.time_pulses.push_back(next);
    if (state.time_pulses.size() > 32)
      state.time_pulses.pop_front();
  } else if (cls == 6 && id == 0x31) {
    if (n != 32 || p[1] != 1) { ++state.malformed; return; }
    if (p[0] != 0) return; // TIM-TP describes TIMEPULSE0 only.
    state.pulse_config = {true, false, u32(p + 8), u32(p + 12), u32(p + 16),
        u32(p + 20), i16(p + 4), i16(p + 6), i32(p + 24), u32(p + 28), at};
  } else if (cls == 5 && id == 0) {
    if (n != 2) { ++state.malformed; return; }
    if (p[0] == 6 && p[1] == 0x31) state.tp5_unsupported = true;
  } else if (cls == 6 && id == 0x8b) {
    if (n < 4 || p[0] != 1) { ++state.malformed; return; }
    ValgetItems items;
    for (size_t j = 4; j < n;) {
      static const size_t sizes[8] = {0, 1, 1, 2, 4, 8, 0, 0};
      const size_t size = j + 4 <= n ? sizes[(p[j + 3] >> 4) & 7] : 0;
      if (!size || j + 4 + size > n) { ++state.malformed; return; }
      items[u32(p + j)] = p + j + 4;
      j += 4 + size;
    }
    // CFG-TP5 stays authoritative (it also reports RF group delay). Only the
    // RAM layer replaces it, and only after the receiver NAKs CFG-TP5.
    if (p[1] != 0 || !state.tp5_unsupported) return;
    try {
      PulseConfig config;
      if (valgetPulseConfig(items, at, config)) state.pulse_config = config;
    } catch (const std::invalid_argument &) {
      ++state.malformed;
    }
  } else if (cls == 1 && id == 0x26) {
    if (n != 24 || p[4] != 0) { ++state.malformed; return; }
    state.leap_info = {true, static_cast<int8_t>(p[9]),
        static_cast<int8_t>(p[11]), i32(p + 12), p[23], at};
  } else if (cls == 0x0d && id == 3) {
    if (n != 28 || u32(p + 8) >= 604800000 || u32(p + 16) >= 604800000 ||
        u32(p + 12) >= 1000000 || u32(p + 20) >= 1000000) {
      ++state.malformed;
      return;
    }
    state.time_mark = {true,        p[0],        p[1],        u16(p + 2),
                       u16(p + 4),  u16(p + 6),  u32(p + 8),  u32(p + 12),
                       u32(p + 16), u32(p + 20), u32(p + 24), at};
  } else if (cls == 1 && id == 0x20) {
    if (n != 16) {
      ++state.malformed;
      return;
    }
    TimeSolution next;
    next.itow_ms = u32(p);
    next.ftow_ns = i32(p + 4);
    next.gps = {i16(p + 8), next.itow_ms * 1e-3 + next.ftow_ns * 1e-9};
    next.accuracy_ns = u32(p + 12);
    next.received = at;
    next.leap_seconds = static_cast<int8_t>(p[10]);
    next.leap_valid = p[11] & 4;
    next.valid = (p[11] & 3) == 3 && next.gps.week >= 0 &&
                 next.itow_ms < 604800000 &&
                 std::abs(int64_t(next.ftow_ns)) <= 500000;
    if (next.gps.sec < 0) {
      next.gps.sec += 604800;
      --next.gps.week;
    }
    if (next.gps.week < 0)
      next.valid = false;
    if (!state.times.empty() && next.valid) {
      const auto &last = state.times.back();
      double dt = subGpsTime(next.gps, last.gps), arrival = at - last.received;
      if (dt <= 0 || dt > 2.5 || arrival <= 0 || std::fabs(dt - arrival) > 0.25)
        next.valid = false;
    }
    if (!next.valid) {
      ++state.time_rejections;
      state.times.clear();
    } else {
      state.times.push_back(next);
      if (state.times.size() > 16)
        state.times.pop_front();
    }
    state.time = next;
  } else if (cls == 1 && id == 7) {
    if (n != 92) {
      ++state.malformed;
      return;
    }
    Position next;
    next.itow_ms = u32(p);
    next.received = at;
    next.satellites = p[23];
    next.longitude = i32(p + 24) * 1e-7;
    next.latitude = i32(p + 28) * 1e-7;
    next.height = i32(p + 32) * 1e-3;
    next.horizontal_accuracy = u32(p + 40) * 1e-3;
    next.vertical_accuracy = u32(p + 44) * 1e-3;
    next.speed = i32(p + 60) * 1e-3;
    next.valid = (p[21] & 1) && (p[20] == 3 || p[20] == 4) && !(p[78] & 1) &&
                 std::fabs(next.latitude) <= 90 &&
                 std::fabs(next.longitude) <= 180 && next.itow_ms < 604800000;
    state.position = next;
    ++state.pvt_epochs;
    state.valid_3d_epochs += next.valid;
  } else if (cls == 1 && id == 0x22) {
    if (n != 20) {
      ++state.malformed;
      return;
    }
    state.clock_itow_ms = u32(p);
    state.clock_bias_ns = i32(p + 4);
    state.clock_drift_ns_s = i32(p + 8);
    state.clock_received = at;
  } else if (cls == 2 && id == 0x15) {
    if (n < 16 || p[13] != 1 || n != size_t(16 + 32 * p[11])) {
      ++state.malformed;
      return;
    }
    double tow = real<double>(p);
    if (!std::isfinite(tow) || tow < 0 || tow >= 604800) {
      ++state.malformed;
      return;
    }
    std::vector<Observation> observations;
    for (size_t j = 16; j < n; j += 32) {
      Observation o;
      o.pseudorange = real<double>(p + j);
      o.carrier_cycles = real<double>(p + j + 8);
      o.doppler = real<float>(p + j + 16);
      o.gnss = p[j + 20];
      o.prn = p[j + 21];
      o.signal = p[j + 22];
      o.lock_ms = u16(p + j + 24);
      o.cno = p[j + 26];
      o.flags = p[j + 30];
      if (!std::isfinite(o.pseudorange) || !std::isfinite(o.carrier_cycles) ||
          !std::isfinite(o.doppler)) {
        ++state.malformed;
        return;
      }
      observations.push_back(o);
    }
    if (p[12] & 2) {
      ++state.clock_resets;
      state.times.clear();
      state.time.valid = false;
    }
    state.raw_epoch = {u16(p + 8), tow};
    state.raw_received = at;
    state.gps_l1_observed.fill(false);
    for (const auto &o : observations)
      if (o.gnss == 0 && o.signal == 0 && o.prn >= 1 && o.prn <= MAX_SAT &&
          (o.flags & 1))
        state.gps_l1_observed[o.prn - 1] = true;
    state.observations = std::move(observations);
    ++state.rawx_epochs;
    state.nonempty_rawx_epochs += !state.observations.empty();
  } else if (cls == 2 && id == 0x13) {
    if (n < 8 || p[6] != 2 || n != size_t(8 + 4 * p[4]) || p[4] > 10) {
      ++state.malformed;
      return;
    }
    if (p[0] != 0 || p[2] != 0)
      return; // Only GPS L1 C/A, never L2C/CNAV.
    if (p[1] < 1 || p[1] > MAX_SAT || p[4] != 10) {
      ++state.malformed;
      return;
    }
    if (!state.time.valid)
      return;
    std::array<uint32_t, 10> words;
    for (size_t j = 0; j < 10; ++j)
      words[j] = u32(p + 8 + j * 4);
    if (!ingestLnav(state.navigation[p[1] - 1], words, state.time.gps, at))
      ++state.malformed;
  }
}

Receiver::Receiver(const std::string &endpoint, const std::string &path) {
  thread_ = std::thread(&Receiver::run, this, endpoint, path);
}
Receiver::~Receiver() {
  stop_ = true;
  if (thread_.joinable())
    thread_.join();
}
Snapshot Receiver::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return decoder_.state;
}
void Receiver::run(std::string endpoint, std::string path) {
  try {
    Fd raw, arrivals;
    if (!path.empty()) {
      raw.value = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
      arrivals.value =
          open((path + ".rx.csv").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (raw.value < 0 || arrivals.value < 0)
        throw std::runtime_error("UBX record paths must be new writable files");
      std::string header = "offset,length,host_monotonic_s\n";
      writeAll(arrivals.value, header.data(), header.size());
    }
    Fd socket_fd{connectTcp(endpoint)};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      decoder_.state.connected = true;
    }
    uint64_t offset = 0;
    while (!stop_) {
      pollfd poller{socket_fd.value, POLLIN, 0};
      int status = poll(&poller, 1, 100);
      if (status < 0 && errno == EINTR)
        continue;
      if (status < 0)
        throw std::runtime_error("UBX socket poll failed");
      if (!status)
        continue;
      uint8_t data[16384];
      ssize_t n = recv(socket_fd.value, data, sizeof(data), 0);
      double received = monotonicSeconds();
      if (n < 0 && (errno == EAGAIN || errno == EINTR))
        continue;
      if (n <= 0)
        throw std::runtime_error(
            "UBX TCP disconnected; new segment and warmup required");
      if (raw.value >= 0) {
        writeAll(raw.value, data, n);
        std::ostringstream row;
        row << std::setprecision(17) << offset << ',' << n << ',' << received
            << '\n';
        auto text = row.str();
        writeAll(arrivals.value, text.data(), text.size());
        offset += n;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      decoder_.feed(data, n, received);
    }
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(mutex_);
    decoder_.state.failure = e.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  decoder_.state.connected = false;
}

Snapshot replay(const std::string &path) {
  std::ifstream raw(path, std::ios::binary), arrivals(path + ".rx.csv");
  if (!raw || !arrivals)
    throw std::runtime_error("replay requires UBX and .rx.csv arrivals");
  Decoder decoder;
  std::string line;
  std::getline(arrivals, line);
  if (line != "offset,length,host_monotonic_s")
    throw std::runtime_error("invalid UBX arrivals header");
  uint64_t expected = 0;
  double previous = -1;
  while (std::getline(arrivals, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    uint64_t offset, n;
    double at;
    if (!(row >> offset >> n >> at) || offset != expected || !n || n > 16384 ||
        !std::isfinite(at) || at < previous)
      throw std::runtime_error("invalid UBX arrival record");
    std::vector<uint8_t> data(n);
    if (!raw.read(reinterpret_cast<char *>(data.data()), n))
      throw std::runtime_error("truncated UBX recording");
    decoder.feed(data.data(), n, at);
    expected += n;
    previous = at;
  }
  if (raw.peek() != std::char_traits<char>::eof())
    throw std::runtime_error("UBX bytes missing arrival records");
  decoder.state.replay = true;
  return decoder.state;
}

std::vector<int> observedPrns(const Snapshot &s) {
  std::vector<int> result;
  for (int prn = 1; prn <= MAX_SAT; ++prn)
    if (s.gps_l1_observed[prn - 1])
      result.push_back(prn);
  return result;
}
bool supportedIdentity(const std::string &identity) {
  return identity.find("MOD=ZED-F9P") != std::string::npos ||
         identity.find("MOD=ZED-F9K") != std::string::npos;
}
std::string readiness(const Snapshot &s, double now, gpstime_t epoch,
                      const std::vector<int> &prns, bool need_navigation) {
  if (!s.failure.empty())
    return s.failure;
  if (!s.connected && !s.replay)
    return "UBX TCP is not connected";
  if (!supportedIdentity(s.identity))
    return "waiting for MON-VER ZED-F9P or ZED-F9K identity";
  if (!s.time.valid || s.times.size() < 3)
    return "need three consecutive valid NAV-TIMEGPS epochs";
  if (now < s.time.received || now - s.time.received > 2.5)
    return "NAV-TIMEGPS is stale";
  if (!need_navigation)
    return {};
  if (!s.position.valid || now - s.position.received > 2.5 ||
      now < s.position.received)
    return "valid fresh 3D NAV-PVT position required";
  double epoch_gap =
      std::fabs(static_cast<double>(s.position.itow_ms) - s.time.itow_ms) *
      1e-3;
  epoch_gap = std::min(epoch_gap, 604800 - epoch_gap);
  if (epoch_gap > 1.1)
    return "NAV-PVT and NAV-TIMEGPS epochs do not agree";
  if (now < s.raw_received || now - s.raw_received > 2.5)
    return "RXM-RAWX is stale";
  if (prns.empty())
    return "no GPS L1 C/A PRNs selected or observed";
  for (int prn : prns) {
    if (prn < 1 || prn > MAX_SAT)
      return "invalid selected GPS PRN";
    const auto &nav = s.navigation[prn - 1];
    if ((nav.coverage & 7) != 7) {
      std::string missing;
      for (unsigned sf = 1; sf <= 3; ++sf)
        if (!(nav.coverage & (1u << (sf - 1))))
          missing += (missing.empty() ? "" : ",") + std::string("SF") +
                     std::to_string(sf);
      return "PRN " + std::to_string(prn) + " missing " + missing;
    }
    if (!usableNavigation(nav, epoch, now))
      return "PRN " + std::to_string(prn) +
             " needs issue-consistent fresh healthy SF1-SF3";
  }
  return {};
}

GpsNowEstimate estimateGpsNow(const Snapshot &s, double now,
                              double delivery_delay) {
  if (s.replay)
    throw std::runtime_error("recorded UBX cannot supply current GPS time");
  if (!std::isfinite(now) || !std::isfinite(delivery_delay) || delivery_delay < 0 ||
      delivery_delay > 10)
    throw std::runtime_error("invalid GPS-now estimate inputs");
  auto problem = readiness(s, now, s.time.gps, {}, false);
  if (!problem.empty())
    throw std::runtime_error(problem);
  if (!std::isfinite(s.time.received) || !std::isfinite(s.time.gps.sec) ||
      s.time.gps.sec < 0 || s.time.gps.sec >= 604800 || s.time.gps.week < 0)
    throw std::runtime_error("invalid GPS-now receiver epoch");
  GpsNowEstimate estimate;
  estimate.gps = s.time.gps;
  estimate.host_monotonic = now;
  estimate.solution_age = now - s.time.received;
  estimate.delivery_delay = delivery_delay;
  double minimum = 0;
  for (const auto &t : s.times) {
    const double age = s.time.received - t.received;
    if (!t.valid || !std::isfinite(age) || age < 0 ||
        !std::isfinite(t.gps.sec) || t.gps.sec < 0 ||
        t.gps.sec >= 604800 || t.gps.week < 0)
      throw std::runtime_error("invalid GPS time observation");
    if (age > 5)
      continue;
    const double offset = subGpsTime(t.gps, s.time.gps) + age;
    minimum = std::min(minimum, offset);
    estimate.arrival_correction = std::max(estimate.arrival_correction, offset);
    estimate.window_seconds = std::max(estimate.window_seconds, age);
    ++estimate.observations;
  }
  if (estimate.observations < 3)
    throw std::runtime_error("insufficient recent time observations");
  estimate.arrival_variation = estimate.arrival_correction - minimum;
  // Suppress extra queuing delay, without fitting host drift into sample time.
  // delivery_delay estimates the remaining delay of the fastest observation;
  // it cannot be inferred from this one-way stream or from receiver tAcc.
  estimate.gps.sec += estimate.solution_age + estimate.arrival_correction +
                      delivery_delay;
  if (estimate.gps.sec >= 604800) {
    estimate.gps.sec -= 604800;
    ++estimate.gps.week;
  }
  return estimate;
}

std::string timeEstimateJson(const GpsNowEstimate &estimate) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"method\":\"minimum_delay_window\",\"observations\":"
      << estimate.observations << ",\"window_s\":" << estimate.window_seconds
      << ",\"arrival_correction_s\":" << estimate.arrival_correction
      << ",\"arrival_variation_s\":" << estimate.arrival_variation
      << ",\"delivery_delay_estimate_s\":" << estimate.delivery_delay
      << ",\"constant_delivery_bias_known\":false}";
  return out.str();
}

std::string snapshotJson(const Snapshot &s, double now) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"identity\":\"" << s.identity
      << "\",\"unique_id\":\"" << s.unique_id
      << "\",\"connected\":" << (s.connected ? "true" : "false")
      << ",\"replay\":" << (s.replay ? "true" : "false")
      << ",\"time_valid\":" << (s.time.valid ? "true" : "false")
      << ",\"gps_week\":" << s.time.gps.week
      << ",\"gps_tow\":" << s.time.gps.sec
      << ",\"time_received_monotonic_s\":" << s.time.received
      << ",\"time_age_s\":" << now - s.time.received
      << ",\"ftow_ns\":" << s.time.ftow_ns
      << ",\"receiver_tacc_ns\":" << s.time.accuracy_ns
      << ",\"consecutive_times\":" << s.times.size() << ",\"leap_seconds\":";
  if (s.time.leap_valid)
    out << s.time.leap_seconds;
  else
    out << "null";
  out << ",\"position_valid\":" << (s.position.valid ? "true" : "false")
      << ",\"latitude_deg\":" << s.position.latitude
      << ",\"longitude_deg\":" << s.position.longitude
      << ",\"ellipsoid_height_m\":" << s.position.height
      << ",\"horizontal_accuracy_m\":" << s.position.horizontal_accuracy
      << ",\"speed_m_s\":" << s.position.speed
      << ",\"satellites_used\":" << s.position.satellites
      << ",\"pvt_epochs\":" << s.pvt_epochs
      << ",\"valid_3d_epochs\":" << s.valid_3d_epochs
      << ",\"rawx_epochs\":" << s.rawx_epochs
      << ",\"nonempty_rawx_epochs\":" << s.nonempty_rawx_epochs
      << ",\"clock_itow_ms\":" << s.clock_itow_ms
      << ",\"clock_bias_ns\":" << s.clock_bias_ns
      << ",\"clock_drift_ns_s\":" << s.clock_drift_ns_s
      << ",\"rawx_week\":" << s.raw_epoch.week
      << ",\"rawx_tow\":" << s.raw_epoch.sec
      << ",\"rawx_measurements\":" << s.observations.size()
      << ",\"bytes\":" << s.bytes << ",\"frames\":" << s.frames
      << ",\"checksum_errors\":" << s.checksum_errors
      << ",\"malformed\":" << s.malformed
      << ",\"discarded_bytes\":" << s.discarded_bytes
      << ",\"time_rejections\":" << s.time_rejections
      << ",\"clock_resets\":" << s.clock_resets
      << ",\"hardware_edge_association_verified\":false,\"rf_monitor\":";
  if (!s.rf_present) {
    out << "null";
  } else {
    out << "{\"age_s\":" << now - s.rf_received << ",\"blocks\":[";
    for (size_t j = 0; j < s.rf_blocks.size(); ++j) {
      const auto &r = s.rf_blocks[j];
      if (j)
        out << ',';
      out << "{\"id\":" << r.id << ",\"jamming_state\":" << r.jamming_state
          << ",\"antenna_status\":" << r.antenna_status
          << ",\"antenna_power\":" << r.antenna_power
          << ",\"noise_per_ms\":" << r.noise_per_ms
          << ",\"agc_count\":" << r.agc_count
          << ",\"cw_suppression\":" << r.cw_suppression << '}';
    }
    out << "]}";
  }
  out << ",\"time_pulse\":";
  if (!s.time_pulse.present) {
    out << "null";
  } else {
    const auto &t = s.time_pulse;
    out << "{\"age_s\":" << now - t.received << ",\"week\":" << t.week
        << ",\"tow_ms\":" << t.tow_ms
        << ",\"tow_sub_ms_units_2_neg_32\":" << t.tow_sub_ms
        << ",\"quantization_error_ps\":" << t.quantization_error_ps
        << ",\"flags\":" << t.flags
        << ",\"reference_info\":" << t.reference_info
        << ",\"sequence\":" << t.sequence
        << ",\"quantization_error_valid\":" << ((t.flags & 16) ? "false" : "true")
        << ",\"labels_next_pulse\":true}";
  }
  out << ",\"pulse_history_size\":" << s.time_pulses.size()
      << ",\"pulse_duplicates\":" << s.pulse_duplicates
      << ",\"cfg_tp5_unsupported\":" << (s.tp5_unsupported ? "true" : "false")
      << ",\"pulse_config\":{\"present\":" << (s.pulse_config.present ? "true" : "false")
      << ",\"source\":\"" << (s.pulse_config.from_valget ? "CFG-VALGET" : "CFG-TP5") << '"'
      << ",\"age_s\":" << now - s.pulse_config.received
      << ",\"flags\":" << s.pulse_config.flags
      << ",\"locked_period\":" << s.pulse_config.locked_period
      << ",\"locked_length\":" << s.pulse_config.locked_length
      << ",\"user_delay_ns\":" << s.pulse_config.user_delay_ns << '}'
      << ",\"leap_info\":{\"present\":" << (s.leap_info.present ? "true" : "false")
      << ",\"age_s\":" << now - s.leap_info.received
      << ",\"valid\":" << s.leap_info.valid
      << ",\"current\":" << s.leap_info.current
      << ",\"change\":" << s.leap_info.change
      << ",\"seconds_to_event\":" << s.leap_info.seconds_to_event << '}';
  out << ",\"time_mark\":";
  if (!s.time_mark.present) {
    out << "null";
  } else {
    const auto &t = s.time_mark;
    out << "{\"age_s\":" << now - t.received << ",\"channel\":" << t.channel
        << ",\"flags\":" << t.flags << ",\"rising_edge_counter\":" << t.count
        << ",\"rising_week\":" << t.rising_week
        << ",\"rising_tow_ms\":" << t.rising_ms
        << ",\"rising_tow_sub_ns\":" << t.rising_sub_ns
        << ",\"falling_week\":" << t.falling_week
        << ",\"falling_tow_ms\":" << t.falling_ms
        << ",\"falling_tow_sub_ns\":" << t.falling_sub_ns
        << ",\"accuracy_ns\":" << t.accuracy_ns << '}';
  }
  out << ",\"rawx_observations\":[";
  for (size_t j = 0; j < s.observations.size(); ++j) {
    const auto &o = s.observations[j];
    if (j)
      out << ',';
    out << "{\"gnss\":" << o.gnss << ",\"prn\":" << o.prn
        << ",\"signal\":" << o.signal << ",\"tracking_flags\":" << o.flags
        << ",\"lock_ms\":" << o.lock_ms << ",\"cno_dbhz\":" << o.cno
        << ",\"pseudorange_m\":" << o.pseudorange
        << ",\"carrier_cycles\":" << o.carrier_cycles
        << ",\"doppler_hz\":" << o.doppler << '}';
  }
  out << "],\"navigation\":[";
  bool first = true;
  for (int prn = 1; prn <= MAX_SAT; ++prn) {
    const auto &n = s.navigation[prn - 1];
    if (!n.coverage)
      continue;
    if (!first)
      out << ',';
    first = false;
    out << "{\"prn\":" << prn << ",\"coverage_mask\":" << n.coverage
        << ",\"complete\":" << (n.complete ? "true" : "false")
        << ",\"iode\":" << n.eph.iode << ",\"iodc\":" << n.eph.iodc
        << ",\"health\":" << n.eph.svhlth << ",\"ura\":" << n.ura
        << ",\"extended_fit\":" << (n.extended_fit ? "true" : "false")
        << ",\"toe_week\":" << n.eph.toe.week
        << ",\"toe_tow\":" << n.eph.toe.sec
        << ",\"toc_week\":" << n.eph.toc.week
        << ",\"toc_tow\":" << n.eph.toc.sec << '}';
  }
  out << "]}";
  return out.str();
}
} // namespace ubx

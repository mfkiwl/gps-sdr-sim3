#ifndef TEST_FAKE_F9P_PPS_HPP
#define TEST_FAKE_F9P_PPS_HPP
#include "player/x300_pps_time.hpp"
#include <cstring>

// Real UBX frames with deterministic host receipt times; no sockets or RF.
struct FakeF9pPps {
  ubx::Decoder decoder;
  gpstime_t base{2347, 439122}; // GPS at fake monotonic zero
  int last_second = -1;
  bool utc_grid = true;
  int leap = 18;
  bool emit_pulses = true;
  // ZED-F9K: NAK CFG-TP5, answer CFG-VALGET CFG-TP-* instead.
  bool f9k = false;
  using Bytes = std::vector<uint8_t>;
  static void put(Bytes &b, size_t offset, uint32_t value, size_t length = 4) {
    for (size_t j = 0; j < length; ++j) b[offset + j] = (value >> (8 * j)) & 255;
  }
  void feed(unsigned cls, unsigned id, const Bytes &p, double at) {
    Bytes b{0xb5, 0x62, uint8_t(cls), uint8_t(id), uint8_t(p.size()), uint8_t(p.size() >> 8)};
    b.insert(b.end(), p.begin(), p.end());
    uint8_t a = 0, c = 0;
    for (size_t j = 2; j < b.size(); ++j) { a += b[j]; c += a; }
    b.push_back(a); b.push_back(c);
    decoder.feed(b.data(), b.size(), at);
  }
  // CFG-VALGET response (RAM layer) matching the CFG-TP5 fixture below.
  Bytes valget(uint8_t length_def = 1, double locked_duty = 10,
               uint8_t pulse_def = 0, uint8_t layer = 0,
               uint32_t omit = 0) const {
    const std::vector<std::pair<uint32_t, uint64_t>> items = {
        {0x30050001, 50},      {0x40050002, 1000000}, {0x40050003, 1000000},
        {0x40050004, 0},       {0x40050005, 100000},  {0x40050006, 0},
        {0x10050007, 1},       {0x10050008, 1},       {0x10050009, 1},
        {0x1005000a, 1},       {0x1005000b, 1},       {0x2005000c, utc_grid ? 0u : 1u},
        {0x20050023, pulse_def}, {0x40050024, 1},     {0x40050025, 1},
        {0x5005002a, 0},       {0x5005002b, 0},       {0x20050030, length_def}};
    static const size_t sizes[8] = {0, 1, 1, 2, 4, 8, 0, 0};
    Bytes p{1, layer, 0, 0};
    for (auto [key, value] : items) {
      if (key == omit) continue;
      if (key == 0x5005002b) std::memcpy(&value, &locked_duty, 8);
      const size_t size = sizes[key >> 28 & 7];
      for (int j = 0; j < 4; ++j) p.push_back(key >> (8 * j) & 255);
      for (size_t j = 0; j < size; ++j) p.push_back(value >> (8 * j) & 255);
    }
    return p;
  }
  gpstime_t gps(double at) const {
    auto g = base; g.sec += at; return x300::sampleTime(g, 0, 1);
  }
  double unixUtc(double at) const {
    return 315964800.0 + base.week * 604800.0 + base.sec + at - leap;
  }
  void pulse(double second, double received, int32_t qerr = 777) {
    auto g = gps(second - (utc_grid ? leap : 0));
    Bytes p(16); put(p, 0, uint32_t(std::llround(g.sec * 1000)));
    put(p, 8, qerr); put(p, 12, g.week, 2);
    p[14] = utc_grid ? 27 : 16; p[15] = utc_grid ? 63 : 0;
    feed(13, 1, p, received);
  }
  void advance(double now) {
    if (last_second < 0) {
      decoder.state.connected = true;
      Bytes id(70);
      std::memcpy(id.data() + 40, f9k ? "MOD=ZED-F9K" : "MOD=ZED-F9P", 11);
      feed(10, 4, id, now - 3);
      last_second = int(std::floor(now)) - 4;
    }
    const int latest = int(std::floor(now - .1 + 1e-9));
    while (last_second < latest) {
      const int n = ++last_second;
      const double at = n + .1;
      auto g = gps(n);
      Bytes t(16); put(t, 0, uint32_t(std::llround(g.sec * 1000)));
      put(t, 8, g.week, 2); t[10] = leap; t[11] = 7; put(t, 12, 10);
      feed(1, 0x20, t, at);
      Bytes cfg(32); cfg[1] = 1; put(cfg, 4, 50, 2);
      put(cfg, 8, 1000000); put(cfg, 12, 1000000); put(cfg, 20, 100000);
      put(cfg, 28, 0x77 | (utc_grid ? 0 : 128));
      if (f9k) { feed(5, 0, {6, 0x31}, at); feed(6, 0x8b, valget(), at); }
      else feed(6, 0x31, cfg, at);
      Bytes ls(24); ls[9] = leap; ls[23] = 3; put(ls, 12, 1000000);
      feed(1, 0x26, ls, at);
      if (emit_pulses) pulse(n + 1, at);
    }
  }
};
#endif

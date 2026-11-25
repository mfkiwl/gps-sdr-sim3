#include "fake_f9p_pps.hpp"
#include "player/x300_live_time.hpp"
#include <cassert>
#include <iostream>

template <typename F> void rejects(F call, const std::string &message) {
  bool failed = false;
  try { call(); } catch (const std::exception &e) {
    failed = true;
    if (std::string(e.what()).find(message) == std::string::npos) {
      std::cerr << "Expected " << message << "; got " << e.what() << '\n';
      std::abort();
    }
  }
  if (!failed) { std::cerr << "Expected rejection: " << message << '\n'; std::abort(); }
}
struct Fixture {
  FakeF9pPps source;
  x300::PpsAssociation association{{.2, .8}, 100.25};
  double now = 100.25;
  x300::PpsObservation observation() const {
    return {now, now, std::floor(now), source.unixUtc(now), now};
  }
  void step(double at) {
    now = at; source.advance(now);
    association.observe(source.decoder.state, observation());
  }
  void acquire() {
    step(100.25);
    for (int n = 101; n <= 104; ++n) { step(n + .01); step(n + .2); }
    assert(association.reference().verified);
  }
};
void testParserAndRollover() {
  Fixture f; f.source.base = {2347, 604697}; f.acquire();
  const auto ref = f.association.reference();
  assert(ref.gps.week == 2347 && ref.gps.sec == 604799);
  const auto &s = f.source.decoder.state;
  assert(s.time.gps.week == 2348 && s.time.gps.sec == 1);
  assert(s.pulse_config.present && s.pulse_config.antenna_delay_ns == 50);
  const auto count = s.time_pulses.size();
  const auto received = s.time_pulse.received;
  f.source.pulse(105, 104.4);
  assert(s.time_pulses.size() == count && s.time_pulse.received == received);
  assert(s.pulse_duplicates == 1);
  assert(x300::ppsReferenceJson(ref).find("\"qerr_valid\":false") != std::string::npos);
  assert(x300::ppsReferenceJson(ref).find("\"qerr_applied\":false") != std::string::npos);
  auto p = x300::planPpsStart(s, {104.4, 104.4, 104.4}, ref, 2, .0001, .00004);
  assert(p.hardware_start == 107 && p.gps_zero.week == 2348);
  assert(std::fabs(p.gps_zero.sec - 4.00006) < 1e-9);
  assert(p.estimate.observations == 0 && p.delivery_delay == 0);
  auto later = x300::sampleTime(p.gps_zero, 2500000, 2500000);
  assert(std::fabs(later.sec - 5.00006) < 1e-9);
  Fixture gps; gps.source.utc_grid = false; gps.acquire();
  assert(gps.association.reference().gps.sec == 439224);
  Fixture future; future.source.leap = 19; future.acquire();
  assert(future.association.reference().gps.sec == 439224);
  Fixture utc_rollover; utc_rollover.source.base = {2347, 604714};
  utc_rollover.acquire();
  assert(utc_rollover.association.reference().gps.week == 2348);
  assert(utc_rollover.association.reference().gps.sec == 16);
  assert(utc_rollover.source.decoder.state.time_pulse.week == 2348);
  auto valid_qerr = gps.source.decoder.state.time_pulse;
  valid_qerr.flags &= ~16u;
  assert(x300::pulseGps(valid_qerr, gps.source.decoder.state).sec == 439227);
}
void testAmbiguity() {
  for (int offset : {-1, 1}) {
    Fixture f; f.step(100.25); f.now = 101.2; f.source.advance(f.now);
    auto &p = f.source.decoder.state.time_pulses.back();
    p.tow_ms += offset * 1000; // previous/next second confusion
    rejects([&] { f.association.observe(f.source.decoder.state, f.observation()); }, "ambiguous/late");
  }
  Fixture delayed; delayed.step(100.25); delayed.now = 102.2;
  delayed.source.advance(delayed.now);
  // Correct sequence with an entire second of extra transport delay.
  for (auto &p : delayed.source.decoder.state.time_pulses) p.received += 1;
  delayed.source.decoder.state.time_pulses.pop_back();
  rejects([&] { delayed.association.observe(delayed.source.decoder.state, delayed.observation()); }, "ambiguous/late");
  Fixture late; late.step(100.25); late.now = 101.95; late.source.advance(late.now);
  late.source.decoder.state.time_pulses.back().received = late.now;
  rejects([&] { late.association.observe(late.source.decoder.state, late.observation()); }, "delivery bound");
  Fixture bracket; bracket.step(100.25); bracket.now = 101.2; bracket.source.advance(bracket.now);
  auto o = bracket.observation(); o.after += .051;
  rejects([&] { bracket.association.observe(bracket.source.decoder.state, o); }, "observation");
}
void testFailures() {
  const std::vector<std::pair<std::string, std::function<void(Fixture &)>>> cases = {
    {"duplicate", [](Fixture &f) { f.source.pulse(105, 104.3, 888); }},
    {"ambiguous/late", [](Fixture &f) { f.source.pulse(104, 104.3); }},
    {"stale", [](Fixture &f) { auto &p=f.source.decoder.state.time_pulses.back(); ++p.sequence; p.received -= 4; }},
    {"history overrun", [](Fixture &f) { f.source.pulse(106, 105.1); f.source.decoder.state.time_pulses.back().sequence += 2; }},
    {"reset", [](Fixture &f) { ++f.source.decoder.state.clock_resets; }},
    {"reset", [](Fixture &f) { ++f.source.decoder.state.time_rejections; }},
    {"1 Hz rising", [](Fixture &f) { f.source.decoder.state.pulse_config.flags &= ~64; }},
    {"1 Hz rising", [](Fixture &f) { f.source.decoder.state.pulse_config.locked_period = 500000; }},
    {"configuration changed", [](Fixture &f) { ++f.source.decoder.state.pulse_config.antenna_delay_ns; }},
    {"leap information", [](Fixture &f) { f.source.decoder.state.time.leap_valid = false; }},
    {"leap information", [](Fixture &f) { f.source.decoder.state.leap_info.valid = 1; }},
    {"leap information", [](Fixture &f) { ++f.source.decoder.state.leap_info.current; }},
    {"leap-second", [](Fixture &f) { f.source.decoder.state.leap_info.seconds_to_event = 5; }},
    {"leap-second", [](Fixture &f) { f.source.decoder.state.leap_info.change = -1; f.source.decoder.state.leap_info.seconds_to_event = 5; }},
    {"fresh CFG-TP5", [](Fixture &f) { f.source.decoder.state.pulse_config.received -= 11; }},
    {"not connected", [](Fixture &f) { f.source.decoder.state.connected = false; }},
    {"fresh live", [](Fixture &f) { f.source.decoder.state.replay = true; }}
  };
  for (const auto &c : cases) {
    Fixture f; f.acquire(); c.second(f); f.now = 104.4;
    rejects([&] { f.association.observe(f.source.decoder.state, f.observation()); }, c.first);
  }
  Fixture missing; missing.acquire(); missing.source.emit_pulses = false;
  rejects([&] { missing.step(106.2); }, "stream stopped");
  Fixture skipped; skipped.acquire(); skipped.source.emit_pulses = false;
  skipped.source.advance(105.2); skipped.source.emit_pulses = true;
  rejects([&] { skipped.step(106.2); }, "skipped");
  Fixture drift; drift.acquire(); auto o = drift.observation(); o.pps += .000002;
  rejects([&] { drift.association.observe(drift.source.decoder.state, o); }, "drift");
  o = drift.observation(); o.pps -= 2;
  rejects([&] { drift.association.observe(drift.source.decoder.state, o); }, "observation");
  o = drift.observation(); o.hardware -= 3;
  rejects([&] { drift.association.observe(drift.source.decoder.state, o); }, "observation");
  o = drift.observation(); o.utc += 1;
  rejects([&] { drift.association.observe(drift.source.decoder.state, o); }, "stepped");
  Fixture host; host.step(100.25); host.now = 101.2; host.source.advance(host.now);
  auto wrong = host.observation(); wrong.utc += .3;
  // Start with the coarse error already present: epoch residual still fails.
  x300::PpsAssociation bad_clock({.2, .8}, 101);
  rejects([&] { bad_clock.observe(host.source.decoder.state, wrong); }, "host UTC bound");
  Fixture missed_edge; missed_edge.step(100.25); missed_edge.step(101.2);
  missed_edge.step(102.01);
  rejects([&] { missed_edge.step(104.2); }, "skipped hardware PPS");
  Fixture labels; labels.acquire();
  auto bad_label = labels.source.decoder.state.time_pulse;
  bad_label.tow_sub_ms = 1;
  rejects([&] { x300::pulseGps(bad_label, labels.source.decoder.state); }, "invalid TIM-TP");
  bad_label = labels.source.decoder.state.time_pulse;
  bad_label.reference_info = 255;
  rejects([&] { x300::pulseGps(bad_label, labels.source.decoder.state); }, "invalid TIM-TP");
  bad_label = labels.source.decoder.state.time_pulse;
  bad_label.flags &= ~2u;
  rejects([&] { x300::pulseGps(bad_label, labels.source.decoder.state); }, "invalid TIM-TP");
}
void testF9kValget() {
  Fixture tp5; tp5.acquire();
  Fixture f9k; f9k.source.f9k = true; f9k.acquire();
  const auto &s = f9k.source.decoder.state;
  assert(ubx::supportedIdentity(s.identity) && s.tp5_unsupported);
  // Same TIMEPULSE0 settings as CFG-TP5, differing only in provenance.
  auto c = s.pulse_config;
  assert(c.present && c.from_valget && c.flags == 0x77 &&
         c.locked_length == 100000 && c.antenna_delay_ns == 50);
  c.from_valget = false;
  assert(x300::samePulseConfig(c, tp5.source.decoder.state.pulse_config));
  const auto ref = f9k.association.reference();
  assert(ref.gps.week == tp5.association.reference().gps.week &&
         ref.gps.sec == tp5.association.reference().gps.sec);
  const auto json = x300::ppsReferenceJson(ref);
  assert(json.find("\"pulse_config_source\":\"CFG-VALGET\"") != std::string::npos);
  assert(json.find("\"receiver_rf_delay_setting_ns\":null") != std::string::npos);
  assert(x300::ppsReferenceJson(tp5.association.reference())
             .find("\"pulse_config_source\":\"CFG-TP5\"") != std::string::npos);
  Fixture gps; gps.source.f9k = true; gps.source.utc_grid = false; gps.acquire();
  assert(gps.association.reference().gps.sec == 439224);

  // F9P: CFG-TP5 stays authoritative; a VALGET reply never overrides it.
  auto &p = tp5.source.decoder.state;
  tp5.source.feed(6, 0x8b, tp5.source.valget(1, 50), 104.3);
  assert(!p.tp5_unsupported && !p.pulse_config.from_valget &&
         p.pulse_config.locked_length == 100000);

  FakeF9pPps d; d.f9k = true;
  auto &q = d.decoder.state;
  d.feed(6, 0x8b, d.valget(), 1); // before the NAK: ignored
  assert(!q.pulse_config.present && !q.malformed);
  d.feed(5, 0, {6, 0x31}, 1);
  d.feed(6, 0x8b, d.valget(1, 10, 0, 7), 1); // default layer, not RAM
  d.feed(6, 0x8b, d.valget(1, 10, 0, 0, 0x2005000c), 1); // partial
  assert(q.tp5_unsupported && !q.pulse_config.present && !q.malformed);
  d.feed(6, 0x8b, d.valget(0, 10), 2); // duty ratio in CFG-TP5 2^-32 units
  assert(q.pulse_config.present && !(q.pulse_config.flags & 16) &&
         q.pulse_config.locked_length == 429496729u);
  auto truncated = d.valget(); truncated.pop_back();
  d.feed(6, 0x8b, truncated, 3);
  d.feed(6, 0x8b, d.valget(1, 10, 2), 3); // invalid pulse definition
  d.feed(6, 0x8b, d.valget(0, 150), 3);   // invalid duty
  assert(q.malformed == 3 && q.pulse_config.received == 2);

  const std::vector<std::pair<std::string, std::function<void(Fixture &)>>> cases = {
    {"configuration changed", [](Fixture &f) { f.source.decoder.state.pulse_config.from_valget = false; }},
    {"configuration changed", [](Fixture &f) { ++f.source.decoder.state.pulse_config.antenna_delay_ns; }},
    {"1 Hz rising", [](Fixture &f) { f.source.decoder.state.pulse_config.flags &= ~4; }},
    {"CFG-VALGET TIMEPULSE0 readback", [](Fixture &f) { f.source.decoder.state.pulse_config.received -= 11; }},
    {"ZED-F9K identity", [](Fixture &f) { f.source.decoder.state.identity = "MOD=NEO-M8T"; }}
  };
  for (const auto &c : cases) {
    Fixture f; f.source.f9k = true; f.acquire(); c.second(f); f.now = 104.4;
    rejects([&] { f.association.observe(f.source.decoder.state, f.observation()); }, c.first);
  }
}
int main() {
  testParserAndRollover(); testAmbiguity(); testFailures(); testF9kValget();
  std::cout << "PPS epoch association, parser, UTC/leap/week rollover, ZED-F9K "
               "CFG-VALGET and rejection tests passed\n";
}

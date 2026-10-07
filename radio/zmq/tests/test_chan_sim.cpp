/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include <gtest/gtest.h>
#include "chan_sim.h"

#include <sstream>

using namespace chsim;

namespace {

constexpr double FS = 1e4; // small rate: the 1.5 s calibration is 15000 samples
constexpr uint64_t ORIGIN = 100; // stream origin, as zmq_radio passes fs/100
constexpr float AMP = 0.1f; // signal amplitude; reference power = AMP^2

uint64_t at(double t)
{
  return ORIGIN + (uint64_t)std::llround(t * FS);
}

// Runs n samples of a constant signal through the link starting at air time t; returns the samples.
std::vector<cf> run(chsim::link &l, double t, size_t n, float amp = AMP)
{
  std::vector<cf> x(n, cf(amp, 0));
  l.process(x.data(), n, at(t));
  return x;
}

// Locks the reference of a fresh link: three 0.5 s blocks of full-load signal.
void calibrate(chsim::link &l)
{
  for (int i = 0; i < 3; i++)
    run(l, 0.5 * i, 5000);
  ASSERT_TRUE(l.st.calibrated.load());
}

config base_cfg()
{
  config c;
  EXPECT_EQ(parse("distance=50,dl_snr_ref=30,ul_snr_ref=30,ul_noise_dbfs=-65,trace_ctl=/tmp/ue1.ctl", c), "");
  return c;
}

std::shared_ptr<const trace> make_trace(const std::string &name,
                                        std::vector<double> t,
                                        std::vector<double> dl,
                                        std::vector<double> ul)
{
  std::ostringstream csv;
  csv << "t_s,dl_snr_db,ul_snr_db\n";
  for (size_t i = 0; i < t.size(); i++)
    csv << t[i] << "," << dl[i] << "," << ul[i] << "\n";
  std::istringstream in(csv.str());
  auto tr = std::make_shared<trace>();
  EXPECT_EQ(load_trace(in, *tr), "");
  tr->name = name;
  return tr;
}

} // namespace

TEST(chan_sim_parse, trace_ctl)
{
  config c;
  EXPECT_EQ(parse("trace_ctl=/tmp/traces/ue1.ctl,ul_noise_dbfs=-65", c), "");
  EXPECT_EQ(c.trace_ctl, "/tmp/traces/ue1.ctl");
  EXPECT_NE(parse("trace_ctl=/tmp/traces/ue1.ctl", c).find("ul_noise_dbfs"), std::string::npos);
  EXPECT_NE(parse("trace_ctl=ue1.ctl,ul_noise_dbfs=-65", c).find("absolute"), std::string::npos);
  EXPECT_NE(parse("bogus=1", c).find("unknown option"), std::string::npos);
}

TEST(chan_sim_trace, load_valid)
{
  std::istringstream in("t_s,dl_snr_db,ul_snr_db,d_m\r\n0,20,15,50\r\n0.01,19.5,14.5,51\n\n0.02,19,14,52\n");
  trace tr;
  ASSERT_EQ(load_trace(in, tr), "");
  ASSERT_EQ(tr.t.size(), 3u);
  EXPECT_DOUBLE_EQ(tr.dl[1], 19.5);
  EXPECT_DOUBLE_EQ(tr.ul[2], 14);
  EXPECT_NEAR(tr.period, 0.03, 1e-12);
}

TEST(chan_sim_trace, load_errors)
{
  const char *bad[] = {
      "", // empty
      "t,dl,ul\n0,1,2\n", // header
      "t_s,dl_snr_db,ul_snr_dbX\n0,1,2\n", // header must match whole columns
      "t_s,dl_snr_db,ul_snr_db\n", // no rows
      "t_s,dl_snr_db,ul_snr_db\n0.5,1,2\n", // must start at 0
      "t_s,dl_snr_db,ul_snr_db\n0,1,2\n1,1,2\n1,1,2\n", // must increase
      "t_s,dl_snr_db,ul_snr_db\n0,25dB,2\n", // whole field must be a number
      "t_s,dl_snr_db,ul_snr_db\n0,1\n", // missing value
      "t_s,dl_snr_db,ul_snr_db\n0,nan,2\n", // not finite
  };
  for (const char *text : bad) {
    std::istringstream in(text);
    trace tr;
    EXPECT_NE(load_trace(in, tr), "") << text;
  }
}

TEST(chan_sim_trace, lookup_step_hold_loop)
{
  auto tr = make_trace("a", {0, 1, 2}, {10, 20, 30}, {1, 2, 3});
  EXPECT_DOUBLE_EQ(tr->period, 3);
  double dl, ul;
  size_t row;
  const std::pair<double, size_t> looped[] = {{0, 0}, {0.99, 0}, {1, 1}, {2.99, 2}, {3, 0}, {4.5, 1}, {9.2, 0}};
  for (auto [tt, want] : looped) {
    trace_at(*tr, tt, true, dl, ul, row);
    EXPECT_EQ(row, want) << tt;
  }
  trace_at(*tr, 10, false, dl, ul, row); // hold the last row
  EXPECT_EQ(row, 2u);
  EXPECT_DOUBLE_EQ(dl, 30);
  EXPECT_DOUBLE_EQ(ul, 3);

  auto one = make_trace("c", {0}, {12}, {8}); // a single row is a constant
  EXPECT_DOUBLE_EQ(one->period, 0);
  trace_at(*one, 1234.5, true, dl, ul, row);
  EXPECT_EQ(row, 0u);
  EXPECT_DOUBLE_EQ(dl, 12);
}

TEST(chan_sim_trace, parse_ctl)
{
  ctl_cmd c;
  EXPECT_EQ(parse_ctl("# switch ue1\ntrace=/tmp/traces/a.csv\nstart=12.5\nloop=0\n", c), "");
  EXPECT_EQ(c.trace_path, "/tmp/traces/a.csv");
  EXPECT_DOUBLE_EQ(c.start, 12.5);
  EXPECT_FALSE(c.loop);
  EXPECT_EQ(parse_ctl("trace=none\nstart=3\n", c), "");
  EXPECT_EQ(c.trace_path, "none");
  EXPECT_TRUE(c.loop);
  const char *bad[] = {"trace=/a.csv\n",
                       "start=1\n",
                       "trace=a.csv\nstart=1\n",
                       "trace=/a.csv\nstart=1\nloop=2\n",
                       "trace=/a.csv\nstart=x\n",
                       "trace=/a.csv\nstart=1\nfoo=1\n",
                       "trace /a.csv\nstart=1\n"};
  for (const char *text : bad)
    EXPECT_NE(parse_ctl(text, c), "") << text;
}

TEST(chan_sim_trace, pending_switches)
{
  trace_source src;
  auto A = make_trace("A", {0}, {1}, {1}), B = make_trace("B", {0}, {2}, {2}), C = make_trace("C", {0}, {3}, {3});
  src.publish(A, 10, true, 0);
  EXPECT_EQ(state_at(src.get().get(), 5), nullptr); // base until A starts
  EXPECT_EQ(state_at(src.get().get(), 10)->tr, A);
  src.publish(B, 20, true, 12);
  src.publish(C, 30, true, 18); // B still pending: A until 20, B until 30, then C
  const trace_state *s = src.get().get();
  EXPECT_EQ(state_at(s, 19)->tr, A);
  EXPECT_EQ(state_at(s, 25)->tr, B);
  EXPECT_EQ(state_at(s, 31)->tr, C);
  src.publish(nullptr, 40, true, 35); // states already superseded are dropped
  int depth = 0;
  for (s = src.get().get(); s; s = s->prev.get())
    depth++;
  EXPECT_EQ(depth, 2); // the stop and C, which plays until it
}

TEST(chan_sim_link, replays_and_switches_in_both_directions)
{
  const config c = base_cfg();
  auto src = std::make_shared<trace_source>();
  chsim::link dl(chsim::link::dir::dl, c, FS, ORIGIN, src), ul(chsim::link::dir::ul, c, FS, ORIGIN, src);
  calibrate(dl);
  calibrate(ul);

  auto snr = [&](double t) {
    run(dl, t, 100);
    run(ul, t, 100);
    EXPECT_NEAR(ul.st.ul_gain_db.load(), ul.st.ul_snr_db.load() - 65 - ul.st.ref_dbfs.load(), 1e-9);
    return std::make_pair(dl.st.dl_snr_db.load(), ul.st.ul_snr_db.load());
  };
  EXPECT_EQ(snr(1.6), std::make_pair(30.0, 30.0)); // base channel
  EXPECT_EQ(dl.st.trace_row.load(), -1);

  auto A = make_trace("A", {0, 1}, {20, 10}, {15, 5}); // period 2 s
  src->publish(A, 3.0, true, 1.6);
  EXPECT_EQ(snr(2.99), std::make_pair(30.0, 30.0)); // not yet
  EXPECT_EQ(snr(3.0), std::make_pair(20.0, 15.0)); // switches on the sample, both directions
  EXPECT_NE(dl.where().find("trace A row 0"), std::string::npos);
  EXPECT_EQ(snr(4.2), std::make_pair(10.0, 5.0)); // step-hold, row 1
  EXPECT_EQ(snr(5.3), std::make_pair(20.0, 15.0)); // looped

  auto B = make_trace("B", {0}, {12}, {8});
  src->publish(B, 6.0, false, 5.5);
  EXPECT_EQ(snr(5.9), std::make_pair(20.0, 15.0)); // A until B's start: no gap
  EXPECT_EQ(snr(6.0), std::make_pair(12.0, 8.0));

  src->publish(nullptr, 7.0, true, 6.5); // back to the base channel
  EXPECT_EQ(snr(7.0), std::make_pair(30.0, 30.0));
  EXPECT_NE(dl.where().find("d 50.0 m"), std::string::npos);

  src->publish(A, 8.0, true, 9.0); // noticed late: stays in phase with its T
  EXPECT_EQ(snr(9.2), std::make_pair(10.0, 5.0));
}

TEST(chan_sim_link, applies_the_trace_snr)
{
  const config c = base_cfg();
  auto src = std::make_shared<trace_source>();
  chsim::link dl(chsim::link::dir::dl, c, FS, ORIGIN, src), ul(chsim::link::dir::ul, c, FS, ORIGIN, src);
  calibrate(dl);
  calibrate(ul);
  src->publish(make_trace("C", {0}, {20}, {12}), 2.0, true, 1.5);

  // DL: on a silent input the output is the added noise alone, at reference power - SNR
  std::vector<cf> x = run(dl, 2.0, 200000, 0.0f);
  double p = 0;
  for (auto v : x)
    p += std::norm(v);
  EXPECT_NEAR(10 * std::log10(p / x.size()), 10 * std::log10(AMP * AMP) - 20, 0.1);

  // UL: samples scaled so the signal sits ul_snr above the -65 dBFS noise floor
  std::vector<cf> y = run(ul, 2.0, 100);
  EXPECT_NEAR(20 * std::log10(std::abs(y[0])), -65 + 12, 1e-4);
}

TEST(chan_sim_link, model_uses_air_time_in_both_directions)
{
  config c;
  ASSERT_EQ(parse("distance=50,speed=36,min_distance=40,max_distance=60,dl_snr_ref=25,ul_noise_dbfs=-65", c), "");
  chsim::link dl(chsim::link::dir::dl, c, FS, ORIGIN), ul(chsim::link::dir::ul, c, FS, ORIGIN);
  calibrate(dl);
  for (int i = 0; i < 3; i++) // the UL starts 0.3 s later than the DL
    run(ul, 0.3 + 0.5 * i, 5000);
  run(dl, 2.4, 100);
  run(ul, 2.4, 100);
  EXPECT_DOUBLE_EQ(dl.st.distance_m.load(), c.distance_at(2.4));
  EXPECT_DOUBLE_EQ(ul.st.distance_m.load(), c.distance_at(2.4));
  EXPECT_DOUBLE_EQ(dl.st.dl_snr_db.load(), std::max(25 - c.loss_db_at(2.4), 8.0));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 *
 * Minimal mobility-driven channel for the ZMQ radio: distance -> path loss -> SNR.
 * No fading, no delay spread, no Doppler -- just a scalar level that moves as the UE moves.
 *
 *   d(t)      distance, from the starting distance and speed, bouncing between min and max
 *   loss(t)   path_loss_exp * log10( d(t) / d_start )        dB, relative to the start
 *   DL        white Gaussian noise added so the UE sees dl_snr_ref - loss(t)
 *   UL        transmit samples attenuated by loss(t), against the gNB noise the broker adds
 *
 * Instead of the model, a UE can replay SNR traces (DL and UL SNR over time), switched at run time
 * through a control file (trace_ctl=, see trace_source). The model is then the base channel: what
 * plays before the first trace and after a trace=none.
 *
 * Time is air time: samples since the stream origin over the sample rate. It is the same for the DL,
 * the UL and every UE behind the broker, so traces switch at the same gNB sample everywhere.
 */

#ifndef ZMQ_CHAN_SIM_H
#define ZMQ_CHAN_SIM_H

#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <istream>
#include <memory>
#include <string>
#include <vector>

namespace chsim {

using cf = std::complex<float>;

struct config {
  double distance_m = 100;      // starting distance
  double speed_kmh = 0;         // 0 = stationary
  double min_m = 0;             // bounce limits; min >= max disables bouncing (moves away forever)
  double max_m = 0;
  double dl_snr_ref = 25;       // DL SNR in dB at the starting distance
  double ul_snr_ref = 25;       // UL SNR in dB at the gNB at the starting distance
  double ul_noise_dbfs = NAN;   // the noise floor the broker adds; NaN = leave the UL alone
  double path_loss_exp = 37.6;  // dB per decade of distance (3GPP-ish urban default)
  // SNR floors. Distance alone could otherwise walk the UE below the point where the gNB releases
  // it, and this OAI UE never reconnects after a release.
  double dl_snr_min = 8;
  double ul_snr_min = 5;
  // Slow random SNR offset (shadowing), standard deviation in dB, 0 = off. Path loss alone changes
  // too smoothly for link adaptation to ever fall behind, so this is what produces real block
  // errors. DL and UL get the same offset, as one obstruction would affect both.
  double snr_jitter_db = 0;
  uint64_t seed = 0; // fixed per UE by the wrappers (seed=<ue>)
  std::string trace_ctl; // control file for trace replay (absolute path); empty = model only

  double distance_at(double t_sec) const;
  double loss_db_at(double t_sec) const;   // path loss relative to the starting distance
  double jitter_db_at(double t_sec) const; // shadowing offset at this instant
};

// Returns an empty string on success, else an error message. Spec is "key=value,key=value".
std::string parse(const std::string &spec, config &out);
std::string describe(const config &cfg);

// SNR over time. Step-hold: row i holds from t[i] until t[i+1].
struct trace {
  std::string name; // for logs
  std::vector<double> t; // seconds; t[0] == 0, strictly increasing
  std::vector<double> dl; // DL SNR, dB
  std::vector<double> ul; // UL SNR at the gNB, dB
  double period = 0; // loop length: last t plus the last step; 0 for a single row (constant)
};

// Reads a trace CSV: header "t_s,dl_snr_db,ul_snr_db" (more columns are ignored), one row per line.
std::string load_trace(std::istream &in, trace &out);

// The SNRs tt seconds into the trace, looping or holding the last row; row is the index used.
void trace_at(const trace &tr, double tt, bool loop, double &dl_snr, double &ul_snr, size_t &row);

// A control file: "trace=<csv path>|none", "start=<air time, s>", optional "loop=0|1", one per line.
struct ctl_cmd {
  std::string trace_path; // "none" = back to the base channel
  double start = 0;
  bool loop = true;
};
std::string parse_ctl(const std::string &text, ctl_cmd &out);

// What a UE plays from air time T on. Immutable once published. Until T the state it replaced
// (prev) stays in force, so a switch lands on the same sample in both directions, with no gap.
struct trace_state {
  std::shared_ptr<const trace> tr; // nullptr = the base channel
  double T = 0;
  bool loop = true;
  std::shared_ptr<const trace_state> prev;
};

// The latest trace_state of one UE: written by its control thread, read by its DL and UL links.
class trace_source {
 public:
  std::shared_ptr<const trace_state> get() const
  {
    return std::atomic_load(&cur_);
  }
  // Takes effect at air time T; t_now (current air time) drops states that can no longer apply.
  void publish(std::shared_ptr<const trace> tr, double T, bool loop, double t_now);

 private:
  std::shared_ptr<const trace_state> cur_;
};

// The state in force at air time t, walking back past switches that have not happened yet.
const trace_state *state_at(const trace_state *s, double t);

// One-time reference level, so dl_snr_ref/ul_snr_ref mean what they say. It is the power of a
// *fully loaded* slot: the largest per-block mean seen over the calibration window. Taking a median
// or average instead would make the reference depend on how busy the link happened to be while
// calibrating -- measured ~4 dB low when calibrating on an idle DL, which left the real SNR that
// much above the requested value and pinned CQI high. The peak is load-independent because the
// gNB's SSB and SIB slots are always full-band.
class power_probe
{
 public:
  explicit power_probe(uint64_t samples_needed) : need_(samples_needed) {}
  void feed(const cf *x, size_t n);
  bool locked() const { return locked_; }
  double power() const { return power_; } // linear, mean |x|^2 of a fully loaded slot

 private:
  uint64_t need_;
  uint64_t seen_ = 0;
  double peak_ = 0;
  bool locked_ = false;
  double power_ = 0;
};

struct stats {
  std::atomic<double> t_air{0};
  std::atomic<double> distance_m{0};
  std::atomic<double> dl_snr_db{0};
  std::atomic<double> ul_snr_db{0};
  std::atomic<double> ul_gain_db{0};
  std::atomic<double> ref_dbfs{0};
  std::atomic<bool> calibrated{false};
  std::atomic<long> trace_row{-1}; // -1 = base channel
  std::atomic<double> trace_T{0};
};

// One direction of the link. Processes samples in place; t0 is the absolute sample time of x[0] and
// origin the sample time of the stream start, so air time = (t0 - origin) / fs.
class link
{
 public:
  enum class dir { dl, ul };
  link(dir d, const config &cfg, double fs, uint64_t origin, std::shared_ptr<trace_source> traces = nullptr);
  void process(cf *x, size_t n, uint64_t t0);
  // "trace <name> row <i> (T <s>)" or "d <m> m", for the once-per-second log; same thread as process()
  std::string where() const;
  stats st;

 private:
  dir d_;
  config cfg_;
  double fs_;
  uint64_t origin_;
  std::shared_ptr<trace_source> traces_;
  std::shared_ptr<const trace_state> state_; // keeps the trace in use alive
  const trace *playing_ = nullptr;
  power_probe probe_;
  uint64_t rng_;
};

} // namespace chsim

#endif

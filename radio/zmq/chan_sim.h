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
 * The UL is attenuated rather than pinned to a target SNR, so the UE's own power control stays
 * in charge of its transmit power, as on a real link.
 */

#ifndef ZMQ_CHAN_SIM_H
#define ZMQ_CHAN_SIM_H

#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
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
  uint64_t seed = 0;            // 0 = random

  double distance_at(double t_sec) const;
  double loss_db_at(double t_sec) const;   // path loss relative to the starting distance
  double jitter_db_at(double t_sec) const; // shadowing offset at this instant
};

// Returns an empty string on success, else an error message. Spec is "key=value,key=value".
std::string parse(const std::string &spec, config &out);
std::string describe(const config &cfg);

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
  std::atomic<double> distance_m{0};
  std::atomic<double> dl_snr_db{0};
  std::atomic<double> ul_gain_db{0};
  std::atomic<double> ref_dbfs{0};
  std::atomic<bool> calibrated{false};
};

// One direction of the link. Processes samples in place; t0 is the absolute sample time of x[0].
class link
{
 public:
  enum class dir { dl, ul };
  link(dir d, const config &cfg, double fs);
  void process(cf *x, size_t n, uint64_t t0);
  stats st;

 private:
  dir d_;
  config cfg_;
  double fs_;
  power_probe probe_;
  uint64_t first_t_ = UINT64_MAX;
  uint64_t rng_;
};

} // namespace chsim

#endif

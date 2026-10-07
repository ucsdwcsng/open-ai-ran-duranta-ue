/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "chan_sim.h"

#include <algorithm>
#include <mutex>
#include <random>
#include <sstream>

namespace chsim {

namespace {

constexpr size_t NOISE_TABLE = 1u << 20;

// Unit-power complex Gaussian table, built once and read at a random offset per block.
const std::vector<cf> &noise_table()
{
  static std::vector<cf> t;
  static std::once_flag once;
  std::call_once(once, [] {
    std::mt19937_64 gen(0xC0FFEEULL);
    std::normal_distribution<float> nd(0.0f, std::sqrt(0.5f)); // 0.5 per component -> unit total
    t.resize(NOISE_TABLE);
    for (auto &v : t)
      v = cf(nd(gen), nd(gen));
  });
  return t;
}

uint64_t next_rand(uint64_t &s)
{
  uint64_t z = (s += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

// A standard normal that is a pure function of (index, seed), so the shadowing is reproducible and
// both directions can evaluate it independently and still agree.
double hash_gauss(uint64_t k, uint64_t seed)
{
  uint64_t a = k * 2 + 1, b = k * 2 + 2;
  a ^= seed * 0x9e3779b97f4a7c15ULL;
  b ^= seed * 0x9e3779b97f4a7c15ULL;
  a = next_rand(a);
  b = next_rand(b);
  double u1 = std::max((a >> 11) * 0x1.0p-53, 1e-12);
  double u2 = (b >> 11) * 0x1.0p-53;
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2 * M_PI * u2);
}

std::string trim(const std::string &s)
{
  size_t b = s.find_first_not_of(" \t\r\n\"'");
  size_t e = s.find_last_not_of(" \t\r\n\"'");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

// A whole field as a finite number; no exceptions, as this also runs on the control thread
bool to_num(const std::string &field, double &v)
{
  const std::string s = trim(field);
  char *end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return !s.empty() && *end == '\0' && std::isfinite(v);
}

} // namespace

double config::distance_at(double t_sec) const
{
  if (speed_kmh == 0.0)
    return std::max(distance_m, 1.0);
  double d = distance_m + speed_kmh / 3.6 * t_sec;
  if (max_m > min_m) {
    // fold into [min_m, max_m]: a triangle wave, so the UE bounces between the two limits
    double span = max_m - min_m;
    double x = std::fmod(d - min_m, 2 * span);
    if (x < 0)
      x += 2 * span;
    d = min_m + (x <= span ? x : 2 * span - x);
  }
  return std::max(d, 1.0);
}

double config::loss_db_at(double t_sec) const
{
  return path_loss_exp * std::log10(distance_at(t_sec) / std::max(distance_m, 1.0));
}

double config::jitter_db_at(double t_sec) const
{
  if (snr_jitter_db <= 0)
    return 0;
  constexpr double PERIOD = 0.010; // a new shadowing value every 10 ms, interpolated between
  double x = t_sec / PERIOD;
  uint64_t k = (uint64_t)std::floor(std::max(x, 0.0));
  double f = x - (double)k;
  double g0 = hash_gauss(k, seed), g1 = hash_gauss(k + 1, seed);
  return snr_jitter_db * (g0 + (g1 - g0) * f);
}

std::string parse(const std::string &spec, config &out)
{
  config c;
  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (item.empty())
      continue;
    size_t eq = item.find('=');
    if (eq == std::string::npos)
      return "expected key=value, got '" + item + "'";
    std::string k = trim(item.substr(0, eq)), v = trim(item.substr(eq + 1));
    try {
      if (k == "distance")
        c.distance_m = std::stod(v);
      else if (k == "speed")
        c.speed_kmh = std::stod(v);
      else if (k == "min_distance")
        c.min_m = std::stod(v);
      else if (k == "max_distance")
        c.max_m = std::stod(v);
      else if (k == "dl_snr_ref")
        c.dl_snr_ref = std::stod(v);
      else if (k == "ul_snr_ref")
        c.ul_snr_ref = std::stod(v);
      else if (k == "ul_noise_dbfs")
        c.ul_noise_dbfs = std::stod(v);
      else if (k == "dl_snr_min")
        c.dl_snr_min = std::stod(v);
      else if (k == "ul_snr_min")
        c.ul_snr_min = std::stod(v);
      else if (k == "jitter_db")
        c.snr_jitter_db = std::stod(v);
      else if (k == "path_loss_exp")
        c.path_loss_exp = std::stod(v);
      else if (k == "seed")
        c.seed = std::stoull(v);
      else if (k == "trace_ctl")
        c.trace_ctl = v;
      else
        return "unknown option '" + k + "'";
    } catch (const std::exception &) {
      return "bad value for '" + k + "': '" + v + "'";
    }
  }
  if (c.distance_m <= 0)
    return "distance must be > 0";
  if (c.max_m > 0 && c.max_m <= c.min_m)
    return "max_distance must be greater than min_distance";
  if (c.path_loss_exp <= 0)
    return "path_loss_exp must be > 0";
  if (c.snr_jitter_db < 0)
    return "jitter_db must be >= 0";
  if (!c.trace_ctl.empty() && c.trace_ctl[0] != '/')
    return "trace_ctl must be an absolute path";
  if (!c.trace_ctl.empty() && std::isnan(c.ul_noise_dbfs))
    return "trace_ctl needs ul_noise_dbfs, or the UL of a trace could not be applied";
  out = c;
  return "";
}

std::string describe(const config &c)
{
  std::ostringstream o;
  o << "start " << c.distance_m << " m, " << c.speed_kmh << " km/h";
  if (c.max_m > c.min_m)
    o << ", bouncing " << c.min_m << "-" << c.max_m << " m";
  o << ", path loss " << c.path_loss_exp << " dB/decade, DL SNR " << c.dl_snr_ref << " dB at start (floor "
    << c.dl_snr_min << " dB)";
  if (c.snr_jitter_db > 0)
    o << ", shadowing " << c.snr_jitter_db << " dB rms";
  if (std::isnan(c.ul_noise_dbfs))
    o << ", UL untouched (no broker noise floor)";
  else
    o << ", UL SNR " << c.ul_snr_ref << " dB at start over " << c.ul_noise_dbfs << " dBFS";
  if (!c.trace_ctl.empty())
    o << "; traces switched through " << c.trace_ctl;
  return o.str();
}

std::string load_trace(std::istream &in, trace &out)
{
  trace tr;
  std::string line;
  if (!std::getline(in, line))
    return "empty trace";
  line = trim(line);
  const std::string head = "t_s,dl_snr_db,ul_snr_db";
  if (line.compare(0, head.size(), head) != 0 || (line.size() > head.size() && line[head.size()] != ','))
    return "header must start with '" + head + "', got '" + line + "'";
  for (size_t n = 2; std::getline(in, line); n++) {
    line = trim(line);
    if (line.empty())
      continue;
    std::stringstream ss(line);
    std::string f;
    double v[3];
    for (int i = 0; i < 3; i++) {
      if (!std::getline(ss, f, ','))
        return "line " + std::to_string(n) + ": expected 3 values";
      if (!to_num(f, v[i]))
        return "line " + std::to_string(n) + ": bad value '" + f + "'";
    }
    if (tr.t.empty() ? v[0] != 0 : v[0] <= tr.t.back())
      return "line " + std::to_string(n) + (tr.t.empty() ? ": t_s must start at 0" : ": t_s must increase");
    tr.t.push_back(v[0]);
    tr.dl.push_back(v[1]);
    tr.ul.push_back(v[2]);
  }
  if (tr.t.empty())
    return "no rows";
  const size_t m = tr.t.size();
  tr.period = m > 1 ? tr.t[m - 1] + (tr.t[m - 1] - tr.t[m - 2]) : 0;
  out = std::move(tr);
  return "";
}

void trace_at(const trace &tr, double tt, bool loop, double &dl_snr, double &ul_snr, size_t &row)
{
  if (loop && tr.period > 0) {
    tt = std::fmod(tt, tr.period);
    if (tt < 0)
      tt += tr.period;
  }
  const size_t i = std::upper_bound(tr.t.begin(), tr.t.end(), tt) - tr.t.begin();
  row = i > 0 ? i - 1 : 0;
  dl_snr = tr.dl[row];
  ul_snr = tr.ul[row];
}

std::string parse_ctl(const std::string &text, ctl_cmd &out)
{
  ctl_cmd c;
  bool have_trace = false, have_start = false;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    line = trim(line.substr(0, line.find('#')));
    if (line.empty())
      continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos)
      return "expected key=value, got '" + line + "'";
    const std::string k = trim(line.substr(0, eq)), v = trim(line.substr(eq + 1));
    if (k == "trace") {
      if (v != "none" && (v.empty() || v[0] != '/'))
        return "trace must be none or an absolute path";
      c.trace_path = v;
      have_trace = true;
    } else if (k == "start") {
      if (!to_num(v, c.start))
        return "bad start '" + v + "'";
      have_start = true;
    } else if (k == "loop") {
      if (v != "0" && v != "1")
        return "loop must be 0 or 1";
      c.loop = v == "1";
    } else {
      return "unknown key '" + k + "'";
    }
  }
  if (!have_trace || !have_start)
    return "needs trace= and start=";
  out = c;
  return "";
}

namespace {

// s and the switches still ahead at t_now; states already superseded by t_now are dropped, so the
// chain (and the traces it keeps alive) does not grow with every switch.
std::shared_ptr<const trace_state> pending_from(const std::shared_ptr<const trace_state> &s, double t_now)
{
  if (!s)
    return nullptr;
  auto c = std::make_shared<trace_state>(*s);
  c->prev = t_now >= s->T ? nullptr : pending_from(s->prev, t_now);
  return c;
}

} // namespace

void trace_source::publish(std::shared_ptr<const trace> tr, double T, bool loop, double t_now)
{
  auto s = std::make_shared<trace_state>();
  s->tr = std::move(tr);
  s->T = T;
  s->loop = loop;
  s->prev = pending_from(get(), t_now);
  std::atomic_store(&cur_, std::shared_ptr<const trace_state>(std::move(s)));
}

const trace_state *state_at(const trace_state *s, double t)
{
  while (s && t < s->T)
    s = s->prev.get();
  return s;
}

void power_probe::feed(const cf *x, size_t n)
{
  if (locked_ || n == 0)
    return;
  seen_ += n;
  // Blocks shorter than a few thousand samples can sit on an OFDM peak and read high (the waveform
  // has a high peak-to-average ratio), so only whole-slot-ish blocks set the reference.
  if (n >= 4096) {
    double sum = 0;
    for (size_t i = 0; i < n; i++)
      sum += std::norm(x[i]);
    peak_ = std::max(peak_, sum / n);
  }
  if (seen_ >= need_ && peak_ > 0) {
    power_ = peak_;
    locked_ = true;
  }
}

link::link(dir d, const config &cfg, double fs, uint64_t origin, std::shared_ptr<trace_source> traces)
    : d_(d),
      cfg_(cfg),
      fs_(fs),
      origin_(origin),
      traces_(std::move(traces)),
      probe_((uint64_t)(fs * 1.5)), // 1.5 s of samples, long enough to catch full-band SSB slots
      rng_(cfg.seed * 2 + (d == dir::ul) + 1)
{
  noise_table(); // build it now, not in the first block
}

std::string link::where() const
{
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(1);
  if (playing_)
    o << "trace " << playing_->name << " row " << st.trace_row.load() << " (T " << st.trace_T.load() << " s)";
  else
    o << "d " << st.distance_m.load() << " m";
  return o.str();
}

void link::process(cf *x, size_t n, uint64_t t0)
{
  if (n == 0)
    return;
  if (d_ == dir::ul && std::isnan(cfg_.ul_noise_dbfs))
    return; // attenuating the UL means nothing unless the gNB has a noise floor
  const double t = t0 > origin_ ? double(t0 - origin_) / fs_ : 0.0;
  st.t_air = t;

  // Calibrate on the clean signal, before anything is applied
  probe_.feed(x, n);
  if (!probe_.locked())
    return;
  st.calibrated = true;
  st.ref_dbfs = 10 * std::log10(probe_.power());

  // A trace if one is in force at t, else the model
  double dl_snr, ul_snr;
  const trace_state *s = nullptr;
  if (traces_) {
    state_ = traces_->get();
    s = state_at(state_.get(), t);
  }
  if (s && s->tr) {
    size_t row;
    trace_at(*s->tr, t - s->T, s->loop, dl_snr, ul_snr, row);
    const double j = cfg_.jitter_db_at(t); // 0 unless jitter_db is set
    dl_snr += j;
    ul_snr += j;
    playing_ = s->tr.get();
    st.trace_row = (long)row;
    st.trace_T = s->T;
  } else {
    const double loss = cfg_.loss_db_at(t) - cfg_.jitter_db_at(t);
    st.distance_m = cfg_.distance_at(t);
    dl_snr = std::max(cfg_.dl_snr_ref - loss, cfg_.dl_snr_min);
    ul_snr = std::max(cfg_.ul_snr_ref - loss, cfg_.ul_snr_min);
    playing_ = nullptr;
    st.trace_row = -1;
  }
  float *v = reinterpret_cast<float *>(x);

  if (d_ == dir::dl) {
    const double snr = dl_snr;
    st.dl_snr_db = snr;
    const float sigma = (float)std::sqrt(probe_.power() / std::pow(10.0, snr / 10.0));
    const auto &tab = noise_table();
    size_t off = next_rand(rng_) & (NOISE_TABLE - 1);
    size_t done = 0;
    while (done < n) {
      size_t len = std::min(n - done, NOISE_TABLE - off);
      const float *__restrict src = reinterpret_cast<const float *>(tab.data() + off);
      float *__restrict dst = v + 2 * done;
      for (size_t k = 0; k < 2 * len; k++)
        dst[k] += sigma * src[k];
      done += len;
      off = 0;
    }
  } else {
    // Scale against the one-time reference so the gNB sees ul_snr over its noise floor; the UE
    // applies no transmit power of its own to the samples, so the reference stays valid
    const double gain = ul_snr + cfg_.ul_noise_dbfs - 10 * std::log10(probe_.power());
    st.ul_snr_db = ul_snr;
    st.ul_gain_db = gain;
    const float g = (float)std::pow(10.0, gain / 20.0);
    for (size_t k = 0; k < 2 * n; k++)
      v[k] *= g;
  }
}

} // namespace chsim

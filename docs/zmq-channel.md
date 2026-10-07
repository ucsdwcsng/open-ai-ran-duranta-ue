# ZMQ mobility channel

A minimal channel model for the `zmq` radio driver: distance drives path loss, path loss drives SNR.
No fading, no delay spread, no Doppler.

```
d(t)   = starting distance + speed * t, bouncing between min_distance and max_distance
loss   = path_loss_exp * log10( d(t) / distance )          dB, relative to the start
DL     = white Gaussian noise added so the UE sees  dl_snr_ref - loss + shadowing
UL     = transmit samples attenuated by the same amount, against the gNB noise floor
```

Both SNRs are floored (`dl_snr_min`, `ul_snr_min`) so distance can never walk a UE into radio link
failure — this UE does not reconnect after an RRC release.

Instead of the model, a UE can also replay SNR traces and switch between them at run time
([Trace replay](#trace-replay)).

Implemented in `radio/zmq/chan_sim.{h,cpp}`, unit tests in `radio/zmq/tests/test_chan_sim.cpp`.
Hooks: DL in `rx_poll_thread` (`zmq_radio.cpp`), UL in `zmq_tx_channel::transmit`
(`zmq_imported.cpp`), the trace control thread in `zmq_radio.cpp`.

## Options

Passed as one comma-separated string: `--zmq.[0].chan "distance=50,speed=30,..."`

| key | default | meaning |
|---|---|---|
| `distance` | 100 | starting distance, m. `dl_snr_ref` is the SNR **here** |
| `speed` | 0 | km/h; 0 = stationary |
| `min_distance` / `max_distance` | 0 / 0 | bounce limits, m. `max <= min` disables bouncing (recedes forever) |
| `dl_snr_ref` | 25 | DL SNR in dB at `distance` |
| `ul_snr_ref` | 25 | UL SNR in dB at the gNB at `distance` |
| `ul_noise_dbfs` | unset | gNB noise floor the broker adds. **Unset leaves the UL untouched** |
| `dl_snr_min` / `ul_snr_min` | 8 / 5 | SNR floors, dB |
| `jitter_db` | 0 | random SNR offset (shadowing), rms dB: a new independent value every 10 ms, interpolated between, the same in DL and UL |
| `path_loss_exp` | 37.6 | dB per decade of distance |
| `seed` | 0 | seeds the shadowing and the DL noise; the same seed gives the same channel |
| `trace_ctl` | unset | control file for [trace replay](#trace-replay), an absolute path. Needs `ul_noise_dbfs` |

`distance` must be above 0 and, when bouncing, `max_distance` above `min_distance`, or the UE aborts
at startup with `[ZMQ] bad chan config: ...` in its log. A starting distance outside the bounce
range folds into it.

## Usage

The wrapper lives in the parent project. One line per UE in `configs/ue_mobility.conf`:

```
# ue   dist_m   speed_kmh   min_m   max_m   dl_snr_ref   ul_snr_ref   jitter_db   dl_min   ul_min
  1    50       30          40      140     25           25           5
  2    80       3           60      180     22           22           5           -5       -5
ul_noise_dbfs -65
```

Columns 8-10 are optional. `ul_noise_dbfs` is cell-wide and is passed to the broker automatically.
UEs with no line get a clean channel.

```bash
./scripts/start_gnb_zmq.sh                                        # terminal 1
./scripts/start_oai_ues.sh 2 --mobility configs/ue_mobility.conf  # terminal 2

sudo ip netns exec ue1 iperf3 -s >/dev/null 2>&1 &                # terminal 3
sudo ip netns exec ue2 iperf3 -s >/dev/null 2>&1 &
sudo iperf3 -c 10.45.0.7 -b 0M -t 600 -i 1 &                      # UE1
sudo iperf3 -c 10.45.0.8 -b 0M -t 600 -i 1 &                      # UE2
```

Watch the channel (once per second of air time per UE):

```bash
grep 'ZMQ chan' /tmp/oai_ues/ue1.log | tail -5
#   [ZMQ chan] t 125.0 s, d 125.0 m, DL SNR 8.4 dB (ref -27.2 dBFS), UL SNR 5.0 dB, UL gain -7.1 dB
```

`t` is the air time (see [Trace replay](#trace-replay)); the model runs on it too.

**Restart the gNB between runs.** Stopping the UEs leaves the gNB's UL socket waiting on a reply
that never comes, which wedges its radio loop; the next UEs then fail to sync with no error.

**The first ~1.5 s is calibration.** The log says `calibrating DL reference level` and no noise is
applied until the reference is measured. Ignore metrics from that window.

## Trace replay

Instead of the model, a UE can replay SNR traces (DL and UL SNR over time), and switch to another
trace at run time. Add `trace_ctl` to the spec, with `ul_noise_dbfs`, which it requires:

```
--zmq.[0].chan "dl_snr_ref=30,ul_snr_ref=30,seed=1,ul_noise_dbfs=-65,trace_ctl=/tmp/traces/ue1.ctl"
```

The model still runs as the **base channel**: it plays until a trace starts and again after
`trace=none`.

### Trace files

CSV, one row per step:

```
t_s,dl_snr_db,ul_snr_db,d_m
0,25.27,25.27,50.0
0.01,25.14,25.14,50.0
```

- The header starts with `t_s,dl_snr_db,ul_snr_db`; further columns are ignored.
- `t_s` starts at 0 and strictly increases. Row i holds from `t[i]` until `t[i+1]` (no
  interpolation). A trace loops with a period of its last `t` plus its last step; a one-row trace
  is a constant.
- The SNRs apply as given: no floors. `jitter_db`, if set, adds on top.

### The control file

One per UE. A normal-priority thread polls it every 100 ms; file reading never runs on the radio
threads. Replace the file whole (write a temp file, then `mv`) so the UE never reads half of one:

```
trace=/tmp/traces/trace_2.csv      # or trace=none: back to the base channel
start=1234.5                       # air time, s, at which it takes over
loop=1                             # 0: play once, then hold the last row
```

- When it changes, the UE loads the trace and logs
  `[ZMQ chan] trace trace_2 (30000 rows, looping) from t 1234.5 s (set at t 1231.6 s)`.
- The current channel plays until `start`. The new one then takes over on the same sample in both
  directions, with no gap, starting from its row 0.
- A `start` already past plays in phase with it (`set 1.2 s late` in the log).
- A bad control or trace file is logged with `channel unchanged`, and the UE carries on.
- With no control file the UE plays the base channel. Delete stale control files before starting a
  UE, or it replays the last one.

### Air time

Air time is samples since the stream origin, over the sample rate. The DL and the UL run on it, and
so does every UE behind a broker that starts all UEs on the same gNB sample (the C++ broker does).
So one `start` written to several control files switches those UEs at the same instant, and UEs on
the same trace see the same SNRs at the same time. The status line shows it first:

```
[ZMQ chan] t 1236.0 s, trace trace_2 row 150 (T 1234.5 s), DL SNR 21.3 dB (ref -27.3 dBFS), UL SNR 21.3 dB, UL gain 3.7 dB
```

### Limits

- Keep both SNRs at 0 dB or above. This UE turns its measured SINR into CQI through an unsigned
  conversion (`nr_csi_rs_cqi_estimation` in `csi_rx.c`): a negative SINR reports CQI 15, and 0 dB
  reports CQI 0. A UE the gNB then releases does not come back.
- The gNB adapts the DL from CSI every 20 ms (`csi_rs_period`). Faster changes show up as block
  errors, not as CQI.
- The UL SNR is against the broker's noise floor, so the broker must add `ul_noise_dbfs`, and any
  broker UL gain shifts every UL value.

## Measured CQI vs SNR

20 MHz n78 TDD cell, static UE, `jitter_db 0`, `olla_max_cqi_offset 10`, `olla_target_bler 0.03`:

| DL SNR | CQI | MCS | BLER | Mbps | link |
|---|---|---|---|---|---|
| 30 dB | 15 | 27.0 | 0.0% | 65.7 | up |
| 22 dB | 15 | 26.0 | 2.4% | 63.1 | up |
| 16 dB | 13 | 19.0 | 3.1% | 44.4 | up |
| 12 dB | 12 | 12.5 | 3.1% | 27.8 | up |
| 8 dB | 9 | 7.8 | 2.9% | 18.0 | up |
| 2 dB | 6 | 3.1 | 2.9% | 8.3 | up |
| 0 dB | 4 | 1.6 | 2.8% | 4.6 | up |
| -3 dB | 4 | 1.4 | 38.6% | 0.2 | up |

CQI saturates at 15 above ~22 dB. Below ~0 dB the MCS bottoms out, so BLER climbs and throughput
collapses while the link still holds. For a CQI 4-15 swing you need 22 dB of loss, a distance ratio
of 4.4 (e.g. 40 -> 175 m) and `dl_min` lowered below the 8 dB default.

## gNB-side settings that matter

The scheduler, not the channel, decides MCS in **both** directions, from the UE's CQI report (DL)
and its own PUSCH SNR measurement (UL).

`olla_target_bler` only has an effect if `olla_max_cqi_offset` is large enough. The offset is hard
clamped at +/- its maximum; once OLLA saturates there, the target BLER is unreachable and changing
it does nothing. With OCUDU's default 4.0 this channel sits at 21-29% BLER whatever the target.

| `olla_max_cqi_offset` | `olla_target_bler` | avg MCS | BLER | throughput |
|---|---|---|---|---|
| 4.0 (default) | 0.03 | 13.9 | 21-29% | 12.1 Mbps |
| 10.0 | 0.03 | 6.9 | 3.1% | 8.3 Mbps |
| 15.0 | 0.01 | 5.1 | 1.3% | 6.4 Mbps |

Set in the gNB yaml under `cell_cfg`, or on the command line:

```bash
./scripts/start_gnb_zmq.sh configs/gnb_zmq_tdd_n78_20mhz.yml -- \
    cell_cfg pdsch --olla_max_cqi_offset 10 --olla_target_bler 0.03
```

The UL equivalent is `pusch.olla_max_snr_offset` (default 5.0), in **dB of SNR offset** rather than
CQI steps.

CSI-RS period is 20 ms by default (`cell_cfg csi --csi_rs_period`, values 10/20/40/80 ms; 10 ms is
rejected above 52 PRB). That is the feedback rate link adaptation runs on — ample for smooth path
loss, but it cannot follow `jitter_db` shadowing, which is what makes the block errors.

## Known limits

- Single antenna (1x1) only.
- The UL is scaled against a one-time measurement of the UE's transmit level. This UE writes its
  samples at a fixed amplitude (3GPP power control does not scale them), so that reference holds and
  the UL SNR at the broker's noise floor is what `ul_snr_ref` and the model (or a trace) ask for.
- The gNB's reported `pusch_snr` reads a few dB below `ul_snr_ref` (~2.7 dB on this setup): it
  averages instantaneous dB values over a narrow, bursty PUSCH, and the UE's PUCCH lands ~4 dB above
  its PUSCH, so one UE-side anchor cannot match both.

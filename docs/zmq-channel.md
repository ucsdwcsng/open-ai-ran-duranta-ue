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

Implemented in `radio/zmq/chan_sim.{h,cpp}`, ~230 lines. Hooks: DL in `rx_poll_thread`
(`zmq_radio.cpp`), UL in `zmq_tx_channel::transmit` (`zmq_imported.cpp`).

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
| `jitter_db` | 0 | slow random SNR offset (shadowing), rms dB, new value every 10 ms |
| `path_loss_exp` | 37.6 | dB per decade of distance |
| `seed` | 0 | 0 = random |

`distance` must be inside `[min_distance, max_distance]`, and `max_distance > min_distance`, or the
UE aborts at startup with `[ZMQ] bad chan config: ...` in its log.

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

Watch the channel (once per second per UE):

```bash
grep 'ZMQ chan' /tmp/oai_ues/ue1.log | tail -5
#   [ZMQ chan] d 125 m, DL SNR 8.4 dB (ref -27.2 dBFS), UL gain -7.1 dB
```

**Restart the gNB between runs.** Stopping the UEs leaves the gNB's UL socket waiting on a reply
that never comes, which wedges its radio loop; the next UEs then fail to sync with no error.

**The first ~1.5 s is calibration.** The log says `calibrating DL reference level` and no noise is
applied until the reference is measured. Ignore metrics from that window.

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
- `ul_snr_ref` is anchored by a one-time measurement of the UE's own transmit power, then path loss
  is applied. The UE keeps running 3GPP power control underneath, so the level the gNB actually
  receives drifts from the target. `dl_snr_ref` is exact; treat `ul_snr_ref` as a starting point.
- The gNB's reported `pusch_snr` reads a few dB below `ul_snr_ref` (~2.7 dB on this setup): it
  averages instantaneous dB values over a narrow, bursty PUSCH, and the UE's PUCCH lands ~4 dB above
  its PUSCH, so one UE-side anchor cannot match both.

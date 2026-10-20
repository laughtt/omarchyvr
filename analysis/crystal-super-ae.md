# Crystal Super: what Pimax Play's camera auto-exposure sends (capture 2026-10-17)

Source: the 14 decoded takes of `crystal-super-capture-20261017.md` (cold start x2, idle, IPD, settings, controllers,
sleep/wake, shutdown, eye calibration, lighthouse x3, SLAM x2), every IF0 OUT report `58 19 51 00 ..` to the headset's
HID chip 34a4:0040 and every reply. Raw files stay off the repo (serials). The camera stream (isochronous) is not in
the decode, so latency and image brightness are not measured here; the Dream Air's are in `slam-exposure.md`.
M = measured, I = inferred.

## Frame and timing

| Item | Value |
|---|---|
| Frame | `58 19 51 00 01 32 e0 e1 e2 e3 g0 g1 g2 g3 led_e led_g` (u16 LE), the Dream Air's layout [M] |
| Commands | 16751 in the 14 takes, each answered `4c 04 51 00 01 07` (16751 replies) [M] |
| Rate | median interval 80.0 ms (p10 79.9, p90 80.1) in every take: 12.5 Hz, as on the Dream Air [M] |
| Head | `01 32` in every command (0x32 = 50: the frame-sync request [I]; on the Dream Air the first push turns the 60 fps stream into 50 fps, probably the same here) [M/I] |
| LED pair | 40 / 62 in every command [M] |
| Pairs | e0 = e1, e2 = e3, g0 = g1, g2 = g3 in every command: lower pair (cameras 0, 1) and upper pair (2, 3) [M] |
| Changes | 90.5 % of the commands repeat the previous setting (per pair); a change is usually one 3 % step (63 %), 2 steps 16 %, 3 steps 10 %, up to ~12 steps while ramping after a start [M] |
| Start | the first command of a cold start is 2200 / 13 on all four cameras, held 2.2-2.4 s (takes cold1, cold2), then a ramp of large steps (~1.5 s) to the room's setting; 2200 / 13 also for 0.16 s late in takes lighthouse and lighthouse2 (a camera restart when the mode changes [I]) [M] |
| Firmware default before the first push | 3500 / 60 (frame metadata on Linux, 2026-10-18) [M] |

## Ranges (min..max over all takes)

| | Lower pair (0, 1) | Upper pair (2, 3) | Dream Air (Windows) |
|---|---|---|---|
| Exposure (us [I]) | 2200..10410, median 6739 | 2200..8941, median 4584 | 1140..4167 |
| Gain code | 13..449, median 288 | 13..439, median 215 | 13..215 |

The Crystal runs far brighter settings than the Dream Air did: most of the session it sits above the Dream Air's
ceiling (4108 / 215). The 2200 floor is the start value; no take was in a room bright enough to need less, so the
floor below 2200 is unknown and not used.

## How exposure and gain are split (one 1-D ladder)

98 distinct exposures and 56 gain codes; every (exposure, gain) pair falls on one ascending ladder of brightness:

1. **Gain 13, exposure up**: 2200, 2875, 2962, 3758, 3988 seen, all on the Dream Air's 3 % exposure ladder
   (ratio 1.0302) [M].
2. **Gain up, exposure ~3.6k..4.17k**: gain codes 13, 17, 19, 21, 23, 29, 32, 35, 43, 46, 49, 53, 58, 65, 71, 78, 79,
   87, 95, 105, 116, 128, 141, 155, 171, 188, 207, 215, the exposure retuned just under ~4.1k at each code (the Dream
   Air's gain stage; 49, 58, 65 are new, 15, 26, 39, 59 not seen) [M].
3. **Gain 215, exposure up**: 3615, 3724, 3836, 3952, 4071, 4194, 4320, 4450, 4584, 4722, 4864, 5051, 5161, 5316,
   5476, 5641 (3 % steps) [M].
4. **Exposure and gain together**, one fixed gain per exposure: 5720/220, 5811/226, 5986/237, 6072/244, 6166/249,
   6351/261, 6440/270, 6542/275, 6632/280, 6739/288, 6942/303, 7042/309, 7151/318, 7251/326, 7366/334, 7462/334,
   7587/351, 7662/358, 7815/368, 7880/370, 7980/375, 8050/387, 8292/406, 8342/421, 8541/426, 8741/431, 8941/439,
   9541/439, 9841/446, 10410/449. Brightness steps of 1.3..8.4 % [M]. The top is the Dream Air's power-on 10410/449.

## What the Linux controller uses (`rust/pimax-cam/src/ae.rs`, `AeModel::CrystalSuper`)

- The same control law, target, metering, 80 ms period and rate limit as the Dream Air; only the constants differ.
- Ladder at the low gains: the Dream Air's ladder from 2200 to the soft cap 4108 (`CRYSTAL_EXPOSURE_LADDER`); gain
  codes up to 215 as Windows sent them on the Crystal (`CRYSTAL_GAIN_CODES`); above the soft cap the stage 3 and
  stage 4 settings above (`CRYSTAL_HIGH`, 41 settings from 4194/215 to 10410/449), the nearest one in brightness.
- Limits = Windows' ranges per pair (`CRYSTAL_PAIR_RANGES`): lower 2200..10410 / 13..449, upper 2200..8941 / 13..439,
  enforced on every setting (`AeModel::allows`, and the C ABI does not send a setting outside them). No automatic
  extended range (the normal range already reaches 10410/449) and no low exposure stage below 2200.
- Model: `PIMAX_CAM_AE_MODEL=dream-air|crystal-super`, otherwise the HID chip on USB (34a4:0040 = Crystal Super).
  Start-up log line `PIMAX_AE_MODEL: crystal-super (USB 34a4:0040); Windows ranges enforced: ...`.
- The `camera_ae` allowlist entry of `rust/pimax-device/devices/crystal-super/device.toml` has the same ranges
  (approved by the user 2026-10-19; sent only with `PIMAX_CAM_AE=1`).
- Not measured: the image brightness Windows aimed at on the Crystal (target 64 kept), the latency (the Dream Air's
  2 tracking frames assumed), the fps after the first push (likely 60 -> 50 as on the Dream Air; the tracking frames
  then come at 25 Hz instead of 30 Hz, the period and rate limit are in time, not frames).

# Pimax Crystal Super on Linux (experimental)

Status 2026-10-18, one headset (Crystal Super Micro-OLED, Lighthouse faceplate), RTX 5090, Ubuntu 26.04 (KDE Wayland),
NVIDIA 595.99.02 open modules. Findings and captures: [../analysis/crystal-super-capture-20261017.md](../analysis/crystal-super-capture-20261017.md).

The Crystal Super speaks the Dream Air's USB protocol (`58 ..` frames on IF0, the same IMU and controller reports,
the same cameras `34a4:0014`, the same Tobii eye tracker), so the omarchyvr stack drives it: keepalive, video on, 3DoF,
the Pimax controllers (optical tracking works), the lens (a LUT from the headset's own Pimax distortion dump).
Inside-out head tracking (SLAM) is not stable yet. With the Lighthouse faceplate the light setup below works today.

## The light setup: SteamVR Lighthouse + CustomHeadsetOpenVR + omarchyvr

| Part | Who |
|---|---|
| Head and Index controller tracking | SteamVR's own lighthouse driver |
| The headset display in SteamVR (EDID vendor, render size, distortion, IPD) | [CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) (sboys3) with `linux/crystal-super/chovr-linux-direct-usb.patch` |
| Wake, keepalive, panel/video on, headset sound | `omarchyvr keepalived --video 90 --follow vrserver --audio` (user service) |
| Eye tracking (`XR_EXT_eye_gaze_interaction` for OpenXR apps) | `omarchyvr eyed` (user service) -> shared memory -> CustomHeadsetOpenVR with `linux/crystal-super/chovr-linux-eye-gaze.patch` |
| 7104x3840 at 90 Hz over DSC | the NVIDIA patch `patches/nvidia/blackwell/` and the EDID override |
| Sound | USB audio of the headset (WirePlumber rule) |

Monado is not used in this mode. Start SteamVR from Steam as usual.

### Install (one time)

1. Access to the headset: the udev rule `linux/70-pimax.rules` (its first three lines are enough without the hub
   power helper), in `/etc/udev/rules.d/`.
2. The keepalive service: `~/.local/bin/omarchyvr` (from `rust/`), `linux/omarchyvr-keepalived.service` into
   `~/.config/systemd/user/`, plus the drop-in `linux/crystal-super/omarchyvr-keepalived-crystal-lighthouse.conf` as
   `~/.config/systemd/user/omarchyvr-keepalived.service.d/crystal-lighthouse.conf`; `systemctl --user enable --now
   omarchyvr-keepalived`. It follows SteamVR (`--follow vrserver`): while `vrserver` runs it wakes the headset, turns
   the panel on and moves the default sound output and input to the headset (`--audio`); 5 s after SteamVR closes it
   gives the sound back and sends one keepalive with rate 0 and then nothing, so the headset goes to its standby
   (green LED), as when Pimax Play closes on Windows.
3. The NVIDIA modules (`patches/nvidia/blackwell/README.md`): build the open modules of your driver version with the
   patch, create a MOK key and enroll it (`mokutil --import`, use a digits-only password: MokManager runs with a US
   keyboard layout), sign the five modules with `scripts/sign-file` of the kernel headers.
4. The EDID: `linux/crystal-super/make_crystal_edid.py /sys/class/drm/card*-DP-N/edid crystal-edid.bin` with the
   headset awake (the DRM connector whose EDID is `PVR`).
5. `sudo linux/crystal-super/install-nvidia.sh WORKDIR` (WORKDIR = `modules/`, `mok/MOK.der`, `edid/crystal-edid.bin`),
   reboot. Undo: `sudo linux/crystal-super/uninstall-nvidia.sh` (from a rescue boot: give the kernel version).
6. CustomHeadsetOpenVR: build it on Linux (its README), apply the patch, register it with
   `SteamVR/bin/vrpathreg.sh adddriver <build>/CustomHeadsetOpenVR`, and enable the headset in
   `~/.config/CustomHeadset/settings.json`: `{"crystalSuperMicroOLED": {"enable": true, "distortionProfile":
   "Crystal Super micro-OLED like Pimax"}}` (or with its GUI, `CustomHeadsetGUI`, a Tauri app that builds on Linux).
   The "like Pimax" profile (fitted to the Pimax compositor's mesh, optical centre off the panel centre) matched the
   lens best in the test; the driver's default profile is "Crystal Super Micro-OLED Default".
7. Sound: `linux/crystal-super/51-pimax-crystal-super.conf` into `~/.config/wireplumber/wireplumber.conf.d/`, restart
   WirePlumber. The speakers are ALSA device 1 of the card; the stock analog profiles only look at device 0.
8. Switch the Lighthouse faceplate on (its switch). The Valve devices (`28de:2300` LHR, `28de:2101` dongles) appear
   on their own.
9. Eye tracking (optional), see below.

### Eye tracking

The Tobii chip is started with the bytes of a Windows capture of this headset (Pimax Play's own start), with the
user's current Tobii calibration profile in place of the one in the capture. The chip then does the calibration and
the compensation of the headset's fit itself, as under Windows (findings:
[../analysis/crystal-super-eye-20261018.md](../analysis/crystal-super-eye-20261018.md)).

1. Under Windows, with Pimax Play: calibrate the eye tracking, capture one cold start of the client
   (`windows/usbpcap-capture.cmd`, then close and start Pimax Play) and copy the Tobii files with
   `windows/crystal-eye-files.ps1`: the profile is `ProgramData/Tobii/Tobii Platform Runtime/XR5EYECHIP/<headset>/calibration.setpm`.
2. The start list: `pimax-eye frames <capture.pcap> -o ~/.config/omarchyvr/eye-start-crystal-super.txt`. The profile:
   `pimax-eye profile calibration.setpm -o ~/.config/omarchyvr/eye-profile-crystal-super.bin`. Both files are private
   (they hold this headset's eye profile): never share or commit them. After a new calibration under Windows only the
   profile has to be taken again.
3. `linux/omarchyvr-eyed.service` into `~/.config/systemd/user/`, `systemctl --user enable --now omarchyvr-eyed`. It
   runs the eye tracker while `vrserver` runs (`--follow vrserver`) and publishes the gaze in
   `/dev/shm/omarchyvr-eye-gaze`; when the headset is put back on it switches the eye module on again.
4. CustomHeadsetOpenVR with `linux/crystal-super/chovr-linux-eye-gaze.patch` (on top of the direct-USB patch) reads
   that file and serves SteamVR's eye tracking: OpenXR apps get `XR_EXT_eye_gaze_interaction`.
5. A constant offset remains between the chip's frame and SteamVR's view (seen: 9 and 14 degrees, mostly vertical,
   different per eye). Remove it once with `pimax-eyecal --recenter --pattern pimax --frames
   ~/.config/omarchyvr/eye-start-crystal-super.txt` (with `PIMAX_EYE_PROFILE` set to the profile, `omarchyvr-eyed`
   stopped) over `~/.config/omarchyvr/eye-calibration.toml` with an identity map: 5 targets, A to start, B to save.
   A full host calibration (`pimax-eyecal --pattern pimax`) is only needed without the user's profile.
6. Check: `analysis/tools/xr_grid.c` shows the gaze from the file (cyan) and through OpenXR (magenta).

Quad views and eye-tracked foveated rendering for Vulkan games (Falcon BMS) on top of this: the separate layer
[linux-vulkan-quadviews-openxr-layer](https://github.com/artscout/linux-vulkan-quadviews-openxr-layer).

### Known issues

- A small screen-fixed shimmer ("water running down glass") near the centre of both eyes, seen only in SteamVR Home
  (not in other apps' menus): an effect of that scene, not of the stack. Checked and ruled out: DSC (a slice height of
  240 instead of 1920 did not change it) and CustomHeadsetOpenVR's RadialBezier profile (smooth there when probed
  offline: slope ripple below 0.1 %, no fold; SteamVR samples it on a 127 mesh). Opening and closing the SteamVR
  dashboard (the controller's system button) makes it go away; it sometimes comes back, and the dashboard clears it
  again (seen 2026-10-18): a state of SteamVR's own rendering.
- About 4 % dropped frames when the head moves fast; SteamVR's asynchronous reprojection is off on this setup.
- Monitors blink once when SteamVR takes or gives back the headset's output.
- vrcompositor aborted once ("Vulkan transfer ring buffer overflow") when Falcon BMS left 3D for its menu; not
  seen in the other runs that did the same (2026-10-18).
- Inside-out 6DoF (Basalt on the headset cameras) diverges; work in progress.

## Without the faceplate: the omarchyvr stack

`omarchyvr keepalived`, Monado (`forks/monado` with `linux/crystal-super/monado-crystal-super.patch`, which lets the
Pimax driver take the Crystal Super's USB id) with
`PIMAX_LENS_CONF` pointing at a `lens.conf` with `lut = <pimax-lens make-lut of the headset's PVR dump>`, the camera
broker and `pimax-ctrltrack live --factory` with `PIMAX_CAM_CALIB` = the headset's factory calibration
(`slam/device_calibration_<serial>.xml` from a Pimax Play install). 3DoF, the picture and the Pimax controllers work;
6DoF of the head does not yet.

### Camera auto exposure (experimental, off by default)

The tracking cameras start at the Crystal's firmware default (3500 us, gain 60). Pimax Play on Windows pushes exposure
and gain every 80 ms with the same HID frame as on the Dream Air, but with the Crystal's own, much brighter ranges:
lower pair 2200..10410 us / gain 13..449, upper pair 2200..8941 us / gain 13..439 (`analysis/crystal-super-ae.md`).
The Linux controller (`pimax-cam`, the same law as the Dream Air) uses exactly those values for the Crystal, and the
headset's allowlist (`devices/crystal-super/device.toml`, `camera_ae`) refuses anything outside them.

- Turn it on: `PIMAX_CAM_AE=1` in the Monado / SteamVR driver environment (in the 6DoF setup the `SETTINGS` of
  `crystal-6dof.sh`; for a recording `rec_env=PIMAX_CAM_AE=1`). `PIMAX_CAM_AE=dry` only computes and logs.
- The model is taken from the USB id (34a4:0040); `PIMAX_CAM_AE_MODEL=crystal-super` forces it. The Monado log shows
  `PIMAX_AE_MODEL: crystal-super (USB 34a4:0040); Windows ranges enforced: ...` once at start.
- Expect one short camera gap at the first push (the cameras restart; probably 60 -> 50 fps as on the Dream Air).
- Needs a Monado relinked against this pimax-cam (the `PIMAX_AE_MODEL` marker is then in `monado-service`).

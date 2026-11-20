//! C ABI of [`crate::ae`] (declared in `include/pimax_cam.h`, section "Auto-exposure").
//!
//! The driver calls [`pimax_cam_ae_update`] after every tracking frame; when it returns 1 the driver may send
//! `payload` as HID command `0x51` option 0 to the headset. Whether it really sends is the driver's decision
//! (`PIMAX_CAM_AE=1`; `dry` = compute and log only). Nothing here touches the device.

use crate::ae::{
    payload, AeModel, AeParams, AeSetting, AutoExposure, LightEvent, Mask, TileView, AE_PAYLOAD_LEN,
};
use crate::camera::{FLAG_META_VALID, FLAG_REPLAY};
use crate::capi::{guard, PimaxCam};
use crate::frame::FrameKind;
use std::ffi::{c_char, c_int, CStr};

/// Opaque controller handle.
pub struct CAe {
    ae: AutoExposure,
    mask: Mask,
    refused: u64,
}

/// `pimax_cam_ae_out`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CAeOut {
    /// 1 = send `payload` now (cmd 0x51, opt 0).
    pub send: u32,
    /// 1 = the target changed in this update.
    pub changed: u32,
    /// Counters: frames analysed, sends handed out, target changes.
    pub frames: u64,
    /// Sends.
    pub sends: u64,
    /// Changes.
    pub changes: u64,
    /// Command payload (after `58 19 51 00`).
    pub payload: [u8; AE_PAYLOAD_LEN],
    /// Padding, zero.
    pub pad: [u8; 2],
    /// Target exposure of cameras 0..3.
    pub exposure: [u16; 4],
    /// Target gain of cameras 0..3.
    pub gain: [u16; 4],
    /// LED pair (unchanged).
    pub led_exposure: u16,
    /// LED gain.
    pub led_gain: u16,
    /// Exposure/gain the analysed frame was taken with (metadata), cameras 0..3.
    pub frame_exposure: [u16; 4],
    /// Frame gain.
    pub frame_gain: [u16; 4],
    /// Trimmed mean of the lower and the upper pair (0..255).
    pub mean: [f32; 2],
    /// Saturated fraction per pair.
    pub sat: [f32; 2],
    /// Log error per pair (0 inside the deadband).
    pub err: [f32; 2],
    /// Low-light state: bits 0..1 = range stage (0 normal, 1 gain extended, 2 gain and exposure extended), bit 2 =
    /// "tracking light low" warning raised.
    pub light: u32,
}

fn create(model: AeModel, why: &str) -> *mut CAe {
    let ae = AutoExposure::new(AeParams::from_env_model(model));
    // Once-at-start markers (also in the binary: the build check greps for them).
    eprintln!("{}", model_line(&ae, why));
    eprintln!("{}", start_line(&ae));
    Box::into_raw(Box::new(CAe {
        ae,
        mask: Mask::default(),
        refused: 0,
    }))
}

/// Create a controller with the defaults of the headset on USB (`PIMAX_CAM_AE_MODEL` overrides; Dream Air when
/// none is found) and the `PIMAX_CAM_AE_*` environment overrides.
#[no_mangle]
pub extern "C" fn pimax_cam_ae_create() -> *mut CAe {
    guard(std::ptr::null_mut(), || {
        let (m, why) = AeModel::detect();
        Ok(create(m, &why))
    })
}

/// [`pimax_cam_ae_create`] for a named headset model (`"dream-air"`, `"crystal-super"`; NULL or "" = detect).
/// `PIMAX_CAM_AE_MODEL` still overrides. NULL (and `pimax_cam_last_error`) for an unknown name.
///
/// # Safety
/// `model` NULL or a NUL-terminated string.
#[no_mangle]
pub unsafe extern "C" fn pimax_cam_ae_create_model(model: *const c_char) -> *mut CAe {
    guard(std::ptr::null_mut(), || {
        let name = if model.is_null() {
            String::new()
        } else {
            // SAFETY: per contract.
            unsafe { CStr::from_ptr(model) }.to_string_lossy().into_owned()
        };
        let env = std::env::var("PIMAX_CAM_AE_MODEL").ok().and_then(|v| AeModel::parse(&v));
        if name.trim().is_empty() || env.is_some() {
            let (m, why) = AeModel::detect();
            return Ok(create(m, &why));
        }
        let m = AeModel::parse(&name).ok_or_else(|| format!("unknown headset model {name:?} (dream-air or crystal-super)"))?;
        Ok(create(m, "the driver"))
    })
}

/// Destroy (NULL ok).
///
/// # Safety
/// `ae` from `pimax_cam_ae_create`, not used afterwards.
#[no_mangle]
pub unsafe extern "C" fn pimax_cam_ae_destroy(ae: *mut CAe) {
    if !ae.is_null() {
        // SAFETY: created by Box::into_raw.
        drop(unsafe { Box::from_raw(ae) });
    }
}

/// Analyse the current frame of `cam` (call right after `pimax_cam_next` returned a frame). Returns 1 = send
/// `out->payload` now, 0 = nothing to send (LED frame, replay, no metadata, or not due), -1 = error.
///
/// # Safety
/// `ae` and `cam` valid; `out` writable.
#[no_mangle]
pub unsafe extern "C" fn pimax_cam_ae_update(
    ae: *mut CAe,
    cam: *const PimaxCam,
    now_ns: u64,
    out: *mut CAeOut,
) -> c_int {
    guard(-1, || {
        // SAFETY: per contract.
        let a = unsafe { ae.as_mut() }.ok_or("null controller")?;
        // SAFETY: per contract.
        let c = unsafe { cam.as_ref() }.ok_or("null camera")?;
        if out.is_null() {
            return Err("null output".into());
        }
        let fi = *c.cam.info();
        // Only live tracking frames with metadata drive the loop (a replay cannot be controlled).
        if fi.kind != FrameKind::Tracking as u32
            || fi.flags & FLAG_META_VALID == 0
            || fi.flags & FLAG_REPLAY != 0
        {
            return Ok(0);
        }
        let mut views: Vec<TileView> = Vec::with_capacity(4);
        for t in 0..4 {
            views.push(c.cam.tile_view(t).map_err(|e| e.to_string())?);
        }
        let tiles: [TileView; 4] = [views[0], views[1], views[2], views[3]];
        let setting = AeSetting::from_info(&fi, a.ae.p.led_exposure, a.ae.p.led_gain);
        let stats = a.ae.frame_stats(&tiles, &a.mask);
        let d = a.ae.update(&setting, stats, now_ns);
        for ev in a.ae.take_events() {
            report(ev, &a.ae);
        }
        let tgt = a.ae.target().unwrap_or(setting);
        // Last guard: a model with Windows ranges (Crystal Super) never hands out a setting outside them.
        let allowed = a.ae.p.model.allows(&tgt);
        if !allowed {
            a.refused += 1;
            if a.refused == 1 || a.refused % 1000 == 0 {
                eprintln!(
                    "PIMAX_AE_MODEL: {} setting {:?}/{:?} outside the Windows ranges, not sent ({} so far)",
                    a.ae.p.model.id(),
                    tgt.exposure,
                    tgt.gain,
                    a.refused
                );
            }
        }
        let send = d.send && allowed;
        let o = CAeOut {
            send: send as u32,
            changed: d.changed as u32,
            frames: a.ae.frames,
            sends: a.ae.sends,
            changes: a.ae.changes,
            payload: payload(&tgt),
            pad: [0; 2],
            exposure: tgt.exposure,
            gain: tgt.gain,
            led_exposure: tgt.led_exposure,
            led_gain: tgt.led_gain,
            frame_exposure: fi.exposure,
            frame_gain: fi.gain,
            mean: [
                d.stats[0].trimmed_mean as f32,
                d.stats[1].trimmed_mean as f32,
            ],
            sat: [d.stats[0].sat_frac as f32, d.stats[1].sat_frac as f32],
            err: [d.err[0] as f32, d.err[1] as f32],
            light: a.ae.light.stage as u32 | (a.ae.light.low as u32) << 2,
        };
        // The head tracker lowers its fusion weights in a dark image (crate::headpose, PIMAX_FUSE_AUTO).
        crate::headpose::scene::publish(o.mean[0].min(o.mean[1]) as f64);
        // SAFETY: writable per contract.
        unsafe { *out = o };
        Ok(send as c_int)
    })
}

/// The headset text of the low-light warning (also shown on the app's status page).
pub const LOW_LIGHT_TEXT: &str =
    "Tracking cameras see too little infrared light \u{2014} daylight or an IR lamp helps";

/// Start-up marker line of the headset model (`PIMAX_AE_MODEL`): which constants run and where the model came from.
pub fn model_line(ae: &AutoExposure, why: &str) -> String {
    let p = &ae.p;
    let ranges = match p.model.pair_ranges() {
        Some(r) => format!(
            "Windows ranges enforced: lower pair {}..{} us / gain {}..{}, upper pair {}..{} us / gain {}..{}",
            r[0].0, r[0].1, r[0].2, r[0].3, r[1].0, r[1].1, r[1].2, r[1].3
        ),
        None => format!("exposure {}..{} us, gain {}..{}", p.exposure_min, p.exposure_max, p.gain_min, p.gain_max),
    };
    format!("PIMAX_AE_MODEL: {} ({why}); {ranges}", p.model.id())
}

/// Start-up marker line (`PIMAX_AE_EXTENDED`).
pub fn start_line(ae: &AutoExposure) -> String {
    if ae.top_stage() == 0 {
        return "PIMAX_AE_EXTENDED: off (extended range disabled or the normal caps are already the largest)".into();
    }
    let (e, g) = ae.caps(0);
    let (xe, xg) = ae.caps(2);
    format!(
        "PIMAX_AE_EXTENDED: armed, normal {e} us / gain {g}, extended up to {xe} us / gain {xg} (gain first, exposure last)"
    )
}

/// One log line per governor event (the app reads these from the Monado log).
pub fn event_line(ev: LightEvent, ae: &AutoExposure) -> String {
    let (e, g) = ae.caps(ae.light.stage);
    match ev {
        LightEvent::Extended(s) => format!("PIMAX_AE_EXTENDED: range extended to stage {s} (caps {e} us / gain {g})"),
        LightEvent::Retracted => format!("PIMAX_AE_EXTENDED: back to the normal range (caps {e} us / gain {g})"),
        LightEvent::LowRaised => format!("PIMAX_AE_LIGHT_LOW: {LOW_LIGHT_TEXT}"),
        LightEvent::LowCleared => "PIMAX_AE_LIGHT_LOW: cleared, the cameras see enough light again".into(),
    }
}

/// Log the event; on a raised warning also show it in the headset (WayVR's XSOverlay-format UDP listener on
/// 127.0.0.1:42069, the channel omarchyvrd and omarchyvr-voice use for in-headset toasts; nothing listens in a
/// SteamVR game session, then only the log and the app show it).
fn report(ev: LightEvent, ae: &AutoExposure) {
    eprintln!("{}", event_line(ev, ae));
    if ev == LightEvent::LowRaised {
        toast(LOW_LIGHT_TEXT);
    }
}

/// XSOverlay-format message for WayVR.
pub fn toast_message(text: &str) -> String {
    let mut q = String::new();
    for c in text.chars() {
        match c {
            '"' => q.push_str("\\\""),
            '\\' => q.push_str("\\\\"),
            c if (c as u32) < 0x20 => q.push(' '),
            c => q.push(c),
        }
    }
    format!("{{\"messageType\":1,\"title\":\"Pimax\",\"content\":\"{q}\",\"timeout\":8.0,\"volume\":-1.0}}")
}

fn toast(text: &str) {
    if let Ok(s) = std::net::UdpSocket::bind("127.0.0.1:0") {
        let _ = s.send_to(toast_message(text).as_bytes(), "127.0.0.1:42069");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_update_destroy_with_null_camera() {
        let a = pimax_cam_ae_create();
        assert!(!a.is_null());
        let mut o = CAeOut::default();
        // SAFETY: valid controller; a null camera is reported as an error.
        assert_eq!(
            unsafe { pimax_cam_ae_update(a, std::ptr::null(), 0, &mut o) },
            -1
        );
        // SAFETY: created above.
        unsafe { pimax_cam_ae_destroy(a) };
    }

    #[test]
    fn create_for_a_named_model() {
        for name in [c"crystal-super", c"dream-air", c""] {
            // SAFETY: NUL-terminated literal.
            let a = unsafe { pimax_cam_ae_create_model(name.as_ptr()) };
            assert!(!a.is_null(), "{name:?}");
            // SAFETY: created above.
            unsafe { pimax_cam_ae_destroy(a) };
        }
        // SAFETY: NULL is allowed (detect).
        let a = unsafe { pimax_cam_ae_create_model(std::ptr::null()) };
        assert!(!a.is_null());
        // SAFETY: created above.
        unsafe { pimax_cam_ae_destroy(a) };
        if std::env::var("PIMAX_CAM_AE_MODEL").is_err() {
            // SAFETY: NUL-terminated literal.
            assert!(unsafe { pimax_cam_ae_create_model(c"8kx".as_ptr()) }.is_null());
        }
        let c = AutoExposure::new(AeParams::for_model(AeModel::CrystalSuper));
        let l = model_line(&c, "USB 34a4:0040");
        assert!(l.starts_with("PIMAX_AE_MODEL: crystal-super (USB 34a4:0040)"), "{l}");
        assert!(l.contains("upper pair 2200..8941 us / gain 13..439"), "{l}");
        assert!(start_line(&c).starts_with("PIMAX_AE_EXTENDED: off"));
        let d = AutoExposure::new(AeParams::default());
        assert_eq!(model_line(&d, "x"), "PIMAX_AE_MODEL: dream-air (x); exposure 1140..4108 us, gain 13..215");
    }

    #[test]
    fn marker_and_toast_text() {
        let ae = AutoExposure::new(AeParams::default());
        assert!(start_line(&ae).starts_with("PIMAX_AE_EXTENDED: armed"));
        assert!(event_line(LightEvent::LowRaised, &ae).contains("too little infrared light"));
        let m = toast_message(LOW_LIGHT_TEXT);
        assert!(m.contains("\"messageType\":1") && m.contains("infrared"));
        assert!(toast_message("a\"b").contains("a\\\"b"));
    }

    #[test]
    fn out_layout_is_stable() {
        // The C header mirrors this layout.
        assert_eq!(
            std::mem::size_of::<CAeOut>(),
            8 + 24 + 24 + 8 + 8 + 4 + 8 + 8 + 8 + 8 + 8 + 4
        );
        assert_eq!(std::mem::offset_of!(CAeOut, payload), 32);
        assert_eq!(std::mem::offset_of!(CAeOut, mean), 92);
    }
}

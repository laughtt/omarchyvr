//! Auto-exposure (AE) for the tracking cameras (6DoF phase 2; evidence and frame layout in
//! `analysis/slam-exposure.md`).
//!
//! On Windows the host pushes exposure and gain for the four cameras with HID command `0x51` on the headset
//! (34a4:0044), about 12.5 times a second:
//!
//! ```text
//! 58 19 51 00 | 01 32 | e0 e1 e2 e3 | g0 g1 g2 g3 | led_e led_g      (u16 little endian)
//! ```
//!
//! `e`/`g` are the tracking-exposure values of cameras 0..3 (the lower pair 0,1 and the upper pair 2,3 always carry
//! equal values), `led_e`/`led_g` the short controller-LED exposure (constant 40/62). The camera frame metadata
//! echoes the values in effect, so the loop can be closed on what each image was really taken with. On Linux
//! nobody sends it and the cameras stay at the power-on default 10410/449.
//!
//! This module is pure logic: statistics of the tracking tiles -> a new setting per camera pair -> the command
//! payload. Sending is the driver's job and is OFF unless `PIMAX_CAM_AE=1` (needs the user's approval: the frame is
//! derived from captures). `PIMAX_CAM_AE=dry` runs the controller and only logs.
//!
//! Control law (per pair, both tiles pooled): histogram on a subsampled grid inside the fisheye image circle,
//! `m` = mean of the pixels below the 98th percentile (light sources and LED blobs do not drive the loop), error
//! `ln(target / m)` with a deadband, damped multiplicative step `r = exp(k * err)` clamped to `max_step`, applied to
//! the exposure*gain product **of the analysed frame** (from its metadata, so a frame taken before the last
//! command was applied gives the same answer again instead of a second step: no wind-up from the 1-3 frame
//! latency). The product is split exposure first (up to `exposure_max`, the motion-blur cap), then gain.

use crate::camera::FrameInfo;

/// HID command id of the exposure push.
pub const AE_CMD: u8 = 0x51;
/// Option byte of the exposure push.
pub const AE_OPT: u8 = 0x00;
/// Payload length (`01 32` + 10 u16): `58 19 51 00` + 22 bytes, LL = 22 + 3 = 0x19.
pub const AE_PAYLOAD_LEN: usize = 22;
/// The two constant leading payload bytes Windows always sends.
pub const AE_HEAD: [u8; 2] = [0x01, 0x32];
/// Linux power-on default of the tracking exposure (no AE).
pub const POWER_ON_EXPOSURE: u16 = 10410;
/// Windows' exposure ladder (gain 13): geometric, ratio ~1.0302 per step, anchored at 1682; observed values
/// 1682..4108 (`analysis/slam-exposure.md`). Exposure unit = microseconds [inferred].
pub const LADDER_ANCHOR: f64 = 1682.0;
/// Ladder ratio.
pub const LADDER_RATIO: f64 = 1.030_212;
/// Gain codes Windows used (both pairs), ascending.
pub const GAIN_CODES: [u16; 29] = [
    13, 15, 17, 19, 21, 23, 26, 29, 32, 35, 39, 43, 46, 53, 59, 71, 78, 79, 87, 95, 105, 116, 128,
    141, 155, 171, 188, 207, 215,
];

/// Windows' exposure values at gain 13, ascending (observed 1682..4108), extended below 1682 with the same ratio
/// down to Windows' observed minimum ~1140.
pub const EXPOSURE_LADDER: [u16; 44] = [
    1142, 1177, 1212, 1249, 1287, 1326, 1366, 1407, 1449, 1493, 1538, 1585, 1633, 1682, 1733, 1785,
    1839, 1895, 1952, 2011, 2072, 2135, 2200, 2266, 2334, 2405, 2478, 2553, 2630, 2709, 2791, 2875,
    2962, 3051, 3143, 3238, 3336, 3437, 3541, 3648, 3758, 3871, 3988, 4108,
];

/// Nearest ladder value (geometric extension outside the table).
pub fn ladder(e: f64) -> f64 {
    ladder_in(&EXPOSURE_LADDER, e)
}

/// Largest ladder value <= `e` (geometric extension below the table).
pub fn ladder_down(e: f64) -> f64 {
    ladder_down_in(&EXPOSURE_LADDER, e)
}

/// Linux power-on default of the tracking gain (no AE).
pub const POWER_ON_GAIN: u16 = 449;

/// Nearest value of an ascending `table` (geometric extension with Windows' ratio outside it).
pub fn ladder_in(table: &[u16], e: f64) -> f64 {
    let (lo, hi) = (table[0] as f64, table[table.len() - 1] as f64);
    if e < lo || e > hi {
        let k = ((e.max(1.0) / LADDER_ANCHOR).ln() / LADDER_RATIO.ln()).round();
        return (LADDER_ANCHOR * LADDER_RATIO.powf(k)).round();
    }
    let mut best = lo;
    for &v in table {
        if (v as f64 - e).abs() < (best - e).abs() {
            best = v as f64;
        }
    }
    best
}

/// Largest value of an ascending `table` <= `e` (geometric extension below it).
pub fn ladder_down_in(table: &[u16], e: f64) -> f64 {
    match table.iter().rev().find(|&&v| v as f64 <= e) {
        Some(&v) => v as f64,
        None => {
            let k = ((e.max(1.0) / LADDER_ANCHOR).ln() / LADDER_RATIO.ln() + 1e-9).floor();
            (LADDER_ANCHOR * LADDER_RATIO.powf(k)).round()
        }
    }
}

// ---- Crystal Super: what Pimax Play sent on Windows (capture 2026-10-17, 16751 commands in 14 takes;
// `analysis/crystal-super-ae.md`). Same frame, same 80 ms period, same pairs (0,1 / 2,3 always equal), same LED pair.

/// Crystal Super exposure ladder at the low gains: Windows' Dream Air ladder from the Crystal's lowest value (2200,
/// also its first command after a camera start) up to the soft cap 4108. Values seen at gain 13: 2200, 2875, 2962,
/// 3758, 3988, all on this ladder.
pub const CRYSTAL_EXPOSURE_LADDER: [u16; 22] = [
    2200, 2266, 2334, 2405, 2478, 2553, 2630, 2709, 2791, 2875, 2962, 3051, 3143, 3238, 3336, 3437, 3541,
    3648, 3758, 3871, 3988, 4108,
];

/// Crystal Super gain codes below the top gain (215), as Windows sent them on the Crystal.
pub const CRYSTAL_GAIN_CODES: [u16; 28] = [
    13, 17, 19, 21, 23, 29, 32, 35, 43, 46, 49, 53, 58, 65, 71, 78, 79, 87, 95, 105, 116, 128, 141, 155,
    171, 188, 207, 215,
];

/// Crystal Super settings above the soft exposure cap, ascending exposure x gain: Windows first raises the exposure
/// at gain 215 (3 % ladder, 4194..5641 us), then exposure and gain together, one fixed gain per exposure, up to the
/// power-on-like 10410 / 449. Every pair was sent by Windows; the Crystal used this range all session (median
/// lower pair 6739 / 288).
pub const CRYSTAL_HIGH: [(u16, u16); 41] = [
    (4194, 215), (4320, 215), (4450, 215), (4584, 215), (4722, 215), (4864, 215), (5051, 215), (5161, 215),
    (5316, 215), (5476, 215), (5641, 215), (5720, 220), (5811, 226), (5986, 237), (6072, 244), (6166, 249),
    (6351, 261), (6440, 270), (6542, 275), (6632, 280), (6739, 288), (6942, 303), (7042, 309), (7151, 318),
    (7251, 326), (7366, 334), (7462, 334), (7587, 351), (7662, 358), (7815, 368), (7880, 370), (7980, 375),
    (8050, 387), (8292, 406), (8342, 421), (8541, 426), (8741, 431), (8941, 439), (9541, 439), (9841, 446),
    (10410, 449),
];

/// Crystal Super firmware default of the tracking cameras before the first push (frame metadata, 2026-10-18).
pub const CRYSTAL_POWER_ON: (u16, u16) = (3500, 60);

/// Windows' range of one camera pair: (exposure min, exposure max, gain min, gain max).
pub type PairRange = (u16, u16, u16, u16);

/// Crystal Super ranges per pair, exactly what Windows sent: lower pair (cameras 0, 1) 2200..10410 us, gain 13..449;
/// upper pair (2, 3) 2200..8941 us, gain 13..439. The same ranges are the allowlist of `camera_ae` in
/// `pimax-device/devices/crystal-super/device.toml`.
pub const CRYSTAL_PAIR_RANGES: [PairRange; 2] = [(2200, 10410, 13, 449), (2200, 8941, 13, 439)];

/// The headset whose Windows auto-exposure the controller follows. Only the model constants differ (ladder, gain
/// codes, limits, power-on default); the control law is the same.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AeModel {
    /// Pimax Dream Air (34a4:0044): `analysis/slam-exposure.md`. The default, unchanged.
    #[default]
    DreamAir,
    /// Pimax Crystal Super (34a4:0040): `analysis/crystal-super-ae.md`.
    CrystalSuper,
}

impl AeModel {
    /// Device module id (`pimax-device/devices/<id>`).
    pub fn id(self) -> &'static str {
        match self {
            AeModel::DreamAir => "dream-air",
            AeModel::CrystalSuper => "crystal-super",
        }
    }

    /// From a module id (`dream-air`, `crystal-super`; a few spellings).
    pub fn parse(s: &str) -> Option<AeModel> {
        match s.trim().to_ascii_lowercase().replace(['_', ' '], "-").as_str() {
            "dream-air" | "dreamair" | "0044" | "0x0044" => Some(AeModel::DreamAir),
            "crystal-super" | "crystalsuper" | "crystal" | "0040" | "0x0040" => Some(AeModel::CrystalSuper),
            _ => None,
        }
    }

    /// From the product id of the headset's HID chip.
    pub fn by_pid(pid: u16) -> Option<AeModel> {
        match pid {
            0x0044 => Some(AeModel::DreamAir),
            0x0040 => Some(AeModel::CrystalSuper),
            _ => None,
        }
    }

    /// Exposure ladder at the low gains.
    pub fn ladder(self) -> &'static [u16] {
        match self {
            AeModel::DreamAir => &EXPOSURE_LADDER,
            AeModel::CrystalSuper => &CRYSTAL_EXPOSURE_LADDER,
        }
    }

    /// Gain codes used with the soft exposure cap.
    pub fn gain_codes(self) -> &'static [u16] {
        match self {
            AeModel::DreamAir => &GAIN_CODES,
            AeModel::CrystalSuper => &CRYSTAL_GAIN_CODES,
        }
    }

    /// (exposure, gain) settings above the soft cap at the top gain code (empty: the Dream Air's split).
    pub fn high(self) -> &'static [(u16, u16)] {
        match self {
            AeModel::DreamAir => &[],
            AeModel::CrystalSuper => &CRYSTAL_HIGH,
        }
    }

    /// The cameras' setting before the first push.
    pub fn power_on(self) -> (u16, u16) {
        match self {
            AeModel::DreamAir => (POWER_ON_EXPOSURE, POWER_ON_GAIN),
            AeModel::CrystalSuper => CRYSTAL_POWER_ON,
        }
    }

    /// Windows' range of each pair, enforced on every setting (None: not enforced, the Dream Air as before).
    pub fn pair_ranges(self) -> Option<[PairRange; 2]> {
        match self {
            AeModel::DreamAir => None,
            AeModel::CrystalSuper => Some(CRYSTAL_PAIR_RANGES),
        }
    }

    /// The setting is inside Windows' ranges of this model (always true without enforced ranges).
    pub fn allows(self, s: &AeSetting) -> bool {
        let Some(r) = self.pair_ranges() else { return true };
        let ok = |c: usize, (elo, ehi, glo, ghi): PairRange| {
            (elo..=ehi).contains(&s.exposure[c]) && (glo..=ghi).contains(&s.gain[c])
        };
        ok(0, r[0]) && ok(1, r[0]) && ok(2, r[1]) && ok(3, r[1]) && s.led_exposure == 40 && s.led_gain == 62
    }

    /// The model of the headset on USB (`PIMAX_CAM_AE_MODEL` first, then the HID chip in sysfs; Dream Air when none
    /// or both are found) and where that came from.
    pub fn detect() -> (AeModel, String) {
        if let Ok(v) = std::env::var("PIMAX_CAM_AE_MODEL") {
            if let Some(m) = AeModel::parse(&v) {
                return (m, "PIMAX_CAM_AE_MODEL".into());
            }
            if !v.trim().is_empty() {
                eprintln!("PIMAX_CAM_AE_MODEL={v} unknown (dream-air or crystal-super); looking at USB");
            }
        }
        let pids = usb_pids_sysfs(std::path::Path::new("/sys/bus/usb/devices"));
        let (m, why) = model_from_pids(&pids);
        (m, why.into())
    }
}

/// Product ids of the vendor 34a4 devices in a sysfs USB directory.
pub fn usb_pids_sysfs(dir: &std::path::Path) -> Vec<u16> {
    let mut out = Vec::new();
    let Ok(rd) = std::fs::read_dir(dir) else { return out };
    for e in rd.flatten() {
        let p = e.path();
        let rd = |n: &str| std::fs::read_to_string(p.join(n)).ok().map(|s| s.trim().to_ascii_lowercase());
        if rd("idVendor").as_deref() == Some("34a4") {
            if let Some(pid) = rd("idProduct").and_then(|s| u16::from_str_radix(&s, 16).ok()) {
                out.push(pid);
            }
        }
    }
    out
}

/// The model from the Pimax product ids on USB: the Crystal Super only when its HID chip (0040) is there and the
/// Dream Air's (0044) is not; everything else keeps the Dream Air.
pub fn model_from_pids(pids: &[u16]) -> (AeModel, &'static str) {
    let crystal = pids.contains(&0x0040);
    let dream = pids.contains(&0x0044);
    match (crystal, dream) {
        (true, false) => (AeModel::CrystalSuper, "USB 34a4:0040"),
        (false, true) => (AeModel::DreamAir, "USB 34a4:0044"),
        (true, true) => (AeModel::DreamAir, "both headsets on USB, Dream Air kept"),
        (false, false) => (AeModel::DreamAir, "no headset on USB, Dream Air default"),
    }
}

/// Exposure and gain of the four cameras plus the LED-frame pair (raw units, as on the wire).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AeSetting {
    /// Tracking exposure of cameras 0..3.
    pub exposure: [u16; 4],
    /// Tracking gain of cameras 0..3.
    pub gain: [u16; 4],
    /// LED-frame exposure (Windows: 40).
    pub led_exposure: u16,
    /// LED-frame gain (Windows: 62).
    pub led_gain: u16,
}

impl AeSetting {
    /// The setting a frame's metadata reports (LED pair: the given defaults, the metadata does not carry it).
    pub fn from_info(fi: &FrameInfo, led_exposure: u16, led_gain: u16) -> AeSetting {
        AeSetting {
            exposure: fi.exposure,
            gain: fi.gain,
            led_exposure,
            led_gain,
        }
    }
}

/// Command payload (22 bytes after `58 19 51 00`).
pub fn payload(s: &AeSetting) -> [u8; AE_PAYLOAD_LEN] {
    let mut p = [0u8; AE_PAYLOAD_LEN];
    p[..2].copy_from_slice(&AE_HEAD);
    let words = s
        .exposure
        .into_iter()
        .chain(s.gain)
        .chain([s.led_exposure, s.led_gain]);
    for (i, w) in words.enumerate() {
        p[2 + 2 * i..4 + 2 * i].copy_from_slice(&w.to_le_bytes());
    }
    p
}

/// Parse a payload (or a whole `58 19 51 00 ..` report). `None` if it is not an exposure push.
pub fn parse_payload(b: &[u8]) -> Option<AeSetting> {
    let p = if b.len() >= 4 && b[0] == 0x58 && b[1] == 0x19 && b[2] == AE_CMD && b[3] == AE_OPT {
        &b[4..]
    } else {
        b
    };
    if p.len() < AE_PAYLOAD_LEN || p[..2] != AE_HEAD {
        return None;
    }
    let w = |i: usize| u16::from_le_bytes([p[2 + 2 * i], p[3 + 2 * i]]);
    Some(AeSetting {
        exposure: [w(0), w(1), w(2), w(3)],
        gain: [w(4), w(5), w(6), w(7)],
        led_exposure: w(8),
        led_gain: w(9),
    })
}

/// A borrowed 640x480 tile inside a raw frame.
#[derive(Clone, Copy)]
pub struct TileView<'a> {
    /// Raw frame bytes.
    pub data: &'a [u8],
    /// Row stride of `data`.
    pub stride: usize,
    /// First column of the tile.
    pub x0: usize,
    /// First row of the tile.
    pub y0: usize,
    /// Tile width.
    pub w: usize,
    /// Tile height.
    pub h: usize,
}

impl TileView<'_> {
    /// Pixel (x, y) of the tile (no bounds check beyond the slice's own).
    #[inline]
    pub fn px(&self, x: usize, y: usize) -> u8 {
        self.data[(self.y0 + y) * self.stride + self.x0 + x]
    }
}

/// Brightness statistics of one tile (or a pooled pair).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TileStats {
    /// Mean of all sampled pixels.
    pub mean: f64,
    /// Mean of the pixels at or below the `trim` percentile (the AE input).
    pub trimmed_mean: f64,
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// Fraction of samples at or above the saturation level.
    pub sat_frac: f64,
    /// Fraction of samples at or below 8 (black).
    pub dark_frac: f64,
    /// Samples used.
    pub n: u32,
}

/// 256-bin histogram.
#[derive(Clone, Debug)]
pub struct Histogram {
    /// Counts.
    pub bins: [u32; 256],
}

impl Default for Histogram {
    fn default() -> Self {
        Histogram { bins: [0; 256] }
    }
}

impl Histogram {
    /// Total count.
    pub fn total(&self) -> u64 {
        self.bins.iter().map(|&c| c as u64).sum()
    }

    /// Add another histogram.
    pub fn merge(&mut self, o: &Histogram) {
        for (a, b) in self.bins.iter_mut().zip(o.bins.iter()) {
            *a += b;
        }
    }

    /// Value at quantile `q` (0..1).
    pub fn quantile(&self, q: f64) -> f64 {
        let n = self.total();
        if n == 0 {
            return 0.0;
        }
        let want = (q.clamp(0.0, 1.0) * n as f64).ceil().max(1.0) as u64;
        let mut acc = 0u64;
        for (v, &c) in self.bins.iter().enumerate() {
            acc += c as u64;
            if acc >= want {
                return v as f64;
            }
        }
        255.0
    }

    /// Statistics; `trim` = percentile (0..1) above which pixels are left out of the trimmed mean.
    pub fn stats(&self, trim: f64, sat_level: u8) -> TileStats {
        let n = self.total();
        if n == 0 {
            return TileStats::default();
        }
        let cut = self.quantile(trim) as usize;
        let (mut sum, mut tsum, mut tn, mut sat, mut dark) = (0f64, 0f64, 0u64, 0u64, 0u64);
        for (v, &c) in self.bins.iter().enumerate() {
            let c = c as u64;
            sum += (v as u64 * c) as f64;
            if v <= cut {
                tsum += (v as u64 * c) as f64;
                tn += c;
            }
            if v >= sat_level as usize {
                sat += c;
            }
            if v <= 8 {
                dark += c;
            }
        }
        TileStats {
            mean: sum / n as f64,
            trimmed_mean: if tn > 0 { tsum / tn as f64 } else { 0.0 },
            p50: self.quantile(0.5),
            p95: self.quantile(0.95),
            sat_frac: sat as f64 / n as f64,
            dark_frac: dark as f64 / n as f64,
            n: n as u32,
        }
    }
}

/// Sampling mask: pixels inside an ellipse centred on the tile (the fisheye image circle; the corners are dark
/// lens border and would drag the mean down).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mask {
    /// Centre x (tile pixels).
    pub cx: f64,
    /// Centre y.
    pub cy: f64,
    /// Radius x.
    pub rx: f64,
    /// Radius y.
    pub ry: f64,
}

impl Default for Mask {
    /// Image rows 24..489 of the frame are lit (camera-frames.md); a 0.95 x half-width / 0.97 x half-height
    /// ellipse keeps the dark corners out without losing the scene.
    fn default() -> Self {
        Mask {
            cx: 319.5,
            cy: 239.5,
            rx: 320.0 * 0.95,
            ry: 240.0 * 0.97,
        }
    }
}

impl Mask {
    /// Inside?
    #[inline]
    pub fn inside(&self, x: f64, y: f64) -> bool {
        let dx = (x - self.cx) / self.rx;
        let dy = (y - self.cy) / self.ry;
        dx * dx + dy * dy <= 1.0
    }
}

/// Histogram of a tile on a `step`-pixel grid inside `mask`.
pub fn tile_histogram(t: &TileView, step: usize, mask: &Mask) -> Histogram {
    let mut h = Histogram::default();
    let step = step.max(1);
    let mut y = step / 2;
    while y < t.h {
        let mut x = step / 2;
        while x < t.w {
            if mask.inside(x as f64, y as f64) {
                h.bins[t.px(x, y) as usize] += 1;
            }
            x += step;
        }
        y += step;
    }
    h
}

/// Side of a metering cell in tile pixels.
pub const METER_CELL: usize = 16;
/// A cell with more than this fraction of clipped samples is left out of the mean.
pub const METER_CELL_CLIP: f64 = 0.5;

/// The ONE brightness metric (auto exposure, room setup, the "good frame" bit): the mean over the lit image circle
/// (the black lens corners are outside [`Mask`]) that leaves out cells which are more than half clipped, and the
/// clipped fraction over all masked samples.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SceneMetric {
    /// Masked mean of the unclipped cells (0..255). All masked samples when every cell is clipped.
    pub mean: f64,
    /// Fraction of masked samples at or above the saturation level.
    pub sat: f64,
    /// Fraction of masked samples at or below 8.
    pub dark: f64,
    /// Masked samples.
    pub n: u32,
}

/// Metric plus the histograms it was made from (`all` = every masked sample, `kept` = the unclipped cells).
pub fn scene_metric_hist(tiles: &[TileView], step: usize, mask: &Mask, sat_level: u8) -> (SceneMetric, Histogram, Histogram) {
    let step = step.max(1);
    let mut all = Histogram::default();
    let mut kept = Histogram::default();
    let (mut sat, mut dark) = (0u64, 0u64);
    for t in tiles {
        let cw = t.w.div_ceil(METER_CELL);
        let ch = t.h.div_ceil(METER_CELL);
        // Per cell: a small histogram would be heavy; keep sum, count and clipped count, and a list of samples.
        let mut cells: Vec<(u32, u32, u32)> = vec![(0, 0, 0); cw * ch]; // (sum, n, clipped)
        let mut samples: Vec<(u16, u8)> = Vec::new();
        let mut y = step / 2;
        while y < t.h {
            let mut x = step / 2;
            while x < t.w {
                if mask.inside(x as f64, y as f64) {
                    let v = t.px(x, y);
                    let ci = (y / METER_CELL) * cw + x / METER_CELL;
                    let c = &mut cells[ci];
                    c.0 += v as u32;
                    c.1 += 1;
                    if v >= sat_level {
                        c.2 += 1;
                        sat += 1;
                    }
                    if v <= 8 {
                        dark += 1;
                    }
                    all.bins[v as usize] += 1;
                    samples.push((ci as u16, v));
                }
                x += step;
            }
            y += step;
        }
        for (ci, v) in samples {
            let c = cells[ci as usize];
            if (c.2 as f64) <= METER_CELL_CLIP * c.1 as f64 {
                kept.bins[v as usize] += 1;
            }
        }
    }
    let n = all.total();
    let src = if kept.total() > 0 { &kept } else { &all };
    let m = if src.total() > 0 {
        src.bins.iter().enumerate().map(|(v, &c)| (v as u64 * c as u64) as f64).sum::<f64>() / src.total() as f64
    } else {
        0.0
    };
    let nf = n.max(1) as f64;
    (
        SceneMetric { mean: m, sat: sat as f64 / nf, dark: dark as f64 / nf, n: n as u32 },
        all,
        kept,
    )
}

/// The shared brightness metric of some tiles (one camera pair, or one frame set).
pub fn scene_metric(tiles: &[TileView], step: usize, mask: &Mask, sat_level: u8) -> SceneMetric {
    scene_metric_hist(tiles, step, mask, sat_level).0
}

/// [`scene_metric`] of a tightly packed 8-bit grey image (room setup, tests): default mask scaled to `w` x `h`,
/// saturation level 250, every second pixel.
pub fn scene_metric_grey(data: &[u8], w: usize, h: usize) -> SceneMetric {
    if w == 0 || h == 0 || data.len() < w * h {
        return SceneMetric::default();
    }
    let d = Mask::default();
    let mask = Mask {
        cx: (w as f64 - 1.0) / 2.0,
        cy: (h as f64 - 1.0) / 2.0,
        rx: w as f64 / 2.0 * (d.rx / 320.0),
        ry: h as f64 / 2.0 * (d.ry / 240.0),
    };
    scene_metric(&[TileView { data, stride: w, x0: 0, y0: 0, w, h }], 2, &mask, 250)
}

/// "Good frame": masked mean at least this ...
pub const GOOD_MEAN_MIN: f64 = 30.0;
/// ... and at most this ...
pub const GOOD_MEAN_MAX: f64 = 190.0;
/// ... clipped fraction at most this ...
pub const GOOD_SAT_MAX: f64 = 0.20;
/// ... and quiet for this long after a big exposure step (ns).
pub const GOOD_QUIET_NS: u64 = 1_000_000_000;
/// A step of the exposure x gain product by more than this ratio (either way) is a big step.
pub const BIG_STEP_RATIO: f64 = 1.25;
/// Exposure or gain that differs from the previous frame by more than this fraction means a step landed.
pub const STEP_LANDED_FRAC: f64 = 0.03;

/// The one definition of a good tracking frame for a pair (room service, floor, light): the masked mean is in
/// 30..=190, at most 20 % is clipped, no exposure change landed in this frame, it is not within 1 s of a big step,
/// and the light is not settling. LED frames are never gated by this.
pub fn is_good_frame(m: &SceneMetric, step_landed: bool, since_big_step_ns: Option<u64>, settling: bool) -> bool {
    m.n > 0
        && (GOOD_MEAN_MIN..=GOOD_MEAN_MAX).contains(&m.mean)
        && m.sat <= GOOD_SAT_MAX
        && !step_landed
        && since_big_step_ns.is_none_or(|t| t >= GOOD_QUIET_NS)
        && !settling
}

/// Follows a pair's own frame metadata (exposure, gain) to find landed and big steps for [`is_good_frame`].
#[derive(Clone, Copy, Debug, Default)]
pub struct StepWatch {
    prev: Option<(u16, u16)>,
    big_at: Option<u64>,
}

impl StepWatch {
    /// One tracking frame of the pair: its exposure and gain from the metadata and its host time. Returns
    /// (a step landed in this frame, time since the last big step).
    pub fn push(&mut self, exposure: u16, gain: u16, now_ns: u64) -> (bool, Option<u64>) {
        let mut landed = false;
        if let Some((pe, pg)) = self.prev {
            let d = |a: u16, b: u16| (a as f64 - b as f64).abs() > STEP_LANDED_FRAC * b.max(1) as f64;
            landed = d(exposure, pe) || d(gain, pg);
            let (a, b) = (exposure as f64 * gain as f64, pe as f64 * pg as f64);
            let r = a.max(1.0) / b.max(1.0);
            if r > BIG_STEP_RATIO || r < 1.0 / BIG_STEP_RATIO {
                self.big_at = Some(now_ns);
            }
        }
        self.prev = Some((exposure, gain));
        (landed, self.big_at.map(|t| now_ns.saturating_sub(t)))
    }

    /// [`StepWatch::push`] and [`is_good_frame`] in one call.
    pub fn good(&mut self, m: &SceneMetric, exposure: u16, gain: u16, now_ns: u64, settling: bool) -> bool {
        let (landed, since) = self.push(exposure, gain, now_ns);
        is_good_frame(m, landed, since, settling)
    }
}

/// Controller parameters (raw units as on the wire).
#[derive(Clone, Debug, PartialEq)]
pub struct AeParams {
    /// Target of the trimmed mean (0..255).
    pub target: f64,
    /// No change while `|ln(m / target)|` is below this (0.15 = +-16 %).
    pub deadband: f64,
    /// Damping of the log-domain step (1 = one-step model correction).
    pub k: f64,
    /// Largest brightness ratio per update (both directions).
    pub max_step: f64,
    /// Largest ratio per update while the image is mostly saturated or black (faster recovery).
    pub max_step_far: f64,
    /// Exposure range.
    pub exposure_min: u16,
    /// Exposure cap (motion blur); gain is used above it.
    pub exposure_max: u16,
    /// Gain range.
    pub gain_min: u16,
    /// Gain cap.
    pub gain_max: u16,
    /// Pixel level counted as saturated.
    pub sat_level: u8,
    /// Above this saturated fraction the loop pulls down by at least `sat_pull` even if the mean looks fine.
    pub sat_max_frac: f64,
    /// Minimum pull-down (log) when too much is saturated.
    pub sat_pull: f64,
    /// Percentile for the trimmed mean.
    pub trim: f64,
    /// Sampling grid step (pixels).
    pub step: usize,
    /// Command period (Windows: 80 ms = 12.5 Hz).
    pub period_ns: u64,
    /// LED-frame pair sent unchanged.
    pub led_exposure: u16,
    /// LED-frame gain.
    pub led_gain: u16,
    /// Camera pairs controlled together (stereo matching wants equal brightness within a pair).
    pub pairs: [[usize; 2]; 2],
    /// Control the upper pair (cameras 2, 3) too.
    pub upper: bool,
    /// Quantise exposure to Windows' ladder and gain to the codes Windows used.
    pub quantize: bool,
    /// Exposure up to which the split prefers exposure over gain (the normal ceiling). Above it gain comes first and
    /// exposure only grows (up to `exposure_max`) once the gain is at its cap.
    pub exposure_soft: u16,
    /// The automatic extended range (low light).
    pub light: LightParams,
    /// Control law: [`AeLaw::V2`] (masked, unclipped-cell metering, floor 1140 us, rate limited) or the old one.
    pub law: AeLaw,
    /// Shortest time between two changes of one pair's setting (ns; 0 = no limit). 200 ms = 5 changes/s.
    pub min_change_ns: u64,
    /// The "too bright" rule releases (and brightening is allowed again) only below this clipped fraction.
    pub sat_release: f64,
    /// Headset model: ladder, gain codes, the settings above the soft cap and Windows' per-pair ranges.
    pub model: AeModel,
}

/// Which control law runs. `PIMAX_CAM_AE_LAW=old` (also `v1`) selects [`AeLaw::V1`], the kill switch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AeLaw {
    /// The law before 2026-10-17: trimmed mean, floor 100 us, rule at 5 % clipped, deadband 0.08.
    V1,
    /// Masked mean over unclipped cells, floor 1140 us (300 us low stage), rule above 20 %, deadband 0.15, 5 changes/s.
    V2,
}

/// Exposure floor of the normal range (Windows' lowest observed value).
pub const EXPOSURE_FLOOR: u16 = 1140;
/// Exposure floor of the low stage (only after 3 s of more than 30 % clipped).
pub const EXPOSURE_LOW_STAGE: u16 = 300;

impl Default for AeParams {
    /// Windows' behaviour from the captures (`analysis/slam-exposure.md`): target mean ~62/255, one 3 % ladder
    /// step per command typical (small steps keep the optical flow's brightness constancy), exposure ladder
    /// 1140..4108 at gain 13, gain codes 13..215 only above the exposure ceiling. The floor goes below Windows'
    /// indoor minimum (100: the same sensors run 40 µs LED exposures every other frame) for sunlit rooms. A step never moves brightness
    /// the wrong way, so a scene too dark even for the power-on setting keeps it instead of trading it for the cap.
    fn default() -> Self {
        AeParams {
            target: 64.0,
            deadband: 0.15,
            k: 0.5,
            max_step: LADDER_RATIO * LADDER_RATIO * LADDER_RATIO,
            max_step_far: 2.0,
            exposure_min: EXPOSURE_FLOOR,
            exposure_max: 4108,
            gain_min: 13,
            gain_max: 215,
            sat_level: 250,
            sat_max_frac: 0.20,
            sat_pull: 0.15,
            trim: 0.98,
            step: 4,
            period_ns: 80_000_000,
            led_exposure: 40,
            led_gain: 62,
            pairs: [[0, 1], [2, 3]],
            upper: true,
            quantize: true,
            exposure_soft: 4108,
            light: LightParams::default(),
            law: AeLaw::V2,
            min_change_ns: 200_000_000,
            sat_release: 0.12,
            model: AeModel::DreamAir,
        }
    }
}

impl AeParams {
    /// The defaults for a headset model. Dream Air: [`AeParams::default`], unchanged. Crystal Super: Windows'
    /// Crystal ranges (2200..10410 us, gain 13..449; upper pair up to 8941 / 439), the soft cap 4108 (gain before
    /// exposure above it, as Windows), no automatic extended range (the normal range already reaches 10410 / 449)
    /// and no clip-driven low stage (Windows never went below 2200 on the Crystal).
    pub fn for_model(model: AeModel) -> AeParams {
        match model {
            AeModel::DreamAir => AeParams::default(),
            AeModel::CrystalSuper => {
                let [lo, _] = CRYSTAL_PAIR_RANGES;
                AeParams {
                    exposure_min: lo.0,
                    exposure_max: lo.1,
                    gain_min: lo.2,
                    gain_max: lo.3,
                    exposure_soft: 4108,
                    light: LightParams {
                        ext_exposure: lo.1,
                        ext_gain: lo.3,
                        clip_enabled: false,
                        ..LightParams::default()
                    },
                    model,
                    ..AeParams::default()
                }
            }
        }
    }

    /// [`AeParams::old_law`] for a model (the Crystal keeps its Windows floor).
    pub fn old_law_for(model: AeModel) -> AeParams {
        match model {
            AeModel::DreamAir => AeParams::old_law(),
            AeModel::CrystalSuper => {
                let o = AeParams::old_law();
                AeParams {
                    deadband: o.deadband,
                    sat_max_frac: o.sat_max_frac,
                    law: o.law,
                    min_change_ns: o.min_change_ns,
                    sat_release: o.sat_release,
                    ..AeParams::for_model(model)
                }
            }
        }
    }
}

impl AeParams {
    /// The old law (kill switch): floor 100 us, rule at 5 % clipped, deadband 0.08, trimmed-mean metering, no rate
    /// limit, no low stage.
    pub fn old_law() -> AeParams {
        AeParams {
            deadband: 0.08,
            exposure_min: 100,
            sat_max_frac: 0.05,
            law: AeLaw::V1,
            min_change_ns: 0,
            sat_release: 0.05,
            light: LightParams { clip_enabled: false, ..LightParams::default() },
            ..AeParams::default()
        }
    }
}

fn env_f64(name: &str) -> Option<f64> {
    std::env::var(name).ok().and_then(|v| v.trim().parse().ok())
}

impl AeParams {
    /// Defaults with `PIMAX_CAM_AE_TARGET`, `_EXP_MAX`, `_EXP_MIN`, `_GAIN_MAX`, `_GAIN_MIN`, `_PERIOD_MS`,
    /// `_K`, `_DEADBAND`, `_UPPER` (0/1) overrides, and `PIMAX_CAM_AE_LAW=old` (kill switch: the previous law). Values are clamped to sane ranges.
    pub fn from_env() -> AeParams {
        AeParams::from_env_model(AeModel::DreamAir)
    }

    /// [`AeParams::from_env`] for a headset model. For the Crystal Super the exposure and gain overrides stay inside
    /// Windows' Crystal range (2200..10410 us, gain 13..449); the Dream Air's bounds are unchanged.
    pub fn from_env_model(model: AeModel) -> AeParams {
        let old = std::env::var("PIMAX_CAM_AE_LAW")
            .is_ok_and(|v| matches!(v.trim().to_ascii_lowercase().as_str(), "old" | "v1" | "off" | "0"));
        let mut p = if old { AeParams::old_law_for(model) } else { AeParams::for_model(model) };
        // (exposure-cap floor, exposure-floor floor, gain floor, ceilings) for the overrides.
        let (emax_lo, emin_lo, g_lo, e_hi, g_hi) = match model.pair_ranges() {
            None => (100.0, 1.0, 1.0, 10410.0, 449.0),
            Some([r, _]) => (r.0 as f64, r.0 as f64, r.2 as f64, r.1 as f64, r.3 as f64),
        };
        if let Some(v) = env_f64("PIMAX_CAM_AE_TARGET") {
            p.target = v.clamp(20.0, 200.0);
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_EXP_MAX") {
            p.exposure_max = v.clamp(emax_lo, e_hi) as u16;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_EXP_MIN") {
            p.exposure_min = v.clamp(emin_lo, p.exposure_max as f64) as u16;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_GAIN_MAX") {
            p.gain_max = v.clamp(g_lo, g_hi) as u16;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_GAIN_MIN") {
            p.gain_min = v.clamp(g_lo, p.gain_max as f64) as u16;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_PERIOD_MS") {
            p.period_ns = (v.clamp(40.0, 2000.0) * 1e6) as u64;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_K") {
            p.k = v.clamp(0.1, 1.0);
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_DEADBAND") {
            p.deadband = v.clamp(0.0, 1.0);
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_UPPER") {
            p.upper = v != 0.0;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_QUANTIZE") {
            p.quantize = v != 0.0;
        }
        if let Some(v) = env_f64("PIMAX_CAM_AE_EXTEND") {
            p.light.enabled = v != 0.0;
        }
        p.exposure_soft = p.exposure_max.min(p.exposure_soft);
        // An explicit exposure cap from the environment is also the soft one.
        if env_f64("PIMAX_CAM_AE_EXP_MAX").is_some() {
            p.exposure_soft = p.exposure_max;
        }
        p
    }

    /// Exposure and gain caps of pair `k` (0 lower, 1 upper): the current caps, tightened to the model's Windows
    /// range of that pair.
    pub fn pair_caps(&self, k: usize) -> (u16, u16, u16, u16) {
        let (mut elo, mut ehi, mut glo, mut ghi) = (self.exposure_min, self.exposure_max, self.gain_min, self.gain_max);
        if let Some(r) = self.model.pair_ranges() {
            let (a, b, c, d) = r[k.min(1)];
            elo = elo.clamp(a, b);
            ehi = ehi.clamp(elo, b);
            glo = glo.clamp(c, d);
            ghi = ghi.clamp(glo, d);
        }
        (elo, ehi.max(elo), glo, ghi.max(glo))
    }

    /// Split a brightness product `e * g` into (exposure, gain): exposure first up to the cap, then gain (as
    /// Windows does). Gain is taken as linear in its raw code (Windows lowers e by ~7 % when g steps 13 -> 15,
    /// consistent with that). With `quantize`, values snap to Windows' ladder / gain codes; above the exposure
    /// cap the smallest gain code that reaches the product is used and the exposure fine-tunes below the cap.
    /// (The lower pair's caps; [`AeParams::split_pair`] for the upper pair.)
    pub fn split(&self, product: f64) -> (u16, u16) {
        self.split_pair(0, product)
    }

    /// [`AeParams::split`] within the caps of pair `k` ([`AeParams::pair_caps`]). Above the top gain code at the soft
    /// cap, a model with a table of higher settings ([`AeModel::high`], the Crystal Super) takes the nearest one;
    /// the Dream Air grows the exposure (and uses the power-on gain 449 when the cap allows it).
    pub fn split_pair(&self, k: usize, product: f64) -> (u16, u16) {
        let (elo, ehi, glo, ghi) = self.pair_caps(k);
        let clamp = |e: f64, g: f64| -> (u16, u16) {
            (
                e.round().clamp(elo as f64, ehi as f64) as u16,
                g.round().clamp(glo as f64, ghi as f64) as u16,
            )
        };
        let g_min = glo.max(1) as f64;
        let (e_lo, e_hi) = (elo as f64, ehi as f64);
        let e_soft = (self.exposure_soft as f64).clamp(e_lo, e_hi);
        if !self.quantize {
            let e = (product / g_min).clamp(e_lo, e_soft);
            let g = (product / e.max(1.0)).clamp(g_min, ghi as f64);
            // Gain exhausted: the exposure may go above the soft cap (extended range).
            let e = if product / g > e { product / g } else { e };
            return clamp(e, g);
        }
        let table = self.model.ladder();
        if product / g_min <= e_soft {
            let e = ladder_in(table, product / g_min).clamp(e_lo, e_soft);
            return clamp(e, g_min);
        }
        let high = self.model.high();
        let mut codes: Vec<u16> = self
            .model
            .gain_codes()
            .iter()
            .copied()
            .filter(|&g| g >= glo && g <= ghi)
            .collect();
        // The power-on gain (Windows' 449) is the one code above the ladder; only when the cap allows it.
        if high.is_empty() && ghi >= POWER_ON_GAIN && glo <= POWER_ON_GAIN {
            codes.push(POWER_ON_GAIN);
        }
        let need = product / e_soft;
        if let Some(g) = codes.iter().copied().find(|&g| g as f64 >= need) {
            let e = ladder_down_in(table, product / g as f64).clamp(e_lo, e_soft);
            return clamp(e, g as f64);
        }
        if !high.is_empty() {
            // The top of the gain stage and every higher setting inside the caps: the nearest in log brightness.
            let mut cand: Vec<(u16, u16)> = Vec::new();
            if let Some(&g) = codes.last() {
                cand.push(clamp(ladder_down_in(table, e_soft), g as f64));
            }
            cand.extend(high.iter().copied().filter(|&(e, g)| e >= elo && e <= ehi && g >= glo && g <= ghi));
            let dist = |&(e, g): &(u16, u16)| ((e as f64 * g as f64).max(1.0) / product.max(1.0)).ln().abs();
            if let Some(best) = cand.iter().copied().min_by(|a, b| dist(a).total_cmp(&dist(b))) {
                return best;
            }
        }
        // Every gain code is used up: the exposure grows past the soft cap (exposure last), up to `exposure_max`.
        let g = codes.last().copied().unwrap_or(glo) as f64;
        let e = ladder_in(table, product / g).clamp(e_lo, e_hi);
        clamp(e, g)
    }

    /// The pair's setting inside its caps: unchanged when it already is, otherwise the split of its product.
    pub fn fit_pair(&self, k: usize, e: u16, g: u16) -> (u16, u16) {
        let (elo, ehi, glo, ghi) = self.pair_caps(k);
        if self.model.pair_ranges().is_none() || ((elo..=ehi).contains(&e) && (glo..=ghi).contains(&g)) {
            return (e, g);
        }
        self.split_pair(k, e.max(1) as f64 * g.max(1) as f64)
    }
}

/// Thresholds of the low-light governor (extended range and the "tracking light low" warning). Means are the
/// trimmed mean of the tracking tiles, 0..255.
#[derive(Clone, Debug, PartialEq)]
pub struct LightParams {
    /// Automatic extended range on (`PIMAX_CAM_AE_EXTEND=0` turns it off; the warning still works).
    pub enabled: bool,
    /// Extend while the loop sits at its ceiling with a mean below this ...
    pub extend_mean: f64,
    /// ... for this long (ns).
    pub extend_ns: u64,
    /// Back to the normal range when the mean is above this ...
    pub retract_mean: f64,
    /// ... or the loop needs less than half of the normal ceiling (mean at least this) ...
    pub settled_mean: f64,
    /// ... for this long (ns).
    pub retract_ns: u64,
    /// Warn when even the full extended range leaves the mean below this ...
    pub low_mean: f64,
    /// ... for this long (ns).
    pub low_ns: u64,
    /// The warning clears when the mean is above this ...
    pub clear_mean: f64,
    /// ... for this long (ns).
    pub clear_ns: u64,
    /// Extended exposure cap (the camera's power-on value, which Windows holds all session).
    pub ext_exposure: u16,
    /// Extended gain cap.
    pub ext_gain: u16,
    /// The clip-driven low exposure stage is armed (law V2 only).
    pub clip_enabled: bool,
    /// Low stage starts when the clipped fraction of a pair stays above this ...
    pub clip_frac: f64,
    /// ... for this long (ns), with the exposure already at the normal floor.
    pub clip_ns: u64,
    /// Low stage ends when the exposure no longer needs it (above the normal floor) for this long (ns).
    pub clip_back_ns: u64,
}

impl Default for LightParams {
    fn default() -> Self {
        LightParams {
            enabled: true,
            extend_mean: 20.0,
            extend_ns: 5_000_000_000,
            retract_mean: 80.0,
            settled_mean: 40.0,
            retract_ns: 8_000_000_000,
            low_mean: 15.0,
            low_ns: 10_000_000_000,
            clear_mean: 25.0,
            clear_ns: 3_000_000_000,
            ext_exposure: POWER_ON_EXPOSURE,
            ext_gain: POWER_ON_GAIN,
            clip_enabled: true,
            clip_frac: 0.30,
            clip_ns: 3_000_000_000,
            clip_back_ns: 8_000_000_000,
        }
    }
}

/// A change of the low-light governor, for the driver to log and show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightEvent {
    /// Range extended to `stage` (1 = gain up to the extended cap, 2 = exposure too).
    Extended(u8),
    /// Back to the normal range.
    Retracted,
    /// The cameras see too little light even with the full extended range.
    LowRaised,
    /// Light recovered.
    LowCleared,
}

/// Low-light governor state (pure, driven by host time).
#[derive(Clone, Debug, Default)]
pub struct LightGov {
    /// 0 normal, 1 gain extended, 2 gain and exposure extended.
    pub stage: u8,
    /// "Tracking light low" warning raised.
    pub low: bool,
    starved: Option<u64>,
    easy: Option<u64>,
    dark: Option<u64>,
    clear: Option<u64>,
    last_ns: Option<u64>,
    /// The exposure floor is lowered to [`EXPOSURE_LOW_STAGE`] (clipping window in view).
    pub clip_low: bool,
    clipping: Option<u64>,
    unneeded: Option<u64>,
}

fn held(t: &mut Option<u64>, now_ns: u64, cond: bool, ns: u64) -> bool {
    if !cond {
        *t = None;
        return false;
    }
    let since = *t.get_or_insert(now_ns);
    now_ns.saturating_sub(since) >= ns
}

impl LightGov {
    /// One tracking frame. `at_ceiling`: every controlled pair already uses the caps of the current stage;
    /// `settled`: the loop needs less than half of the normal ceiling; `top`: the highest stage that exists.
    pub fn step(
        &mut self,
        lp: &LightParams,
        now_ns: u64,
        mean: f64,
        at_ceiling: bool,
        settled: bool,
        top: u8,
    ) -> Vec<LightEvent> {
        let mut ev = Vec::new();
        // A gap in the tracking frames (camera restart) makes the timers stale.
        if self.last_ns.is_some_and(|t| now_ns.saturating_sub(t) > 2_000_000_000) {
            self.starved = None;
            self.easy = None;
            self.dark = None;
            self.clear = None;
        }
        self.last_ns = Some(now_ns);
        let starving = self.stage < top && at_ceiling && mean < lp.extend_mean;
        if held(&mut self.starved, now_ns, starving, lp.extend_ns) {
            self.stage += 1;
            self.starved = None;
            ev.push(LightEvent::Extended(self.stage));
        }
        let easy = self.stage > 0 && (mean > lp.retract_mean || (settled && mean >= lp.settled_mean));
        if held(&mut self.easy, now_ns, easy, lp.retract_ns) {
            self.stage = 0;
            self.easy = None;
            self.starved = None;
            ev.push(LightEvent::Retracted);
        }
        // The warning: the full range is in use (or none exists) and the image is still dark.
        let full = self.stage >= top;
        if !self.low {
            if held(&mut self.dark, now_ns, full && at_ceiling && mean < lp.low_mean, lp.low_ns) {
                self.low = true;
                self.dark = None;
                self.clear = None;
                ev.push(LightEvent::LowRaised);
            }
        } else if held(&mut self.clear, now_ns, mean > lp.clear_mean || !full, lp.clear_ns) {
            self.low = false;
            self.clear = None;
            self.dark = None;
            ev.push(LightEvent::LowCleared);
        }
        ev
    }
}

impl LightGov {
    /// The clip-driven low stage. `at_floor`: a pair clips more than `clip_frac` with its exposure at the normal
    /// floor; `free`: every controlled pair is above the normal floor (the low floor is not needed). Returns true
    /// when `clip_low` changed.
    pub fn step_clip(&mut self, lp: &LightParams, now_ns: u64, at_floor: bool, free: bool) -> bool {
        if !lp.clip_enabled {
            return std::mem::replace(&mut self.clip_low, false);
        }
        if !self.clip_low {
            if held(&mut self.clipping, now_ns, at_floor, lp.clip_ns) {
                self.clip_low = true;
                self.clipping = None;
                self.unneeded = None;
                return true;
            }
        } else if held(&mut self.unneeded, now_ns, free, lp.clip_back_ns) {
            self.clip_low = false;
            self.unneeded = None;
            self.clipping = None;
            return true;
        }
        false
    }
}

/// What one update decided.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AeDecision {
    /// Pooled statistics per pair.
    pub stats: [TileStats; 2],
    /// Log error per pair (`ln(target / m)`, after the saturation rule), 0 inside the deadband.
    pub err: [f64; 2],
    /// The target setting changed in this update.
    pub changed: bool,
    /// A command should go out now (period elapsed).
    pub send: bool,
}

/// The controller.
#[derive(Clone, Debug)]
pub struct AutoExposure {
    /// Parameters.
    pub p: AeParams,
    target: Option<AeSetting>,
    last_send_ns: Option<u64>,
    /// Commands handed out for sending.
    pub sends: u64,
    /// Updates that changed the target.
    pub changes: u64,
    /// Tracking frames analysed.
    pub frames: u64,
    /// Low-light governor.
    pub light: LightGov,
    base_exposure_max: u16,
    base_gain_max: u16,
    base_exposure_min: u16,
    events: Vec<LightEvent>,
    last_change_ns: [Option<u64>; 2],
    sat_latch: [bool; 2],
}

impl AutoExposure {
    /// New controller.
    pub fn new(p: AeParams) -> AutoExposure {
        AutoExposure {
            target: None,
            last_send_ns: None,
            sends: 0,
            changes: 0,
            frames: 0,
            light: LightGov::default(),
            base_exposure_max: p.exposure_max,
            base_gain_max: p.gain_max,
            base_exposure_min: p.exposure_min,
            events: Vec::new(),
            last_change_ns: [None; 2],
            sat_latch: [false; 2],
            p,
        }
    }

    /// Highest range stage that exists (0: the normal ceiling is already the extended one, or the feature is off).
    pub fn top_stage(&self) -> u8 {
        let l = &self.p.light;
        if l.enabled && (self.base_exposure_max < l.ext_exposure || self.base_gain_max < l.ext_gain) {
            2
        } else {
            0
        }
    }

    /// (exposure cap, gain cap) of a range stage: 0 normal, 1 gain first, 2 exposure last.
    pub fn caps(&self, stage: u8) -> (u16, u16) {
        let l = &self.p.light;
        match stage {
            0 => (self.base_exposure_max, self.base_gain_max),
            1 => (self.base_exposure_max, self.base_gain_max.max(l.ext_gain)),
            _ => (
                self.base_exposure_max.max(l.ext_exposure),
                self.base_gain_max.max(l.ext_gain),
            ),
        }
    }

    /// Light events since the last call (for the driver to log and show).
    pub fn take_events(&mut self) -> Vec<LightEvent> {
        std::mem::take(&mut self.events)
    }

    /// Feed the governor with one tracking frame and apply the caps of its stage.
    fn govern(&mut self, setting: &AeSetting, stats: &[TileStats; 2], now_ns: u64) {
        let pairs: Vec<usize> = (0..if self.p.upper { 2 } else { 1 }).collect();
        let mean = pairs.iter().map(|&k| stats[k].trimmed_mean).sum::<f64>() / pairs.len() as f64;
        let (ce, cg) = self.caps(self.light.stage);
        let (ne, ng) = self.caps(0);
        let ranges = self.p.model.pair_ranges();
        let at_ceiling = pairs.iter().all(|&k| {
            let c = self.p.pairs[k][0];
            // A model with Windows ranges per pair: the pair's own top (the Crystal's upper pair ends at 8941 / 439).
            let (ce, cg) = match ranges {
                Some(r) => (ce.min(r[k].1), cg.min(r[k].3)),
                None => (ce, cg),
            };
            setting.gain[c] >= cg && setting.exposure[c] as f64 >= 0.97 * ce as f64
        });
        // "settled": every controlled pair needs less than half of the normal ceiling product.
        let half = 0.5 * ne as f64 * ng as f64;
        let settled = pairs.iter().all(|&k| {
            let c = self.p.pairs[k][0];
            (setting.exposure[c] as f64 * setting.gain[c] as f64) <= half
        });
        let top = self.top_stage();
        let before = self.light.stage;
        let ev = self.light.step(&self.p.light, now_ns, mean, at_ceiling, settled, top);
        if self.light.stage != before {
            let (e, g) = self.caps(self.light.stage);
            self.p.exposure_max = e;
            self.p.gain_max = g;
        }
        self.events.extend(ev);
        // The low exposure floor (law V2): a window in view that clips more than 30 % for 3 s at the normal floor.
        let floor = self.base_exposure_min as f64 * 1.05;
        let at_floor = pairs.iter().any(|&k| {
            stats[k].sat_frac > self.p.light.clip_frac && (setting.exposure[self.p.pairs[k][0]] as f64) <= floor
        });
        let free = pairs.iter().all(|&k| setting.exposure[self.p.pairs[k][0]] as f64 > floor);
        let lp = self.p.light.clone();
        if self.light.step_clip(&lp, now_ns, at_floor, free) {
            self.p.exposure_min = if self.light.clip_low {
                EXPOSURE_LOW_STAGE.min(self.base_exposure_min)
            } else {
                self.base_exposure_min
            };
        }
    }

    /// Current target (None before the first frame).
    pub fn target(&self) -> Option<AeSetting> {
        self.target
    }

    /// Log-domain error of one pair from its pooled statistics (0 inside the deadband).
    pub fn error(&self, s: &TileStats) -> f64 {
        if s.n == 0 {
            return 0.0;
        }
        let m = s.trimmed_mean.max(1.0);
        let mut err = (self.p.target / m).ln();
        if s.sat_frac > self.p.sat_max_frac {
            err = err.min(-self.p.sat_pull);
        }
        if err.abs() < self.p.deadband {
            0.0
        } else {
            err
        }
    }

    /// New (exposure, gain) for a pair whose frame was taken with `(e, g)` and shows statistics `s`.
    pub fn step_pair(&self, e: u16, g: u16, s: &TileStats) -> (u16, u16, f64) {
        self.step_pair_with(0, e, g, s, false)
    }

    /// [`AutoExposure::step_pair`] for pair `k` (0 lower, 1 upper; the caps of that pair).
    pub fn step_pair_k(&self, k: usize, e: u16, g: u16, s: &TileStats) -> (u16, u16, f64) {
        self.step_pair_with(k, e, g, s, false)
    }

    /// [`AutoExposure::step_pair`]; `no_brighter` (the too-bright rule is latched) turns a brighten request into none.
    fn step_pair_with(&self, k: usize, e: u16, g: u16, s: &TileStats, no_brighter: bool) -> (u16, u16, f64) {
        let mut err = self.error(s);
        if no_brighter && err > 0.0 {
            err = 0.0;
        }
        if err == 0.0 {
            return (e, g, 0.0);
        }
        let far =
            s.sat_frac > 0.5 || s.dark_frac > 0.8 || s.trimmed_mean > 240.0 || s.trimmed_mean < 8.0;
        let lim = if far {
            self.p.max_step_far
        } else {
            self.p.max_step
        }
        .max(1.0)
        .ln();
        let r = (self.p.k * err).clamp(-lim, lim).exp();
        let base = e.max(1) as f64 * g.max(1) as f64;
        let (ne, ng) = self.p.split_pair(k, base * r);
        let new = ne as f64 * ng as f64;
        // The caps can make the split darker than the frame's own setting (e.g. the power-on 10410/449 in a dark
        // room): never step against the error.
        if (err > 0.0 && new < base) || (err < 0.0 && new > base) {
            return (e, g, err);
        }
        (ne, ng, err)
    }

    /// One tracking frame: `setting` = what the frame was taken with (its metadata), `stats` = pooled statistics
    /// of each pair, `now_ns` = host time. Returns the decision; the setting to send is [`AutoExposure::target`].
    pub fn update(
        &mut self,
        setting: &AeSetting,
        stats: [TileStats; 2],
        now_ns: u64,
    ) -> AeDecision {
        self.frames += 1;
        self.govern(setting, &stats, now_ns);
        let prev = self.target;
        let mut next = prev.unwrap_or(AeSetting {
            led_exposure: self.p.led_exposure,
            led_gain: self.p.led_gain,
            ..*setting
        });
        let mut errs = [0.0; 2];
        for (k, pair) in self.p.pairs.iter().enumerate() {
            if k == 1 && !self.p.upper {
                continue;
            }
            let [a, b] = *pair;
            // The pair runs on one setting; take the frame's value of the first camera.
            if self.p.law == AeLaw::V2 {
                // Hysteresis of the too-bright rule: once it fired, no brightening until the clip is below the
                // release level (no limit cycle at the edge of the threshold).
                if stats[k].sat_frac > self.p.sat_max_frac {
                    self.sat_latch[k] = true;
                } else if stats[k].sat_frac < self.p.sat_release {
                    self.sat_latch[k] = false;
                }
            }
            let latched = self.sat_latch[k];
            let (e, g, mut err) =
                self.step_pair_with(k, setting.exposure[a], setting.gain[a], &stats[k], latched);
            // A model with Windows ranges per pair (Crystal Super): the result always lies inside them, even when the
            // frame's own setting (the firmware default) or a no-step answer does not.
            let (e, g) = self.p.fit_pair(k, e, g);
            // Rate limit: at most one change per pair per `min_change_ns` (about 5 changes/s). The frames in
            // between were taken with the old setting and must not be acted on twice.
            if err != 0.0 && prev.is_some() {
                if self.last_change_ns[k].is_some_and(|t| now_ns.saturating_sub(t) < self.p.min_change_ns) {
                    err = 0.0;
                } else {
                    self.last_change_ns[k] = Some(now_ns);
                }
            }
            errs[k] = err;
            if err != 0.0 || prev.is_none() {
                // Outside the deadband (or first frame): the new value is relative to what this frame used.
                next.exposure[a] = e;
                next.exposure[b] = e;
                next.gain[a] = g;
                next.gain[b] = g;
            }
        }
        if self.p.model.pair_ranges().is_some() {
            // Also a pair that is not controlled (`upper` off) goes out inside Windows' ranges, as one equal pair.
            for (k, &[a, b]) in self.p.pairs.iter().enumerate() {
                let (e, g) = self.p.fit_pair(k, next.exposure[a], next.gain[a]);
                next.exposure[a] = e;
                next.exposure[b] = e;
                next.gain[a] = g;
                next.gain[b] = g;
            }
        }
        next.led_exposure = self.p.led_exposure;
        next.led_gain = self.p.led_gain;
        let changed = prev != Some(next);
        if changed {
            self.changes += 1;
        }
        self.target = Some(next);
        let due = match self.last_send_ns {
            None => true,
            Some(t) => now_ns.saturating_sub(t) >= self.p.period_ns,
        };
        let send = due || (changed && prev.is_none());
        if send {
            self.last_send_ns = Some(now_ns);
            self.sends += 1;
        }
        AeDecision {
            stats,
            err: errs,
            changed,
            send,
        }
    }

    /// Pool the tiles of each pair of a tracking frame into one histogram per pair.
    pub fn frame_stats(&self, tiles: &[TileView; 4], mask: &Mask) -> [TileStats; 2] {
        let mut out = [TileStats::default(); 2];
        for (k, pair) in self.p.pairs.iter().enumerate() {
            if self.p.law == AeLaw::V2 {
                // Masked mean of the unclipped cells is the AE input (in `trimmed_mean`); the rest describes
                // the whole masked view.
                let t: Vec<TileView> = pair.iter().map(|&i| tiles[i]).collect();
                let (m, all, _) = scene_metric_hist(&t, self.p.step, mask, self.p.sat_level);
                let mut st = all.stats(self.p.trim, self.p.sat_level);
                st.trimmed_mean = m.mean;
                st.sat_frac = m.sat;
                st.dark_frac = m.dark;
                out[k] = st;
                continue;
            }
            let mut h = Histogram::default();
            for &t in pair {
                h.merge(&tile_histogram(&tiles[t], self.p.step, mask));
            }
            out[k] = h.stats(self.p.trim, self.p.sat_level);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_matches_the_windows_frame() {
        // camera-frames.md: 58 19 51 00 01 32 47 0c 47 0c 57 08 57 08 0d 00 0d 00 0d 00 0d 00 28 00 3e 00
        let s = AeSetting {
            exposure: [0x0c47, 0x0c47, 0x0857, 0x0857],
            gain: [13; 4],
            led_exposure: 40,
            led_gain: 62,
        };
        let p = payload(&s);
        let want = [
            0x01, 0x32, 0x47, 0x0c, 0x47, 0x0c, 0x57, 0x08, 0x57, 0x08, 0x0d, 0x00, 0x0d, 0x00,
            0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00,
        ];
        assert_eq!(p, want);
        let mut report = vec![0x58, 0x19, 0x51, 0x00];
        report.extend_from_slice(&p);
        assert_eq!(parse_payload(&report), Some(s));
        assert_eq!(parse_payload(&p), Some(s));
        assert_eq!(parse_payload(&[0x58, 0x04, 0x39, 0x00, 0x48]), None);
    }

    #[test]
    fn histogram_stats() {
        let mut h = Histogram::default();
        h.bins[10] = 50;
        h.bins[100] = 48;
        h.bins[255] = 2;
        let s = h.stats(0.98, 250);
        assert_eq!(s.n, 100);
        assert!((s.mean - (500.0 + 4800.0 + 510.0) / 100.0).abs() < 1e-9);
        // The two saturated pixels sit above the 98th percentile: trimmed mean leaves them out.
        assert!((s.trimmed_mean - (500.0 + 4800.0) / 98.0).abs() < 1e-9);
        assert_eq!(s.p50, 10.0);
        assert!((s.sat_frac - 0.02).abs() < 1e-12);
        assert_eq!(h.quantile(0.51), 100.0);
    }

    #[test]
    fn split_prefers_exposure() {
        let p = AeParams::default();
        // On the ladder at gain 13 (2200 and 3143 are Windows values).
        assert_eq!(p.split(2200.0 * 13.0), (2200, 13));
        assert_eq!(p.split(3150.0 * 13.0), (3143, 13));
        // Above the exposure cap: smallest gain code that reaches the product, exposure below the cap.
        let (e, g) = p.split(4500.0 * 13.0);
        assert_eq!(g, 15);
        assert!(
            e <= p.exposure_max
                && (e as f64 * 15.0 - 4500.0 * 13.0).abs() / (4500.0 * 13.0) < 0.035
        );
        // Clamped at both ends.
        assert_eq!(p.split(1.0), (p.exposure_min, p.gain_min));
        assert_eq!(p.split(1e12), (p.exposure_max, p.gain_max));
        // Without quantisation: plain exposure-first split.
        let q = AeParams {
            quantize: false,
            ..AeParams::default()
        };
        assert_eq!(q.split(2000.0 * 13.0), (2000, 13));
        // The ladder reproduces Windows' observed values.
        for w in [
            1682.0, 1733.0, 1785.0, 2011.0, 2478.0, 3051.0, 3988.0, 4108.0,
        ] {
            assert_eq!(ladder(w), w);
        }
    }

    /// A simulated sensor: pixel = clamp(radiance * e * g * c), with a textured scene and a bright light blob.
    fn render(radiance: f64, e: u16, g: u16, blob: bool) -> Vec<u8> {
        let mut img = vec![0u8; 640 * 480];
        let c = 1.0 / (2000.0 * 13.0);
        for y in 0..480 {
            for x in 0..640 {
                let tex =
                    0.6 + 0.4 * (((x / 16 + y / 16) % 2) as f64) + 0.1 * ((x as f64 * 0.1).sin());
                let mut v = radiance * tex * e as f64 * g as f64 * c;
                if blob && (x as i32 - 500).pow(2) + (y as i32 - 100).pow(2) < 900 {
                    v = 1e4;
                }
                img[y * 640 + x] = v.round().clamp(0.0, 255.0) as u8;
            }
        }
        img
    }

    fn view(d: &[u8]) -> TileView<'_> {
        TileView {
            data: d,
            stride: 640,
            x0: 0,
            y0: 0,
            w: 640,
            h: 480,
        }
    }

    fn stats_of(ae: &AutoExposure, img0: &[u8], img1: &[u8]) -> [TileStats; 2] {
        let tiles = [view(img0), view(img1), view(img0), view(img1)];
        ae.frame_stats(&tiles, &Mask::default())
    }

    /// Closed loop with a 2-frame application latency (the camera applies a command two tracking frames later),
    /// starting at the Linux power-on default in a bright room: converges into the deadband without
    /// oscillating, keeps the pair equal and stays within the limits.
    #[test]
    fn converges_from_power_on_with_latency() {
        for &(radiance, blob) in &[(90.0, true), (30.0, false), (300.0, false), (6.0, false)] {
            let mut ae = AutoExposure::new(AeParams::default());
            let mut applied = AeSetting {
                exposure: [POWER_ON_EXPOSURE; 4],
                gain: [POWER_ON_GAIN; 4],
                led_exposure: 40,
                led_gain: 62,
            };
            // Measured on Windows: a command sent after tracking frame i is in effect from tracking frame i + 2.
            let mut pipeline: Vec<(usize, AeSetting)> = Vec::new();
            let mut means = Vec::new();
            let mut t = 0u64;
            for i in 0..150usize {
                while pipeline.first().is_some_and(|(f, _)| *f <= i) {
                    applied = pipeline.remove(0).1;
                }
                let img = render(radiance, applied.exposure[0], applied.gain[0], blob);
                let st = stats_of(&ae, &img, &img);
                means.push(st[0].trimmed_mean);
                let d = ae.update(&applied, st, t);
                if d.send {
                    pipeline.push((i + 2, ae.target().unwrap()));
                }
                let s = ae.target().unwrap();
                assert_eq!(s.exposure[0], s.exposure[1]);
                assert_eq!(s.gain[2], s.gain[3]);
                assert!(s.exposure[0] <= ae.p.exposure_max && s.gain[0] <= ae.p.gain_max);
                t += 40_000_000;
            }
            // Too dark even for the caps: keeps the brighter power-on setting or ends at the caps.
            let at_caps = (applied.exposure[0] == ae.p.exposure_max
                && applied.gain[0] == ae.p.gain_max)
                || (applied.exposure[0] == ae.p.exposure_min && applied.gain[0] == ae.p.gain_min)
                || applied.exposure[0] == POWER_ON_EXPOSURE;
            let in_band = |m: f64| (m / ae.p.target).ln().abs() < 0.25;
            // Settles within 80 tracking frames (3.2 s) from the power-on default (or ends at the caps when the
            // scene is too dark for any setting).
            let settled = means.iter().position(|&m| in_band(m));
            eprintln!(
                "radiance {radiance}: settled after {settled:?} tracking frames, final {:?}",
                applied
            );
            assert!(
                at_caps || settled.is_some_and(|i| i < 80),
                "radiance {radiance}: {means:?}"
            );
            // Then steady: no limit cycle in the last 20 frames.
            let tail = &means[means.len() - 20..];
            let (lo, hi) = tail
                .iter()
                .fold((f64::MAX, f64::MIN), |(a, b), &m| (a.min(m), b.max(m)));
            assert!(hi - lo < 1.0, "radiance {radiance}: not steady {tail:?}");
            assert!(at_caps || in_band(*tail.last().unwrap()));
        }
    }

    #[test]
    fn deadband_and_period() {
        let mut ae = AutoExposure::new(AeParams::default());
        let s = AeSetting {
            exposure: [2000; 4],
            gain: [13; 4],
            led_exposure: 40,
            led_gain: 62,
        };
        let st = TileStats {
            trimmed_mean: 66.0,
            n: 1000,
            ..Default::default()
        };
        let d = ae.update(&s, [st, st], 0);
        assert!(d.send); // first update always sends
        assert_eq!(ae.target().unwrap().exposure, [2000; 4]);
        let d = ae.update(&s, [st, st], 40_000_000);
        assert!(!d.send && !d.changed);
        let d = ae.update(&s, [st, st], 80_000_000);
        assert!(d.send && !d.changed); // periodic push like Windows
                                       // A darker scene raises the exposure of both pairs.
        let dark = TileStats {
            trimmed_mean: 35.0,
            n: 1000,
            ..Default::default()
        };
        let d = ae.update(&s, [dark, st], 120_000_000);
        assert!(d.changed && d.err[0] > 0.0 && d.err[1] == 0.0);
        let t = ae.target().unwrap();
        assert!(t.exposure[0] > 2000 && t.exposure[0] == t.exposure[1]);
        assert_eq!(t.exposure[2], 2000);
        // Mostly saturated with an acceptable trimmed mean still pulls down.
        let sat = TileStats {
            trimmed_mean: 95.0,
            sat_frac: 0.2,
            n: 1000,
            ..Default::default()
        };
        let (e, _, err) = ae.step_pair(2000, 13, &sat);
        assert!(err < 0.0 && e < 2000);
        // Too dark even at the power-on setting: keep it (the exposure cap would make it darker).
        let black = TileStats {
            trimmed_mean: 7.0,
            n: 1000,
            ..Default::default()
        };
        assert_eq!(
            ae.step_pair(POWER_ON_EXPOSURE, POWER_ON_GAIN, &black).0,
            POWER_ON_EXPOSURE
        );
        // Bright at the power-on setting: exposure drops to the cap first, gain carries the rest.
        let bright = TileStats {
            trimmed_mean: 240.0,
            sat_frac: 0.6,
            n: 1000,
            ..Default::default()
        };
        let (e, g, _) = ae.step_pair(POWER_ON_EXPOSURE, POWER_ON_GAIN, &bright);
        assert!(
            e <= ae.p.exposure_max
                && (e as f64 * g as f64) < POWER_ON_EXPOSURE as f64 * POWER_ON_GAIN as f64
        );
    }

    // ---- low-light governor: extended range and the "tracking light low" warning ----

    const S: u64 = 1_000_000_000;

    /// Closed-loop run in a scene of constant `radiance` (2-frame latency, 40 ms tracking period) starting from
    /// frame `from` of `ae`'s clock; returns the applied setting and checks the invariants on every frame.
    fn run_scene(
        ae: &mut AutoExposure,
        applied: &mut AeSetting,
        pipeline: &mut Vec<(usize, AeSetting)>,
        radiance: f64,
        frames: usize,
        t: &mut u64,
        events: &mut Vec<(u64, LightEvent)>,
    ) {
        // Commands still in flight from the previous call land at once.
        if let Some((_, last)) = pipeline.pop() {
            *applied = last;
            pipeline.clear();
        }
        for i in 0..frames {
            while pipeline.first().is_some_and(|(f, _)| *f <= i) {
                *applied = pipeline.remove(0).1;
            }
            let img = render(radiance, applied.exposure[0], applied.gain[0], false);
            let st = stats_of(ae, &img, &img);
            let d = ae.update(applied, st, *t);
            for ev in ae.take_events() {
                events.push((*t, ev));
            }
            if d.send {
                pipeline.push((i + 2, ae.target().unwrap()));
            }
            let s = ae.target().unwrap();
            // The LED frames are never touched.
            assert_eq!((s.led_exposure, s.led_gain), (40, 62));
            // Gain before exposure: above the normal exposure ceiling the gain is at its cap.
            if s.exposure[0] > 4108 {
                assert_eq!(s.gain[0], POWER_ON_GAIN, "exposure {} with gain {}", s.exposure[0], s.gain[0]);
            }
            // The exposure stays far below the tracking frame period (40 ms at 25 Hz).
            assert!((s.exposure[0] as u64) * 1000 < 40_000_000);
            assert!(s.exposure[0] <= 10410 && s.gain[0] <= 449);
            *t += 40_000_000;
        }
    }

    fn power_on() -> AeSetting {
        AeSetting {
            exposure: [POWER_ON_EXPOSURE; 4],
            gain: [POWER_ON_GAIN; 4],
            led_exposure: 40,
            led_gain: 62,
        }
    }

    #[test]
    fn extends_gain_first_then_exposure_and_retracts() {
        let mut ae = AutoExposure::new(AeParams::default());
        // Start at the normal ceiling in a room too dark for it (mean ~8 there).
        let mut applied = AeSetting {
            exposure: [4108; 4],
            gain: [215; 4],
            led_exposure: 40,
            led_gain: 62,
        };
        let (mut pipe, mut ev, mut t) = (Vec::new(), Vec::new(), 0u64);
        run_scene(&mut ae, &mut applied, &mut pipe, 0.3, 900, &mut t, &mut ev);
        let ext: Vec<(u64, LightEvent)> = ev.iter().copied().filter(|e| matches!(e.1, LightEvent::Extended(_))).collect();
        assert_eq!(ext.len(), 2, "{ev:?}");
        assert_eq!(ext[0].1, LightEvent::Extended(1));
        assert_eq!(ext[1].1, LightEvent::Extended(2));
        // Not before 5 s at the ceiling, and the second step needs another 5 s at the stage-1 ceiling.
        assert!(ext[0].0 >= 5 * S && ext[0].0 < 8 * S, "{ext:?}");
        assert!(ext[1].0 - ext[0].0 >= 5 * S, "{ext:?}");
        assert_eq!(ae.light.stage, 2);
        assert_eq!((ae.p.exposure_max, ae.p.gain_max), (10410, 449));
        // Stage 1 (gain only) reached the gain cap before any exposure above 4108 was needed.
        assert!(applied.exposure[0] > 4108 || applied.gain[0] == 449);
        // The scene is bright enough with the extended range (mean ~45): no warning.
        assert!(!ae.light.low && !ev.iter().any(|e| e.1 == LightEvent::LowRaised));

        // Light comes back (lamp on): the loop settles low and after 8 s the normal range returns.
        run_scene(&mut ae, &mut applied, &mut pipe, 30.0, 600, &mut t, &mut ev);
        assert_eq!(ae.light.stage, 0, "{ev:?}");
        assert!(ev.iter().any(|e| e.1 == LightEvent::Retracted));
        assert_eq!((ae.p.exposure_max, ae.p.gain_max), (4108, 215));
        assert!(applied.exposure[0] <= 4108 && applied.gain[0] <= 215, "{applied:?}");
    }

    #[test]
    fn gain_goes_to_the_extended_cap_before_exposure_passes_the_normal_one() {
        let p = AeParams {
            exposure_max: 10410,
            gain_max: 449,
            ..AeParams::default()
        };
        // Products between the normal ceiling and 449 x 4108: gain 449, exposure below 4108.
        let (e, g) = p.split(4108.0 * 300.0);
        assert_eq!(g, 449);
        assert!(e <= 4108 && (e as f64 * 449.0 - 4108.0 * 300.0).abs() / (4108.0 * 300.0) < 0.035);
        // Normal-range products ignore the extended exposure (no blur for no reason).
        let (e, g) = p.split(3000.0 * 13.0);
        assert_eq!((e, g), (ladder(3000.0) as u16, 13));
        let (e, g) = p.split(4108.0 * 215.0);
        assert!(e <= 4108 && g <= 449);
        // Beyond 449 x 4108 the exposure grows, up to 10410.
        let (e, g) = p.split(6000.0 * 449.0);
        assert_eq!(g, 449);
        assert!(e > 4108 && e <= 10410);
        assert_eq!(p.split(1e12), (10410, 449));
    }

    /// Feed the governor `secs` seconds of 25 Hz tracking frames with a constant state.
    #[allow(clippy::too_many_arguments)]
    fn tick(
        lp: &LightParams,
        g: &mut LightGov,
        t: &mut u64,
        ev: &mut Vec<LightEvent>,
        secs: f64,
        mean: f64,
        at: bool,
        settled: bool,
    ) {
        for _ in 0..(secs * 25.0).round() as u64 {
            ev.extend(g.step(lp, *t, mean, at, settled, 2));
            *t += 40_000_000;
        }
    }

    #[test]
    fn governor_timers_and_hysteresis() {
        let lp = LightParams::default();
        let mut g = LightGov::default();
        let mut t = 0u64;
        let mut all = Vec::new();
        // 4 s dark at the ceiling: not yet.
        tick(&lp, &mut g, &mut t, &mut all, 4.0, 10.0, true, false);
        assert_eq!(g.stage, 0);
        // Not at the ceiling (the loop still has room): the timer does not run.
        tick(&lp, &mut g, &mut t, &mut all, 10.0, 10.0, false, false);
        assert_eq!(g.stage, 0);
        tick(&lp, &mut g, &mut t, &mut all, 6.0, 10.0, true, false);
        assert_eq!(g.stage, 1);
        tick(&lp, &mut g, &mut t, &mut all, 6.0, 10.0, true, false);
        assert_eq!(g.stage, 2);
        assert_eq!(all, vec![LightEvent::Extended(1), LightEvent::Extended(2)]);
        // A short bright spell does not retract; a long one does.
        tick(&lp, &mut g, &mut t, &mut all, 5.0, 90.0, false, false);
        tick(&lp, &mut g, &mut t, &mut all, 1.0, 30.0, false, false);
        assert_eq!(g.stage, 2);
        tick(&lp, &mut g, &mut t, &mut all, 9.0, 90.0, false, false);
        assert_eq!(g.stage, 0);
        assert_eq!(all.last(), Some(&LightEvent::Retracted));
        // A gap in the frames (> 2 s) resets a running timer.
        let mut g = LightGov::default();
        let mut t = 0u64;
        tick(&lp, &mut g, &mut t, &mut all, 4.0, 10.0, true, false);
        t += 3 * S;
        tick(&lp, &mut g, &mut t, &mut all, 4.0, 10.0, true, false);
        assert_eq!(g.stage, 0);
    }

    #[test]
    fn low_light_warning_raises_and_clears() {
        let lp = LightParams::default();
        let mut g = LightGov::default();
        g.stage = 2;
        let mut t = 0u64;
        let mut ev = Vec::new();
        // 9 s at mean 12 with the full range at its ceiling: not yet; 10 s: raised.
        tick(&lp, &mut g, &mut t, &mut ev, 9.0, 12.0, true, false);
        assert!(!g.low);
        tick(&lp, &mut g, &mut t, &mut ev, 1.5, 12.0, true, false);
        assert!(g.low);
        // Mean 20: above the raise level but not above the clear level (25): stays raised.
        tick(&lp, &mut g, &mut t, &mut ev, 10.0, 20.0, true, false);
        assert!(g.low);
        // Light recovers (> 25 for 3 s): cleared.
        tick(&lp, &mut g, &mut t, &mut ev, 2.0, 40.0, true, false);
        assert!(g.low);
        tick(&lp, &mut g, &mut t, &mut ev, 1.5, 40.0, true, false);
        assert!(!g.low);
        assert_eq!(ev, vec![LightEvent::LowRaised, LightEvent::LowCleared]);
        // Dark but the loop is not at its ceiling (still adjusting): no warning.
        let mut g = LightGov { stage: 2, ..LightGov::default() };
        ev.clear();
        tick(&lp, &mut g, &mut t, &mut ev, 20.0, 5.0, false, false);
        assert!(!g.low);
        // Without an extended range (feature off, top stage 0) the warning still works on the normal ceiling.
        let mut g = LightGov::default();
        for _ in 0..(11 * 25) {
            ev.extend(g.step(&lp, t, 8.0, true, false, 0));
            t += 40_000_000;
        }
        assert!(g.low && g.stage == 0);
    }

    #[test]
    fn closed_loop_warning_in_a_very_dark_room_and_clear_when_lit() {
        let mut ae = AutoExposure::new(AeParams::default());
        let mut applied = power_on();
        let (mut pipe, mut ev, mut t) = (Vec::new(), Vec::new(), 0u64);
        // Radiance so low that even 10410 us at gain 449 gives a mean below 15.
        run_scene(&mut ae, &mut applied, &mut pipe, 0.08, 1100, &mut t, &mut ev);
        assert_eq!(ae.light.stage, 2);
        assert!(ae.light.low, "{ev:?}");
        assert!(ev.iter().any(|e| e.1 == LightEvent::LowRaised));
        let raised = ev.iter().find(|e| e.1 == LightEvent::LowRaised).unwrap().0;
        // After the extension (2 x 5 s from the start) plus 10 s dark: not earlier than 10 s.
        assert!(raised >= 10 * S);
        // A lamp is switched on: the warning clears.
        run_scene(&mut ae, &mut applied, &mut pipe, 20.0, 400, &mut t, &mut ev);
        assert!(!ae.light.low, "{ev:?}");
        assert!(ev.iter().any(|e| e.1 == LightEvent::LowCleared));
    }

    #[test]
    fn extension_is_off_with_the_env_switch_or_when_the_caps_are_already_the_largest() {
        let off = AeParams {
            light: LightParams { enabled: false, ..LightParams::default() },
            ..AeParams::default()
        };
        assert_eq!(AutoExposure::new(off).top_stage(), 0);
        let big = AeParams {
            exposure_max: 10410,
            gain_max: 449,
            ..AeParams::default()
        };
        assert_eq!(AutoExposure::new(big).top_stage(), 0);
        assert_eq!(AutoExposure::new(AeParams::default()).top_stage(), 2);
        // Normal caps are Windows' 4108 / 215 and the extended ones the power-on values.
        let ae = AutoExposure::new(AeParams::default());
        assert_eq!(ae.caps(0), (4108, 215));
        assert_eq!(ae.caps(1), (4108, 449));
        assert_eq!(ae.caps(2), (10410, 449));
    }

    // ---- law V2: window scene, dark room, steady state, floors, rate limit, shared metric, good frame ----

    /// The room (`radiance`) with a window (20x brighter) over the right `frac` of the width, black lens corners.
    fn render_window(radiance: f64, e: u16, g: u16, frac: f64) -> Vec<u8> {
        let mut img = render(radiance, e, g, false);
        let x0 = (640.0 * (1.0 - frac)) as usize;
        let c = 1.0 / (2000.0 * 13.0);
        for y in 0..480 {
            for x in x0..640 {
                img[y * 640 + x] = (20.0 * radiance * 0.8 * e as f64 * g as f64 * c).round().clamp(0.0, 255.0) as u8;
            }
        }
        img
    }

    /// Closed loop (2-frame latency, 25 Hz tracking). Returns per-second change counts of the target, the
    /// per-frame (exposure, gain, room mean, sat) and the controller.
    fn run_law(
        p: AeParams,
        secs: usize,
        scene: &dyn Fn(u16, u16) -> Vec<u8>,
        start: AeSetting,
    ) -> (Vec<u32>, Vec<(u16, u16, f64, f64)>, AutoExposure) {
        let mut ae = AutoExposure::new(p);
        let mut applied = start;
        let mut pipe: Vec<(usize, AeSetting)> = Vec::new();
        let (mut per_s, mut log) = (vec![0u32; secs], Vec::new());
        let mut t = 0u64;
        let mut prev = None;
        for i in 0..secs * 25 {
            while pipe.first().is_some_and(|(f, _)| *f <= i) {
                applied = pipe.remove(0).1;
            }
            let img = scene(applied.exposure[0], applied.gain[0]);
            let st = stats_of(&ae, &img, &img);
            let d = ae.update(&applied, st, t);
            if d.send {
                pipe.push((i + 2, ae.target().unwrap()));
            }
            let tg = ae.target().unwrap();
            if prev.is_some_and(|p| p != tg) {
                per_s[i / 25] += 1;
            }
            prev = Some(tg);
            log.push((applied.exposure[0], applied.gain[0], st[0].trimmed_mean, st[0].sat_frac));
            t += 40_000_000;
        }
        (per_s, log, ae)
    }

    fn start_bright() -> AeSetting {
        AeSetting { exposure: [2000; 4], gain: [13; 4], led_exposure: 40, led_gain: 62 }
    }

    #[test]
    fn window_scene_is_steady_at_or_above_the_floor() {
        // 22 % of the view is a window: above the 20 % rule but below the 30 % low stage.
        let scene = |e, g| render_window(80.0, e, g, 0.22);
        let (per_s, log, ae) = run_law(AeParams::default(), 40, &scene, start_bright());
        let tail = &per_s[15..];
        assert!(tail.iter().all(|&c| c <= 2), "changes/s in the tail {per_s:?}");
        assert!(log.iter().all(|l| l.0 >= 1140), "never below the floor: {:?}", log.iter().map(|l| l.0).min());
        assert!(per_s.iter().all(|&c| c <= 5), "rate limit {per_s:?}");
        assert!(!ae.light.clip_low, "22 % clipped does not open the low stage");
        // The room (unclipped cells) is not starved: the metered mean stays well above the old 22-39 range.
        let m = log.last().unwrap().2;
        assert!(m > 40.0, "room mean {m}");
        // The old law on the same scene drives the room dark (the sunny-room failure).
        let (_, old, _) = run_law(AeParams::old_law(), 40, &scene, start_bright());
        assert!(old.last().unwrap().0 < 1140 || old.last().unwrap().2 < m, "old {:?} new {:?}", old.last(), log.last());
    }

    #[test]
    fn big_window_opens_the_low_stage_after_three_seconds_and_never_goes_to_100() {
        let scene = |e, g| render_window(80.0, e, g, 0.45);
        let (per_s, log, ae) = run_law(AeParams::default(), 40, &scene, start_bright());
        assert!(ae.light.clip_low, "45 % clipped for long: low stage");
        let lowest = log.iter().map(|l| l.0).min().unwrap();
        assert!((300..1140).contains(&lowest), "lowest exposure {lowest}");
        // Before 3 s at the floor, nothing below it.
        let first_low = log.iter().position(|l| l.0 < 1140).unwrap();
        let first_floor = log.iter().position(|l| l.0 <= 1197).unwrap();
        assert!(first_low >= first_floor + 3 * 25, "low stage too early: floor at {first_floor}, low at {first_low}");
        assert!(per_s[20..].iter().all(|&c| c <= 2), "{per_s:?}");
    }

    #[test]
    fn low_stage_returns_when_the_light_changes() {
        let mut ae = AutoExposure::new(AeParams::default());
        let bright = |e, g| render_window(80.0, e, g, 0.45);
        let mut applied = start_bright();
        let mut t = 0u64;
        let mut pipe: Vec<(usize, AeSetting)> = Vec::new();
        for i in 0..1000usize {
            while pipe.first().is_some_and(|(f, _)| *f <= i) {
                applied = pipe.remove(0).1;
            }
            // First 20 s window, then a plain dim room.
            let img = if i < 500 { bright(applied.exposure[0], applied.gain[0]) } else { render(5.0, applied.exposure[0], applied.gain[0], false) };
            let st = stats_of(&ae, &img, &img);
            if ae.update(&applied, st, t).send {
                pipe.push((i + 2, ae.target().unwrap()));
            }
            if i == 499 {
                assert!(ae.light.clip_low);
            }
            t += 40_000_000;
        }
        assert!(!ae.light.clip_low);
        assert_eq!(ae.p.exposure_min, 1140);
    }

    #[test]
    fn dark_room_goes_up_without_oscillation() {
        let scene = |e, g| render(6.0, e, g, false);
        let (per_s, log, _) = run_law(AeParams::default(), 40, &scene, start_bright());
        assert!(per_s[20..].iter().all(|&c| c <= 1), "{per_s:?}");
        let (e, g, m, _) = *log.last().unwrap();
        assert!(e as f64 * g as f64 > 2000.0 * 13.0 * 2.0 && m > 30.0, "{e}/{g} mean {m}");
    }

    #[test]
    fn steady_room_does_not_move_and_floor_holds() {
        // A bright but unclipped room (mean in the deadband at the start): no change at all in 20 s.
        let scene = |e, g| render(80.0, e, g, false);
        let (per_s, _, ae) = run_law(AeParams::default(), 30, &scene, AeSetting { exposure: [2000; 4], gain: [13; 4], led_exposure: 40, led_gain: 62 });
        assert!(per_s[10..].iter().all(|&c| c == 0), "{per_s:?}");
        let t = ae.target().unwrap();
        assert_eq!((t.led_exposure, t.led_gain), (40, 62));
        // A very bright, fully clipped room: exposure stops at the floor (normal stage) and never reaches 100.
        let p = AeParams::default();
        assert_eq!(p.exposure_min, 1140);
        assert_eq!(p.split(1.0).0, 1140);
        assert!(p.sat_max_frac == 0.20 && p.deadband >= 0.15);
        let mut ae = AutoExposure::new(p);
        let allsat = TileStats { trimmed_mean: 250.0, sat_frac: 0.9, n: 1000, ..Default::default() };
        let mut s = start_bright();
        for i in 0..40u64 {
            ae.update(&s, [allsat, allsat], i * 300_000_000);
            s = ae.target().unwrap();
            assert!(s.exposure[0] >= 1140 || ae.light.clip_low, "{s:?}");
            assert!(s.exposure[0] >= 300);
        }
    }

    #[test]
    fn too_bright_rule_only_above_twenty_percent_and_rate_limited() {
        let ae = AutoExposure::new(AeParams::default());
        let ok = TileStats { trimmed_mean: 64.0, sat_frac: 0.19, n: 1000, ..Default::default() };
        assert_eq!(ae.error(&ok), 0.0);
        let bad = TileStats { sat_frac: 0.21, ..ok };
        assert!(ae.error(&bad) < 0.0);
        // Rate limit: a dark scene asks for a change on every frame; the target moves at most 5 times per second.
        let mut ae = AutoExposure::new(AeParams::default());
        let dark = TileStats { trimmed_mean: 20.0, n: 1000, ..Default::default() };
        let mut s = start_bright();
        let mut changes = 0;
        let mut last = ae.target();
        for i in 0..100u64 {
            ae.update(&s, [dark, dark], i * 40_000_000);
            let t = ae.target();
            if i > 0 && t != last {
                changes += 1;
            }
            last = t;
            s = t.unwrap();
        }
        assert!(changes <= 5 * 4 + 1, "changes in 4 s: {changes}");
        assert!(changes >= 10, "still follows the scene: {changes}");
    }

    #[test]
    fn old_law_stays_reachable() {
        let o = AeParams::old_law();
        assert_eq!((o.exposure_min, o.sat_max_frac, o.deadband, o.law, o.min_change_ns), (100, 0.05, 0.08, AeLaw::V1, 0));
        assert_eq!(AeParams::default().law, AeLaw::V2);
        // PIMAX_CAM_AE_LAW is read in from_env (set/unset only inside this test; the env is process wide, so the
        // other tests never read it: they use AeParams::default()).
        std::env::set_var("PIMAX_CAM_AE_LAW", "old");
        let e = AeParams::from_env();
        std::env::remove_var("PIMAX_CAM_AE_LAW");
        assert_eq!(e.law, AeLaw::V1);
        assert_eq!(e.exposure_min, 100);
        assert_eq!(AeParams::from_env().law, AeLaw::V2);
    }

    #[test]
    fn metric_leaves_out_corners_and_clipped_cells() {
        // Flat 80 room, black corners (outside the mask), a clipped block over about 20 % of the view.
        let mut d = vec![80u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                let (dx, dy) = ((x as f64 - 319.5) / 320.0, (y as f64 - 239.5) / 240.0);
                if dx * dx + dy * dy > 1.0 {
                    d[y * 640 + x] = 0;
                }
                if x >= 512 {
                    d[y * 640 + x] = 255;
                }
            }
        }
        let m = scene_metric_grey(&d, 640, 480);
        assert!((m.mean - 80.0).abs() < 1.0, "{m:?}");
        assert!(m.sat > 0.1 && m.sat < 0.3, "{m:?}");
        // The plain mean (corners and window included) is far off.
        let plain = d.iter().map(|&v| v as f64).sum::<f64>() / d.len() as f64;
        assert!((plain - 80.0).abs() > 10.0);
        // All clipped: falls back to the whole view instead of 0.
        let m = scene_metric_grey(&vec![255u8; 640 * 480], 640, 480);
        assert!(m.mean > 250.0 && m.sat > 0.99);
        // Dark room.
        let m = scene_metric_grey(&vec![6u8; 640 * 480], 640, 480);
        assert!((m.mean - 6.0).abs() < 0.5 && m.sat == 0.0 && m.dark > 0.99);
        assert_eq!(scene_metric_grey(&[], 0, 0), SceneMetric::default());
    }

    #[test]
    fn good_frame_definition() {
        let m = |mean, sat| SceneMetric { mean, sat, dark: 0.0, n: 100 };
        assert!(is_good_frame(&m(60.0, 0.1), false, None, false));
        assert!(is_good_frame(&m(30.0, 0.20), false, Some(GOOD_QUIET_NS), false));
        assert!(is_good_frame(&m(190.0, 0.0), false, None, false));
        assert!(!is_good_frame(&m(29.9, 0.0), false, None, false));
        assert!(!is_good_frame(&m(190.1, 0.0), false, None, false));
        assert!(!is_good_frame(&m(60.0, 0.21), false, None, false));
        assert!(!is_good_frame(&m(60.0, 0.1), true, None, false));
        assert!(!is_good_frame(&m(60.0, 0.1), false, Some(GOOD_QUIET_NS - 1), false));
        assert!(!is_good_frame(&m(60.0, 0.1), false, None, true));
        assert!(!is_good_frame(&SceneMetric::default(), false, None, false));
        // StepWatch: a landed step, then 1 s of quiet after a big one.
        let mut w = StepWatch::default();
        let ok = m(60.0, 0.0);
        let mut t = 0u64;
        let step = 40_000_000u64;
        assert!(w.good(&ok, 2000, 13, t, false));
        t += step;
        assert!(w.good(&ok, 2000, 13, t, false));
        t += step;
        assert!(!w.good(&ok, 2600, 13, t, false), "a big step landed");
        let t_big = t;
        t += step;
        assert!(!w.good(&ok, 2600, 13, t, false), "within 1 s of the big step");
        t = t_big + GOOD_QUIET_NS;
        assert!(w.good(&ok, 2600, 13, t, false));
        // A small (3 %) ladder step does not count as big, but its own frame is not good.
        t += step;
        assert!(!w.good(&ok, 2680, 13, t, false));
        t += step;
        assert!(w.good(&ok, 2680, 13, t, false));
    }

    // ---- Crystal Super (analysis/crystal-super-ae.md) ----

    fn crystal() -> AeParams {
        AeParams::for_model(AeModel::CrystalSuper)
    }

    #[test]
    fn dream_air_defaults_are_unchanged() {
        assert_eq!(AeParams::for_model(AeModel::DreamAir), AeParams::default());
        assert_eq!(AeParams::old_law_for(AeModel::DreamAir), AeParams::old_law());
        let p = AeParams::default();
        assert_eq!((p.exposure_min, p.exposure_max, p.gain_min, p.gain_max, p.exposure_soft), (1140, 4108, 13, 215, 4108));
        assert_eq!(p.model, AeModel::DreamAir);
        assert_eq!(AeModel::DreamAir.pair_ranges(), None);
        assert_eq!(AeModel::DreamAir.power_on(), (10410, 449));
        // Golden split values (the same before the model constants existed).
        for (x, want) in [
            (1.0, (1140, 13)),
            (2200.0 * 13.0, (2200, 13)),
            (3150.0 * 13.0, (3143, 13)),
            (4108.0 * 13.0, (4108, 13)),
            (1e12, (4108, 215)),
        ] {
            assert_eq!(p.split(x), want, "{x}");
            assert_eq!(p.split_pair(1, x), want, "{x} upper");
        }
        // Extended stage 2 (Dream Air only): the power-on gain and the long exposure.
        let ae = AutoExposure::new(AeParams::default());
        let (e, g) = ae.caps(2);
        let q = AeParams { exposure_max: e, gain_max: g, ..AeParams::default() };
        assert_eq!(q.split(1e12), (10410, 449));
        assert_eq!(q.split(4108.0 * 300.0).1, 449);
    }

    #[test]
    fn crystal_tables_are_windows_values() {
        // Every pair Windows sent on the Crystal (16751 commands, both pairs) that is not on the low-gain ladder at
        // a gain below 215 is in the high table, and the tables ascend.
        assert!(CRYSTAL_EXPOSURE_LADDER.windows(2).all(|w| w[0] < w[1]));
        assert!(CRYSTAL_GAIN_CODES.windows(2).all(|w| w[0] < w[1]));
        assert!(CRYSTAL_HIGH.windows(2).all(|w| (w[0].0 as u32 * w[0].1 as u32) < (w[1].0 as u32 * w[1].1 as u32)));
        assert!(CRYSTAL_EXPOSURE_LADDER.iter().all(|v| EXPOSURE_LADDER.contains(v)), "the Dream Air's ladder from 2200");
        for e in [2200, 2875, 2962, 3758, 3988] {
            assert!(CRYSTAL_EXPOSURE_LADDER.contains(&e), "{e} at gain 13");
        }
        assert_eq!(CRYSTAL_HIGH[0], (4194, 215));
        assert_eq!(CRYSTAL_HIGH[CRYSTAL_HIGH.len() - 1], (10410, 449));
        assert_eq!(*CRYSTAL_GAIN_CODES.last().unwrap(), 215);
        // Steps above the soft cap stay small (at most 8.5 % in brightness, as Windows).
        let mut prev = 4108.0 * 215.0;
        for &(e, g) in &CRYSTAL_HIGH {
            let x = e as f64 * g as f64;
            assert!(x / prev < 1.085, "{e}/{g}");
            prev = x;
        }
        assert_eq!(AeModel::CrystalSuper.power_on(), (3500, 60));
    }

    #[test]
    fn crystal_split_follows_windows() {
        let p = crystal();
        assert_eq!((p.exposure_min, p.exposure_max, p.gain_min, p.gain_max), (2200, 10410, 13, 449));
        // Low light levels: the ladder at gain 13, never below Windows' 2200.
        assert_eq!(p.split(1.0), (2200, 13));
        assert_eq!(p.split(2200.0 * 13.0), (2200, 13));
        assert_eq!(p.split(2875.0 * 13.0), (2875, 13));
        assert_eq!(p.split(4108.0 * 13.0), (4108, 13));
        // Gain stage: a Crystal gain code with the exposure just below the soft cap.
        let (e, g) = p.split(3900.0 * 49.0);
        assert!(CRYSTAL_GAIN_CODES.contains(&g) && g <= 49 && e <= 4108, "{e}/{g}");
        // Above: every high setting maps onto itself, and the top is 10410 / 449 (upper pair: 8941 / 439).
        for &(e, g) in &CRYSTAL_HIGH {
            let x = e as f64 * g as f64;
            assert_eq!(p.split(x), (e, g));
            if e <= 8941 && g <= 439 {
                assert_eq!(p.split_pair(1, x), (e, g));
            }
        }
        assert_eq!(p.split(1e12), (10410, 449));
        assert_eq!(p.split_pair(1, 1e12), (8941, 439));
        // Never the Dream Air's 449-at-4108 jump: a product just above the gain stage takes gain 215.
        assert_eq!(p.split(4150.0 * 215.0).1, 215);
        // Every split of a product sweep is a Windows setting inside the pair's range.
        let mut x = 1000.0;
        while x < 1e8 {
            for k in 0..2 {
                let (e, g) = p.split_pair(k, x);
                let (elo, ehi, glo, ghi) = CRYSTAL_PAIR_RANGES[k];
                assert!((elo..=ehi).contains(&e) && (glo..=ghi).contains(&g), "{x}: {e}/{g}");
                let on_ladder = g < 215 && CRYSTAL_GAIN_CODES.contains(&g) && e <= 4108;
                let gain13 = g == 13 && CRYSTAL_EXPOSURE_LADDER.contains(&e);
                let top = g == 215 && e <= 4108 && CRYSTAL_EXPOSURE_LADDER.contains(&e);
                assert!(on_ladder || gain13 || top || CRYSTAL_HIGH.contains(&(e, g)), "{x}: {e}/{g} not a Windows setting");
            }
            x *= 1.01;
        }
    }

    #[test]
    fn crystal_overrides_stay_in_the_windows_range() {
        // The env is process wide: only this test sets these names (others build their params directly).
        std::env::set_var("PIMAX_CAM_AE_EXP_MIN", "100");
        std::env::set_var("PIMAX_CAM_AE_GAIN_MAX", "900");
        let c = AeParams::from_env_model(AeModel::CrystalSuper);
        let d = AeParams::from_env_model(AeModel::DreamAir);
        std::env::remove_var("PIMAX_CAM_AE_EXP_MIN");
        std::env::remove_var("PIMAX_CAM_AE_GAIN_MAX");
        assert_eq!((c.exposure_min, c.gain_max), (2200, 449));
        // The Dream Air's bounds are the old ones.
        assert_eq!((d.exposure_min, d.gain_max), (100, 449));
        let o = AeParams::old_law_for(AeModel::CrystalSuper);
        assert_eq!((o.exposure_min, o.law, o.model), (2200, AeLaw::V1, AeModel::CrystalSuper));
        // No extended range and no low stage on the Crystal: the normal range is already Windows' whole range.
        let ae = AutoExposure::new(crystal());
        assert_eq!(ae.top_stage(), 0);
        assert!(!ae.p.light.clip_enabled);
    }

    /// Closed loop on the simulated sensor from the Crystal's firmware default, dark to bright rooms: every target is
    /// inside Windows' per-pair ranges (what the allowlist accepts), the pairs stay equal, it settles.
    #[test]
    fn crystal_closed_loop_stays_in_range() {
        for &(radiance, blob) in &[(90.0, true), (30.0, false), (300.0, false), (6.0, false), (0.5, false), (3000.0, true)] {
            let mut ae = AutoExposure::new(crystal());
            let (pe, pg) = CRYSTAL_POWER_ON;
            let mut applied = AeSetting { exposure: [pe; 4], gain: [pg; 4], led_exposure: 40, led_gain: 62 };
            let mut pipeline: Vec<(usize, AeSetting)> = Vec::new();
            let mut t = 0u64;
            let mut means = Vec::new();
            for i in 0..250usize {
                while pipeline.first().is_some_and(|(f, _)| *f <= i) {
                    applied = pipeline.remove(0).1;
                }
                let lo = render(radiance, applied.exposure[0], applied.gain[0], blob);
                let up = render(radiance * 0.6, applied.exposure[2], applied.gain[2], false);
                let tiles = [view(&lo), view(&lo), view(&up), view(&up)];
                let st = ae.frame_stats(&tiles, &Mask::default());
                means.push(st[0].trimmed_mean);
                let d = ae.update(&applied, st, t);
                let tg = ae.target().unwrap();
                assert!(AeModel::CrystalSuper.allows(&tg), "radiance {radiance}: {tg:?}");
                assert!(tg.exposure[0] == tg.exposure[1] && tg.exposure[2] == tg.exposure[3]);
                assert!(tg.gain[0] == tg.gain[1] && tg.gain[2] == tg.gain[3]);
                assert_eq!(&payload(&tg)[..2], &AE_HEAD);
                if d.send {
                    pipeline.push((i + 2, tg));
                }
                t += 40_000_000;
            }
            let m = *means.last().unwrap();
            let (e, g) = (applied.exposure[0], applied.gain[0]);
            let pinned_low = e == 2200 && g == 13;
            let pinned_high = e == 10410 && g == 449;
            assert!(
                (m / 64.0).ln().abs() < 0.3 || (pinned_low && m > 64.0) || (pinned_high && m < 64.0),
                "radiance {radiance}: mean {m} at {e}/{g}"
            );
        }
    }

    #[test]
    fn crystal_fit_and_allows() {
        let p = crystal();
        // The Dream Air's power-on 10410/449 on the upper pair is outside the Crystal's upper range: refitted.
        assert_eq!(p.fit_pair(1, 10410, 449), (8941, 439));
        assert_eq!(p.fit_pair(0, 10410, 449), (10410, 449));
        assert_eq!(p.fit_pair(0, 3500, 60), (3500, 60), "the firmware default is inside the ranges");
        assert_eq!(p.fit_pair(0, 1000, 13), (2200, 13));
        let ok = AeSetting { exposure: [10410, 10410, 8941, 8941], gain: [449, 449, 439, 439], led_exposure: 40, led_gain: 62 };
        assert!(AeModel::CrystalSuper.allows(&ok));
        assert!(!AeModel::CrystalSuper.allows(&AeSetting { exposure: [10410; 4], ..ok }));
        assert!(!AeModel::CrystalSuper.allows(&AeSetting { gain: [449, 449, 440, 440], ..ok }));
        assert!(!AeModel::CrystalSuper.allows(&AeSetting { exposure: [2199, 2199, 3000, 3000], ..ok }));
        assert!(!AeModel::CrystalSuper.allows(&AeSetting { led_exposure: 41, ..ok }));
        assert!(AeModel::DreamAir.allows(&AeSetting { exposure: [1; 4], ..ok }), "the Dream Air enforces nothing here");
        // The first Windows command on the Crystal (take cold2): 2200/13 on all four cameras.
        let first = parse_payload(&[
            0x58, 0x19, 0x51, 0x00, 0x01, 0x32, 0x98, 0x08, 0x98, 0x08, 0x98, 0x08, 0x98, 0x08, 0x0d, 0x00, 0x0d, 0x00,
            0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00,
        ])
        .unwrap();
        assert_eq!(first.exposure, [2200; 4]);
        assert!(AeModel::CrystalSuper.allows(&first));
    }

    #[test]
    fn model_detection() {
        assert_eq!(model_from_pids(&[0x0040, 0x0041, 0x0014]).0, AeModel::CrystalSuper);
        assert_eq!(model_from_pids(&[0x0044, 0x0043, 0x0014]).0, AeModel::DreamAir);
        assert_eq!(model_from_pids(&[]).0, AeModel::DreamAir);
        assert_eq!(model_from_pids(&[0x0040, 0x0044]).0, AeModel::DreamAir);
        for m in [AeModel::DreamAir, AeModel::CrystalSuper] {
            assert_eq!(AeModel::parse(m.id()), Some(m));
        }
        assert_eq!(AeModel::parse("Crystal_Super"), Some(AeModel::CrystalSuper));
        assert_eq!(AeModel::parse("8kx"), None);
        assert_eq!(AeModel::by_pid(0x0040), Some(AeModel::CrystalSuper));
        assert_eq!(AeModel::by_pid(0x0044), Some(AeModel::DreamAir));
        // A sysfs-like tree.
        let dir = std::env::temp_dir().join(format!("pimax-ae-sysfs-{}", std::process::id()));
        for (n, v, p) in [("1-1", "34a4", "0040"), ("1-2", "34a4", "0014"), ("1-3", "046d", "0040")] {
            let d = dir.join(n);
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join("idVendor"), format!("{v}\n")).unwrap();
            std::fs::write(d.join("idProduct"), format!("{p}\n")).unwrap();
        }
        let mut pids = usb_pids_sysfs(&dir);
        pids.sort();
        assert_eq!(pids, [0x0014, 0x0040]);
        assert_eq!(model_from_pids(&pids).0, AeModel::CrystalSuper);
        let _ = std::fs::remove_dir_all(&dir);
    }
}

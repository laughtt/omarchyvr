//! `omarchyvr-slam`: offline tools for Dream Air 6DoF head tracking (Monado + Basalt).
//!
//! ```text
//! omarchyvr-slam export     [--out DIR] [--calib XML] [--cams 0,1] [--imu F] [--base CONFIG.json] [--gui] [--cam-offset-ns N]
//!                       [--gyro-bias X,Y,Z|none] [--bias-from DATASET] [--accel-bias factory|none|X,Y,Z]
//!                       Basalt calibration (kb4), VIO config and SLAM_CONFIG toml (default DIR ~/.config/omarchyvr/slam).
//!                       Gyro bias seed (calib_gyro_bias, body frame): --gyro-bias in deg/s, else the first still
//!                       second of DATASET, else the driver's cache (~/.cache/omarchyvr/gyro-bias), else none.
//!                       Accel bias (calib_accel_bias, body frame, m/s^2): default the factory Stateinit aBias.
//! omarchyvr-slam check      DATASET [--no-files]           EuRoC recording sanity (rates, gaps, sync, clock overlap)
//! omarchyvr-slam imu-noise  DATASET [--window S] [--inflate K] [--write F]
//!                       IMU noise from the stillest window (default window 2 s, inflate 3, F ~/.config/omarchyvr/slam-imu.json)
//! omarchyvr-slam timing     REC.pxcam [--frames N]         camera frame timing / exposure from a raw recording
//! omarchyvr-slam ate        EST.csv REF.csv [--est-format euroc|pimax-windows] [--ref-format ..] [--max-gap-ms M] [--rpe-s D]
//! omarchyvr-slam loop       EST.csv                        loop-closure drift (run that ends where it started)
//! omarchyvr-slam stats      TRAJ.csv [--format F] [--vs TRAJ2 [--vs-format F]]
//!                       reference-free statistics: rate, still noise, high-frequency noise, drift per metre
//! omarchyvr-slam simulate   TRUTH.csv [--format pimax-windows] [--noise-mm 2] [--latency-ms 25] [--tau-ms 150]
//!                       [--dropout A:B]  head pose output on a real head trajectory (synthetic IMU + tracker)
//! omarchyvr-slam stillgate DATASET [--traj tracking.csv]  the still gate replayed on a recording + a Basalt run
//!                       (default: the newest DATASET/replay-*/tracking.csv): old drop-to-3DoF vs re-zero
//! omarchyvr-slam corrections DATASET [--traj tracking.csv] [--central-vel]
//!                       6DoF output corrections (count, >= 20 mm, vertical) and jitter while worn and quiet,
//!                       current settings vs each smoothing option
//! omarchyvr-slam bench-windows TAKE [--root DIR] [--latency-ms 30] [--predict-ms 0] [--fast-dps 150] [--tsv OUT]
//!                       Windows capture take (e.g. fast-turns: pcap + Pimax pose log) as a benchmark: clock, lag,
//!                       smoothing, IMU fit, Pimax jitter/losses, and our head pose output (real IMU, synthetic
//!                       25 Hz tracker from Pimax's poses) scored against Pimax; PIMAX_SLAM_* switches apply
//! omarchyvr-slam bench-basalt DATASET [--traj tracking.csv]  Basalt vs the gyro binned by head angular speed
//! omarchyvr-slam take-head TAKE [--every N] [--csv OUT]  Record-tracking take: live tracker log + cameras at speed
//!                       (head speed from the images, features kept, exposure, brightness, blur per speed bin)
//! omarchyvr-slam take-euroc TAKE [OUT]   take (cameras.pxcam + head-imu.csv) as a EuRoC dataset for run_slam.sh replay
//! omarchyvr-slam ae         REC.pxcam [--frames N] [--model crystal-super]  auto-exposure dry run on a recording (what it would send)
//! omarchyvr-slam shift      DATASET OUT --cam-offset-ns N  copy with camera timestamps shifted (offset sweeps)
//! omarchyvr-slam camlag     DATASET [--calib XML] [--cam 0] [--every 1] [--range-ms 30] [--radius MIN,MAX]
//!                       camera-IMU time offset, gyro bias and gyro gain from the recording alone (image rotation
//!                       between frames vs the integrated gyro)
//! omarchyvr-slam smoke-euroc REC.pxcam OUT [--cams 0,1] [--seconds S]
//!                       pipeline smoke test: camera frames + a synthetic still IMU (NOT for accuracy)
//! ```
//! DATASET: a EuRoC root (holding mav0) or the driver's recording directory (newest euroc_* inside).

use pimax_cam::calib::DeviceCalib;
use pimax_cam::slam::{self, ImuNoise, SlamSetup};
use omarchyvr_slam::{camlag, euroc, export, imu, sim, stats, stillgate, takehead, timing, traj, winbench};
use std::path::{Path, PathBuf};

type R<T> = Result<T, String>;

struct Args(Vec<String>);

impl Args {
    fn flag(&mut self, n: &str) -> bool {
        match self.0.iter().position(|a| a == n) {
            Some(i) => {
                self.0.remove(i);
                true
            }
            None => false,
        }
    }
    fn opt(&mut self, n: &str) -> R<Option<String>> {
        match self.0.iter().position(|a| a == n) {
            Some(i) if i + 1 < self.0.len() => {
                let v = self.0.remove(i + 1);
                self.0.remove(i);
                Ok(Some(v))
            }
            Some(_) => Err(format!("{n} needs a value")),
            None => Ok(None),
        }
    }
    fn num<T: std::str::FromStr>(&mut self, n: &str, d: T) -> R<T> {
        match self.opt(n)? {
            Some(v) => v.parse().map_err(|_| format!("{n}: bad value {v:?}")),
            None => Ok(d),
        }
    }
    fn pos(&mut self, what: &str) -> R<String> {
        if self.0.is_empty() || self.0[0].starts_with("--") {
            return Err(format!("missing {what}"));
        }
        Ok(self.0.remove(0))
    }
    fn done(&self) -> R<()> {
        match self.0.first() {
            Some(a) => Err(format!("unexpected argument {a:?}")),
            None => Ok(()),
        }
    }
}

fn home() -> PathBuf {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
}

fn load_calib(p: Option<String>) -> R<DeviceCalib> {
    let (c, used) =
        DeviceCalib::load_default(p.as_deref().map(Path::new)).map_err(|e| e.to_string())?;
    eprintln!("calibration: {} ({})", used.display(), c.uid);
    Ok(c)
}

fn cmd_export(mut a: Args) -> R<()> {
    let out = a
        .opt("--out")?
        .map(PathBuf::from)
        .unwrap_or_else(|| home().join(".config/omarchyvr/slam"));
    let cal = load_calib(a.opt("--calib")?)?;
    let cams = slam::parse_cams(
        &a.opt("--cams")?.unwrap_or_else(|| "0,1".into()),
        cal.cameras.len(),
    )?;
    let imu_p = a.opt("--imu")?;
    let base = a.opt("--base")?.map(PathBuf::from);
    let gui = a.flag("--gui");
    let offset: i64 = a.num("--cam-offset-ns", 0)?;
    let bias_arg = a.opt("--gyro-bias")?;
    let bias_from = a.opt("--bias-from")?;
    let accel_arg = a.opt("--accel-bias")?;
    a.done()?;
    let imu = ImuNoise::load(imu_p.as_deref().map(Path::new))?.unwrap_or_default();
    let (gyro_bias, bias_src) = gyro_bias_seed(bias_arg.as_deref(), bias_from.as_deref())?;
    let accel_bias = match accel_arg.as_deref().unwrap_or("factory") {
        "factory" => slam::factory_accel_bias(&cal).unwrap_or([0.0; 3]),
        "none" => [0.0; 3],
        v => {
            let x: Vec<f64> = v
                .split(',')
                .map(|w| w.trim().parse::<f64>())
                .collect::<Result<_, _>>()
                .map_err(|_| format!("--accel-bias {v:?}: want factory, none or X,Y,Z in m/s^2"))?;
            if x.len() != 3 {
                return Err(format!("--accel-bias {v:?}: want factory, none or X,Y,Z in m/s^2"));
            }
            [x[0], x[1], x[2]]
        }
    };
    let setup = SlamSetup {
        cams,
        cam_time_offset_ns: offset,
        gyro_bias,
        accel_bias,
        ..Default::default()
    };
    let e = export::export(&out, &cal, &setup, &imu, base.as_deref(), gui)?;
    println!("calibration  {}", e.calib.display());
    println!(
        "vio config   {} (base {})",
        e.config.display(),
        e.base.display()
    );
    println!("SLAM_CONFIG  {}", e.toml.display());
    println!("IMU noise    {}", imu.source);
    println!(
        "gyro bias    ({:+.3} {:+.3} {:+.3}) deg/s body, {bias_src}",
        gyro_bias[0].to_degrees(),
        gyro_bias[1].to_degrees(),
        gyro_bias[2].to_degrees()
    );
    println!(
        "accel bias   ({:+.4} {:+.4} {:+.4}) m/s^2 body",
        accel_bias[0], accel_bias[1], accel_bias[2]
    );
    for &i in &setup.cams {
        let p = slam::t_imu_cam(&cal, i).unwrap();
        let q = slam::quat_of(&p.r);
        println!(
            "  cam {i} {:<15} T_imu_cam p=({:+.4} {:+.4} {:+.4}) q=({:+.4} {:+.4} {:+.4} {:+.4})",
            cal.cameras[i].name, p.t[0], p.t[1], p.t[2], q[0], q[1], q[2], q[3]
        );
    }
    Ok(())
}

/// Gyro bias seed for Basalt (body frame, rad/s) and where it came from.
fn gyro_bias_seed(arg: Option<&str>, from: Option<&str>) -> R<([f64; 3], String)> {
    match arg {
        Some("none") => return Ok(([0.0; 3], "none (--gyro-bias none)".into())),
        Some(v) => {
            let x: Vec<f64> = v
                .split(',')
                .map(|w| w.trim().parse::<f64>())
                .collect::<Result<_, _>>()
                .map_err(|_| format!("--gyro-bias {v:?}: want X,Y,Z in deg/s"))?;
            if x.len() != 3 {
                return Err(format!("--gyro-bias {v:?}: want X,Y,Z in deg/s"));
            }
            return Ok((
                [x[0], x[1], x[2]].map(f64::to_radians),
                "--gyro-bias".into(),
            ));
        }
        None => {}
    }
    if let Some(d) = from {
        let ds = euroc::Dataset::load(Path::new(d))?;
        if let Some((b, t)) = imu::still_gyro_bias(&ds.imu, 1.0) {
            let at = (t - ds.imu[0].ts) as f64 / 1e9;
            return Ok((
                b,
                format!("first still second of {} (at {at:.1} s)", ds.root.display()),
            ));
        }
        eprintln!("{d}: the headset never lay still; using the driver's cached bias");
    }
    match slam::gyro_bias_cache() {
        Some(b) => Ok((
            b,
            format!(
                "driver cache {}",
                slam::gyro_bias_cache_path().unwrap_or_default().display()
            ),
        )),
        // nothing in the new cache: the cache of the names before the rename (the driver reads it too), else the value
        // the driver starts with on a clean install (the launcher writes it right after this export; LESSONS 13: a
        // zero seed lets the tracker drift)
        None => {
            let env = |k: &str| std::env::var(k).ok();
            let legacy = omarchyvr_config::devicedata::legacy_gyro_bias_path(&home(), &env);
            if let Some(b) = std::fs::read_to_string(&legacy).ok().and_then(|t| slam::parse_gyro_bias_cache(&t)) {
                return Ok((b, format!("driver cache {}", legacy.display())));
            }
            match slam::parse_gyro_bias_cache(omarchyvr_config::devicedata::REFERENCE_GYRO_BIAS) {
                Some(b) => Ok((b, "the Dream Air default (no measured bias on this PC yet)".into())),
                None => Ok(([0.0; 3], "none (no driver cache yet)".into())),
            }
        }
    }
}

fn cmd_check(mut a: Args) -> R<()> {
    let d = a.pos("DATASET")?;
    let files = !a.flag("--no-files");
    a.done()?;
    let ds = euroc::Dataset::load(Path::new(&d))?;
    let r = euroc::check(&ds, files);
    print!("{}", r.text);
    if r.problems.is_empty() {
        Ok(())
    } else {
        Err(format!("{} problem(s)", r.problems.len()))
    }
}

fn cmd_imu_noise(mut a: Args) -> R<()> {
    let d = a.pos("DATASET")?;
    let window: f64 = a.num("--window", 2.0)?;
    let inflate: f64 = a.num("--inflate", 3.0)?;
    let write = a.opt("--write")?;
    a.done()?;
    let ds = euroc::Dataset::load(Path::new(&d))?;
    let e = imu::estimate(&ds.imu, window, inflate)?;
    let t0 = ds.imu[0].ts;
    println!(
        "stillest {window:.1} s: {:.2}..{:.2} s, rate {:.1} Hz",
        (e.window.0 - t0) as f64 / 1e9,
        (e.window.1 - t0) as f64 / 1e9,
        e.rate_hz
    );
    println!(
        "gyro std  {:.5?} rad/s, mean (bias) {:.5?} rad/s",
        e.gyro_std, e.gyro_mean
    );
    println!(
        "accel std {:.5?} m/s², mean {:.4?} m/s²",
        e.accel_std, e.accel_mean
    );
    println!(
        "densities x{inflate}: gyro {:.3e} {:.3e} {:.3e} rad/s/√Hz, accel {:.3e} {:.3e} {:.3e} m/s²/√Hz (bias random walk: defaults)",
        e.noise.gyro_noise_std[0], e.noise.gyro_noise_std[1], e.noise.gyro_noise_std[2],
        e.noise.accel_noise_std[0], e.noise.accel_noise_std[1], e.noise.accel_noise_std[2]
    );
    let out = match write.as_deref() {
        Some("-") => None,
        Some(p) => Some(PathBuf::from(p)),
        None => ImuNoise::default_path(),
    };
    if let Some(p) = out {
        if let Some(parent) = p.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        let comment = format!(
            "omarchyvr-slam imu-noise {} (window {window} s, inflate {inflate}); bias random walk = defaults",
            ds.root.display()
        );
        std::fs::write(&p, e.noise.to_json(&comment))
            .map_err(|e| format!("{}: {e}", p.display()))?;
        println!(
            "wrote {} (used by the driver and by omarchyvr-slam export)",
            p.display()
        );
    }
    Ok(())
}

fn cmd_timing(mut a: Args) -> R<()> {
    let p = a.pos("REC.pxcam")?;
    let n: usize = a.num("--frames", 0)?;
    a.done()?;
    let infos = timing::read_infos(&p, n)?;
    print!("{}", timing::report(&infos));
    Ok(())
}

fn load_traj(p: &str, fmt: &str) -> R<Vec<traj::PoseRow>> {
    let t = std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"))?;
    traj::parse(&t, traj::Format::parse(fmt)?)
}

fn cmd_ate(mut a: Args) -> R<()> {
    let ef = a.opt("--est-format")?.unwrap_or_else(|| "euroc".into());
    let rf = a.opt("--ref-format")?.unwrap_or_else(|| "euroc".into());
    let gap: f64 = a.num("--max-gap-ms", 50.0)?;
    let rpe: f64 = a.num("--rpe-s", 1.0)?;
    let e = a.pos("EST.csv")?;
    let r = a.pos("REF.csv")?;
    a.done()?;
    let est = load_traj(&e, &ef)?;
    let reference = load_traj(&r, &rf)?;
    let m = traj::compare(&est, &reference, (gap * 1e6) as i64, rpe)?;
    println!(
        "{} matched poses, reference path {:.2} m\nATE (rigid alignment): RMSE {:.4} m, median {:.4} m, max {:.4} m\nRPE over {:.1} s: translation RMSE {:.4} m, rotation RMSE {:.3} deg",
        m.pairs, m.path_len, m.ate_rmse, m.ate_median, m.ate_max, m.rpe_delta_s, m.rpe_trans_rmse, m.rpe_rot_rmse_deg
    );
    Ok(())
}

fn cmd_loop(mut a: Args) -> R<()> {
    let fmt = a.opt("--format")?.unwrap_or_else(|| "euroc".into());
    let e = a.pos("EST.csv")?;
    a.done()?;
    let est = load_traj(&e, &fmt)?;
    let (d, rot, path) = traj::loop_drift(&est, 1.0).ok_or("empty trajectory")?;
    println!(
        "end-start distance {:.3} m over a {:.2} m path ({:.2} %), end-start rotation {:.2} deg",
        d,
        path,
        100.0 * d / path.max(1e-9),
        rot
    );
    Ok(())
}

fn cmd_smoke(mut a: Args) -> R<()> {
    let src = a.pos("REC.pxcam")?;
    let out = PathBuf::from(a.pos("OUT")?);
    let cams = slam::parse_cams(&a.opt("--cams")?.unwrap_or_else(|| "0,1".into()), 4)?;
    let secs: f64 = a.num("--seconds", 20.0)?;
    a.done()?;
    use pimax_cam::camera::Camera;
    use pimax_cam::source::ReplayOptions;
    let mut cam =
        Camera::open(&src, 0, ReplayOptions::default()).map_err(|e| format!("open {src}: {e}"))?;
    let mut lists: Vec<Vec<(i64, String)>> = vec![Vec::new(); cams.len()];
    for i in 0..cams.len() {
        std::fs::create_dir_all(out.join(format!("mav0/cam{i}/data")))
            .map_err(|e| e.to_string())?;
    }
    let mut t_first = None;
    let mut buf = vec![0u8; 640 * 480];
    loop {
        let info = match cam.next_frame(100).map_err(|e| e.to_string())? {
            Some(i) => i,
            None if cam.source().finished() => break,
            None => continue,
        };
        if info.kind != 1 {
            continue;
        }
        let ts = (info.dev_ts_us * 1000) as i64;
        let t0 = *t_first.get_or_insert(ts);
        if (ts - t0) as f64 / 1e9 > secs {
            break;
        }
        for (k, &c) in cams.iter().enumerate() {
            cam.copy_tile(c, &mut buf, 640).map_err(|e| e.to_string())?;
            let name = format!("{ts}.png");
            pimax_cam::png::write_grey(
                &out.join(format!("mav0/cam{k}/data/{name}")),
                640,
                480,
                &buf,
            )
            .map_err(|e| e.to_string())?;
            lists[k].push((ts, name));
        }
    }
    let (first, last) = match (lists[0].first(), lists[0].last()) {
        (Some(a), Some(b)) => (a.0, b.0),
        _ => return Err("no tracking frames in the recording".into()),
    };
    // Synthetic still IMU (body frame y up: specific force +g on y), 1 kHz, 0.5 s around the frames.
    let imu: Vec<euroc::ImuRow> = (0..)
        .map(|i| first - 500_000_000 + i * 1_000_000)
        .take_while(|&t| t <= last + 500_000_000)
        .map(|ts| euroc::ImuRow {
            ts,
            gyro: [0.0; 3],
            accel: [0.0, 9.80665, 0.0],
        })
        .collect();
    euroc::write_csvs(&out, &imu, &lists).map_err(|e| e.to_string())?;
    println!(
        "wrote {} frame sets ({} cameras) + {} synthetic IMU samples to {} (smoke test only: the IMU is fake)",
        lists[0].len(),
        cams.len(),
        imu.len(),
        out.display()
    );
    Ok(())
}

fn cmd_stats(mut a: Args) -> R<()> {
    let fmt = a.opt("--format")?.unwrap_or_else(|| "euroc".into());
    let vs = a.opt("--vs")?;
    let vs_fmt = a
        .opt("--vs-format")?
        .unwrap_or_else(|| "pimax-windows".into());
    let e = a.pos("TRAJ.csv")?;
    a.done()?;
    let p = stats::StatsParams::default();
    let s1 = stats::stats(&load_traj(&e, &fmt)?, &p);
    let s2 = match &vs {
        Some(v) => Some(stats::stats(&load_traj(v, &vs_fmt)?, &p)),
        None => None,
    };
    let n1 = Path::new(&e)
        .file_name()
        .map(|x| x.to_string_lossy().into_owned())
        .unwrap_or_default();
    let n2 = vs
        .as_deref()
        .and_then(|v| Path::new(v).file_name())
        .map(|x| x.to_string_lossy().into_owned())
        .unwrap_or_default();
    print!("{}", stats::table(&s1, s2.as_ref(), (&n1, &n2)));
    Ok(())
}

fn replay_inputs(a: &mut Args) -> R<(euroc::Dataset, Vec<traj::PoseRow>)> {
    let traj_arg = a.opt("--traj")?;
    let d = a.pos("DATASET")?;
    let dir = PathBuf::from(&d);
    let traj_path = match traj_arg {
        Some(t) => PathBuf::from(t),
        None => {
            let mut c: Vec<PathBuf> = std::fs::read_dir(&dir)
                .map_err(|e| format!("{d}: {e}"))?
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| {
                    p.file_name()
                        .and_then(|n| n.to_str())
                        .is_some_and(|n| n.starts_with("replay-"))
                        && p.join("tracking.csv").is_file()
                })
                .collect();
            c.sort();
            c.pop()
                .ok_or(
                    "no replay-*/tracking.csv: run `run_slam.sh replay DIR` first or pass --traj",
                )?
                .join("tracking.csv")
        }
    };
    let ds = euroc::Dataset::load(&dir)?;
    let est = load_traj(&traj_path.to_string_lossy(), "euroc")?;
    eprintln!(
        "{}: {} IMU samples, {} tracker poses ({})",
        d,
        ds.imu.len(),
        est.len(),
        traj_path.display()
    );
    Ok((ds, est))
}

fn cmd_stillgate(mut a: Args) -> R<()> {
    let (ds, est) = replay_inputs(&mut a)?;
    a.done()?;
    let mut old = stillgate::GateParams::default();
    old.head.still.rezero = false;
    for (name, p) in [
        ("old (drop to 3DoF)", old),
        ("re-zero (new)", stillgate::GateParams::default()),
    ] {
        match stillgate::run(&ds.imu, &est, &p) {
            Ok(r) => println!("{}", stillgate::summary(name, &r)),
            Err(e) => println!("{name:<22} failed: {e}"),
        }
    }
    Ok(())
}

fn cmd_corrections(mut a: Args) -> R<()> {
    let central = a.flag("--central-vel");
    let (ds, est) = replay_inputs(&mut a)?;
    a.done()?;
    let mut base = stillgate::GateParams::default();
    base.central_vel = central;
    for (name, p) in stillgate::variants(&base) {
        match stillgate::run(&ds.imu, &est, &p) {
            Ok(r) => println!("{}", stillgate::corr_summary(&name, &r)),
            Err(e) => println!("{name:<26} failed: {e}"),
        }
    }
    Ok(())
}

fn cmd_simulate(mut a: Args) -> R<()> {
    let fmt = a.opt("--format")?.unwrap_or_else(|| "pimax-windows".into());
    let mut p = sim::SimParams::default();
    p.vio_noise_m = a.num("--noise-mm", 2.0)? / 1000.0;
    p.latency_ms = a.num("--latency-ms", p.latency_ms)?;
    p.predict_ms = a.num("--predict-ms", p.predict_ms)?;
    let tau: f64 = a.num("--tau-ms", 150.0)?;
    p.head.tau_pos_s = tau / 1000.0;
    p.head.tau_rot_s = tau / 1500.0;
    while let Some(d) = a.opt("--dropout")? {
        let (x, y) = d.split_once(':').ok_or("--dropout wants A:B seconds")?;
        p.dropouts.push((
            x.parse().map_err(|_| "bad --dropout")?,
            y.parse().map_err(|_| "bad --dropout")?,
        ));
    }
    let t = a.pos("TRUTH.csv")?;
    a.done()?;
    let truth = load_traj(&t, &fmt)?;
    eprintln!(
        "truth: {} poses over {:.1} s",
        truth.len(),
        (truth[truth.len() - 1].ts - truth[0].ts) as f64 / 1e9
    );
    let mut raw = p.clone();
    raw.head.smooth = false;
    raw.head.resnap = false;
    let mut nore = p.clone();
    nore.head.resnap = false;
    for (name, q) in [
        ("raw (phase 1)", &raw),
        ("smoothed, no re-snap", &nore),
        ("smoothed + re-snap", &p),
    ] {
        match sim::simulate(&truth, q) {
            Ok(r) => println!("{}", sim::summary(name, &r)),
            Err(e) => println!("{name:<26} failed: {e}"),
        }
    }
    Ok(())
}

fn cmd_ae(mut a: Args) -> R<()> {
    let src = a.pos("REC.pxcam")?;
    let max: usize = a.num("--frames", 0)?;
    let model = match a.opt("--model")? {
        None => pimax_cam::ae::AeModel::DreamAir,
        Some(m) => pimax_cam::ae::AeModel::parse(&m).ok_or(format!("--model {m}: dream-air or crystal-super"))?,
    };
    a.done()?;
    use pimax_cam::ae::{AeParams, AeSetting, AutoExposure, Mask, TileView};
    use pimax_cam::camera::Camera;
    use pimax_cam::source::ReplayOptions;
    let mut cam =
        Camera::open(&src, 0, ReplayOptions::default()).map_err(|e| format!("open {src}: {e}"))?;
    let ae = AutoExposure::new(AeParams::for_model(model));
    let mask = Mask::default();
    let (mut n, mut means, mut sats, mut req_e, mut req_g) =
        (0usize, Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut last_print = 0u64;
    println!("    t     frame exp/gain   lower mean sat   upper mean sat  -> would request lower e/g, upper e/g");
    loop {
        let info = match cam.next_frame(100).map_err(|e| e.to_string())? {
            Some(i) => i,
            None if cam.source().finished() => break,
            None => continue,
        };
        if info.kind != 1 {
            continue;
        }
        let mut v: Vec<TileView> = Vec::new();
        for t in 0..4 {
            v.push(cam.tile_view(t).map_err(|e| e.to_string())?);
        }
        let tiles = [v[0], v[1], v[2], v[3]];
        let st = ae.frame_stats(&tiles, &mask);
        let set = AeSetting::from_info(&info, 40, 62);
        let (e0, g0, _) = ae.step_pair(set.exposure[0], set.gain[0], &st[0]);
        let (e2, g2, _) = ae.step_pair_k(1, set.exposure[2], set.gain[2], &st[1]);
        means.push(st[0].trimmed_mean);
        sats.push(st[0].sat_frac);
        req_e.push(e0 as f64);
        req_g.push(g0 as f64);
        if info.dev_ts_us >= last_print + 1_000_000 {
            last_print = info.dev_ts_us;
            println!(
                "{:>7.1} {:>7}/{:<5} {:>8.1} {:.3} {:>8.1} {:.3}  -> {}/{}, {}/{}",
                info.dev_ts_us as f64 / 1e6,
                set.exposure[0],
                set.gain[0],
                st[0].trimmed_mean,
                st[0].sat_frac,
                st[1].trimmed_mean,
                st[1].sat_frac,
                e0,
                g0,
                e2,
                g2
            );
        }
        n += 1;
        if max > 0 && n >= max {
            break;
        }
    }
    let med = |v: &mut Vec<f64>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.get(v.len() / 2).copied().unwrap_or(f64::NAN)
    };
    println!(
        "{n} tracking frames: lower-pair trimmed mean median {:.1}, saturated median {:.3}; first-step request median exposure {:.0} gain {:.0} (open loop: a recording cannot respond)",
        med(&mut means), med(&mut sats), med(&mut req_e), med(&mut req_g)
    );
    Ok(())
}

fn cmd_shift(mut a: Args) -> R<()> {
    let off: i64 = a.num("--cam-offset-ns", 0)?;
    let d = a.pos("DATASET")?;
    let out = PathBuf::from(a.pos("OUT")?);
    a.done()?;
    let ds = euroc::Dataset::load(Path::new(&d))?;
    let root = euroc::shift_cams(&ds, &out, off)?;
    println!(
        "{} (camera timestamps {off:+} ns, images linked from {})",
        root.display(),
        ds.root.display()
    );
    Ok(())
}

fn cmd_take_head(mut a: Args) -> R<()> {
    let every: usize = a.num("--every", 1)?;
    let csv = a.opt("--csv")?;
    let d = a.pos("TAKE")?;
    a.done()?;
    print!("{}", takehead::run(Path::new(&d), every.max(1), csv.as_deref().map(Path::new))?);
    Ok(())
}

fn cmd_camlag(mut a: Args) -> R<()> {
    let cal = load_calib(a.opt("--calib")?)?;
    let cam: usize = a.num("--cam", 0)?;
    let every: usize = a.num("--every", 1)?;
    let range: f64 = a.num("--range-ms", 30.0)?;
    let radius = match a.opt("--radius")? {
        None => (0.0, f64::INFINITY),
        Some(v) => {
            let x: Vec<f64> = v.split(',').filter_map(|w| w.trim().parse().ok()).collect();
            if x.len() != 2 || x[0] >= x[1] {
                return Err(format!("--radius {v:?}: want MIN,MAX in pixels"));
            }
            (x[0], x[1])
        }
    };
    let d = a.pos("DATASET")?;
    a.done()?;
    let ds = euroc::Dataset::load(Path::new(&d))?;
    print!("{}", camlag::run(&ds, &cal, cam, every, range, radius)?);
    Ok(())
}

fn cmd_take_euroc(mut a: Args) -> R<()> {
    let d = a.pos("TAKE")?;
    let out = if a.0.is_empty() { None } else { Some(a.pos("OUT")?) };
    a.done()?;
    println!("{}", takehead::to_euroc(Path::new(&d), out.as_deref().map(Path::new))?);
    Ok(())
}

fn cmd_bench_basalt(mut a: Args) -> R<()> {
    let (ds, est) = replay_inputs(&mut a)?;
    a.done()?;
    print!("{}", winbench::basalt_fast(&ds.imu, &est));
    Ok(())
}

fn cmd_bench_windows(mut a: Args) -> R<()> {
    let root = a.opt("--root")?.map(PathBuf::from).unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../pimax-data/windows-20261015")
    });
    let mut p = winbench::OursParams::default();
    p.latency_ms = a.num("--latency-ms", p.latency_ms)?;
    p.predict_ms = a.num("--predict-ms", p.predict_ms)?;
    p.fast_dps = a.num("--fast-dps", p.fast_dps)?;
    let tsv = a.opt("--tsv")?;
    let dump = a.opt("--dump")?;
    let take = a.pos("TAKE")?;
    a.done()?;
    let reference = a.opt("--reference")?.unwrap_or_else(|| "slow-look".into());
    let (pcap, poses) = winbench::find_take(&root, &take)?;
    let rf = if take == reference {
        None
    } else {
        let (rp, rq) = winbench::find_take(&root, &reference)?;
        Some(winbench::load_take(&rp, &rq, None)?.fit)
    };
    let t = winbench::load_take(&pcap, &poses, rf.as_ref())?;
    if let Some(f) = dump {
        winbench::dump(&t, Path::new(&f)).map_err(|e| format!("{f}: {e}"))?;
    }
    let text = winbench::report(&take, &t, &p);
    print!("{text}");
    if let Some(f) = tsv {
        std::fs::write(&f, winbench::tsv_row(&take, &t, &p)).map_err(|e| format!("{f}: {e}"))?;
    }
    Ok(())
}

fn main() {
    let mut v: Vec<String> = std::env::args().skip(1).collect();
    if v.is_empty() || v[0] == "-h" || v[0] == "--help" {
        eprintln!(
            "{}",
            include_str!("main.rs")
                .lines()
                .take_while(|l| l.starts_with("//!"))
                .map(|l| l.trim_start_matches("//!").trim_start_matches(' '))
                .collect::<Vec<_>>()
                .join("\n")
        );
        std::process::exit(2);
    }
    let cmd = v.remove(0);
    let a = Args(v);
    let r = match cmd.as_str() {
        "export" => cmd_export(a),
        "check" => cmd_check(a),
        "imu-noise" => cmd_imu_noise(a),
        "timing" => cmd_timing(a),
        "ate" => cmd_ate(a),
        "loop" => cmd_loop(a),
        "smoke-euroc" => cmd_smoke(a),
        "stats" => cmd_stats(a),
        "simulate" => cmd_simulate(a),
        "stillgate" => cmd_stillgate(a),
        "corrections" => cmd_corrections(a),
        "ae" => cmd_ae(a),
        "shift" => cmd_shift(a),
        "bench-windows" => cmd_bench_windows(a),
        "bench-basalt" => cmd_bench_basalt(a),
        "take-head" => cmd_take_head(a),
        "take-euroc" => cmd_take_euroc(a),
        "camlag" => cmd_camlag(a),
        _ => Err(format!(
            "unknown command {cmd:?} (export, check, imu-noise, timing, ate, loop, stats, simulate, ae, shift, smoke-euroc, camlag)"
        )),
    };
    if let Err(e) = r {
        eprintln!("omarchyvr-slam {cmd}: {e}");
        std::process::exit(1);
    }
}

/*
 * pimax_cam.h - C ABI of the Rust crate pimax-cam (rust/pimax-cam): Pimax Dream Air tracking cameras
 * (UVC 34a4:0014). Capture, tile split, frame metadata, IMU-clock timestamps, factory calibration,
 * .pxcam recording/replay. Link libpimax_cam.a (cargo build --release -p pimax-cam).
 *
 * Frame layout: the device says YUYV 1280x832 but each byte is a grey pixel: 2560x832 grey, four 640x480 tiles
 * (factory camera ids 0..3 left to right), tile rows start at a per-tile offset given in the metadata, frames
 * alternate between tracking exposure (kind 1) and controller-LED exposure (kind 2), 50 fps total.
 *
 * Threading: a pimax_cam handle is used by one thread at a time. Errors: functions return -1 / NULL and
 * pimax_cam_last_error() gives a thread-local message.
 *
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pimax_cam pimax_cam;

enum pimax_cam_kind
{
	PIMAX_CAM_KIND_UNKNOWN = 0,
	PIMAX_CAM_KIND_TRACKING = 1, /* frame_type 0x86: SLAM/hand exposure */
	PIMAX_CAM_KIND_LED = 2,      /* frame_type 0x06: short controller-LED exposure */
};

#define PIMAX_CAM_FLAG_META_VALID 1u
#define PIMAX_CAM_FLAG_ROWS_SCANNED 2u
#define PIMAX_CAM_FLAG_REPLAY 4u

#define PIMAX_CAM_REPLAY_LOOP 1u
#define PIMAX_CAM_REPLAY_REALTIME 2u

#define PIMAX_CAM_TILE_W 640
#define PIMAX_CAM_TILE_H 480
#define PIMAX_CAM_TILES 4

typedef struct pimax_cam_open_params
{
	uint32_t fps;          /* 0 = driver default (the camera runs at 50 fps either way) */
	uint32_t replay_flags; /* PIMAX_CAM_REPLAY_* */
	uint32_t verify_rows;  /* 1 = check tile rows against the picture every frame (diagnostics) */
	uint32_t reserved;
} pimax_cam_open_params;

typedef struct pimax_cam_frame_info
{
	uint64_t host_ts_ns; /* CLOCK_MONOTONIC when the driver saw the frame (rebased for replays) */
	uint64_t dev_ts_us;  /* metadata timestamp, MCU microseconds (the IMU reports its low 32 bits) */
	uint64_t mono_ts_ns; /* dev_ts mapped to CLOCK_MONOTONIC by the minimum-delay fallback filter */
	uint32_t counter;    /* metadata frame counter */
	uint32_t sequence;   /* V4L2 sequence */
	uint32_t dropped;    /* frames lost just before this one */
	uint32_t kind;       /* enum pimax_cam_kind */
	uint32_t frame_type; /* raw byte 32 */
	uint32_t stream_index;
	uint16_t exposure[4];
	uint16_t gain[4];
	uint16_t y_offset[4]; /* first frame row of each tile */
	uint32_t tile_width;  /* 640 */
	uint32_t tile_height; /* 480 */
	uint32_t flags;       /* PIMAX_CAM_FLAG_* */
	uint32_t reserved;
} pimax_cam_frame_info;

typedef struct pimax_cam_stats
{
	uint64_t frames, tracking, led, unknown;
	uint64_t dropped;      /* counter gaps */
	uint64_t corrupt;      /* buffers flagged bad by the driver */
	uint64_t scanned_rows; /* frames whose tile rows had to be scanned */
	uint64_t row_mismatch; /* metadata rows disagreeing with the picture (verify_rows only) */
	uint64_t kind_repeats; /* tracking/LED alternation broken */
	int64_t clock_offset_ns;
} pimax_cam_stats;

/* IMU clock reference: mono_ns = dev_ext_us * 1000 + offset_ns for the IMU sample with timestamp dev_ts_u32. */
typedef struct pimax_cam_imu_clock_ref
{
	uint32_t dev_ts_u32;
	uint32_t valid;
	uint64_t dev_ext_us;
	int64_t offset_ns;
} pimax_cam_imu_clock_ref;

typedef struct pimax_cam_kb4
{
	uint32_t width, height;
	double fx, fy, cx, cy;
	double k[4];     /* Kannala-Brandt k1..k4 (OpenCV fisheye, T_DISTORTION_FISHEYE_KB4) */
	double extra[2]; /* the XML's 5th/6th numbers, not used by KB4 */
} pimax_cam_kb4;

typedef struct pimax_cam_camera
{
	char name[32];
	int32_t id;
	uint32_t reserved;
	pimax_cam_kb4 intr;
	double rig_rotation[9];    /* row major, X_i = R X_0 + t */
	double rig_translation[3]; /* metres */
	double time_delta_s;
} pimax_cam_camera;

typedef struct pimax_cam_calib
{
	uint32_t camera_count;
	uint32_t has_imu;
	pimax_cam_camera cameras[4];
	double ombc[3]; /* rotation vector, camera 0 -> body (body = OpenXR head axes) */
	double tbc[3];  /* camera 0 origin in the body frame, metres */
	double imu_cam_delta_s;
	char device_uid[32];
} pimax_cam_calib;

const char *
pimax_cam_version(void);
const char *
pimax_cam_last_error(void);

/* source: NULL, "" or "auto" = find the camera; "/dev/videoN"; or a .pxcam recording. params may be NULL. */
pimax_cam *
pimax_cam_open(const char *source, const pimax_cam_open_params *params);
void
pimax_cam_close(pimax_cam *cam);
size_t
pimax_cam_describe(const pimax_cam *cam, char *buf, size_t len);

/* 1 = frame, 0 = timeout, -1 = error or end of a replay. */
int
pimax_cam_next(pimax_cam *cam, int timeout_ms, pimax_cam_frame_info *info);
/* Copy tile 0..3 of the current frame: 480 rows of 640 bytes, row pitch `stride`. 0 or -1. */
int
pimax_cam_copy_tile(const pimax_cam *cam, uint32_t tile, uint8_t *dst, uint32_t stride);
int
pimax_cam_get_stats(const pimax_cam *cam, pimax_cam_stats *out);

/* Record every following raw frame (writer thread; zlib_level 0 = raw, 1..9 = zlib). */
int
pimax_cam_record_start(pimax_cam *cam, const char *path, uint32_t zlib_level);
/* Frames written, or -1. */
int64_t
pimax_cam_record_stop(pimax_cam *cam);

/* path NULL = $PIMAX_CAM_CALIB, $PIMAX_CALIB, ~/.config/omarchyvr/headset-calibration.xml,
 * <repo>/analysis/headset-calibration.xml (repo = $PIMAX_REPO,
 * ~/.local/share/omarchyvr/data or ~/omarchyvr). used_path may be NULL. */
int
pimax_cam_calib_load(const char *path, pimax_cam_calib *out, char *used_path, size_t used_len);
int
pimax_cam_calib_parse(const char *xml, size_t len, pimax_cam_calib *out);
/* X_b = R X_a + t (OpenCV camera axes, R row major): t_stereo_camera_calibration camera_rotation/translation. */
int
pimax_cam_calib_stereo(const pimax_cam_calib *cal, uint32_t a, uint32_t b, double r[9], double t[3]);
/* X_body = R X_cam + t, body = OpenXR head axes (x right, y up, z back), camera = OpenCV axes. -1 without ombc. */
int
pimax_cam_calib_body_from_cam(const pimax_cam_calib *cal, uint32_t i, double r[9], double t[3]);

/* Device microseconds -> CLOCK_MONOTONIC ns using the HMD's IMU clock reference (0 if ref invalid). */
uint64_t
pimax_cam_dev_to_mono_ns(uint64_t dev_ts_us, const pimax_cam_imu_clock_ref *ref);

/* ---- 6DoF / SLAM support (src/slam.rs, src/capi_slam.rs) -------------------------------------------------------
 * IMU frame for the tracker = the calibration's body frame = OpenXR head axes (x right, y up, z back) = raw IMU
 * axes with x and z negated (the HMD driver's PIMAX_AXIS_MODE=1). Camera frames = OpenCV axes. */

typedef struct pimax_cam_slam_imu
{
	double accel_noise_std[3]; /* m/s^2/sqrt(Hz) */
	double gyro_noise_std[3];  /* rad/s/sqrt(Hz) */
	double accel_bias_std[3];
	double gyro_bias_std[3];
	double rate_hz;
	char source[256]; /* "defaults" or the file used */
} pimax_cam_slam_imu;

/* path NULL = $PIMAX_SLAM_IMU or ~/.config/omarchyvr/slam-imu.json if present. 1 = file, 0 = defaults, -1 = bad file
 * (defaults written). */
int
pimax_cam_slam_imu_load(const char *path, pimax_cam_slam_imu *out);
/* T_imu_cam of factory camera `cam`, column-major 4x4 (xrt_matrix_4x4 layout). 0 or -1. */
int
pimax_cam_slam_t_imu_cam(const pimax_cam_calib *cal, uint32_t cam, double out[16]);
/* Default head origin in the IMU frame (between the lower tracking cameras, 5 cm behind). 0 or -1. */
int
pimax_cam_slam_default_head(const pimax_cam_calib *cal, double out[3]);
/* "0,1" / "0,1,2,3" -> ids (distinct, < count). Returns how many, or -1. */
int
pimax_cam_slam_parse_cams(const char *s, uint32_t count, uint32_t *out, uint32_t cap);
/* Write dir/pimax-basalt-calib.json (Basalt cereal format, kb4) and dir/pimax-slam-info.json. 0 or -1. */
int
pimax_cam_slam_write_basalt(const pimax_cam_calib *cal,
                            const uint32_t *cams,
                            uint32_t n,
                            double cam_hz,
                            int64_t cam_time_offset_ns,
                            const pimax_cam_slam_imu *imu,
                            const char *dir,
                            const char *note);

/* Pose supervisor: when to trust the tracker, continuity across losses, reset requests. */
typedef struct pimax_cam_track pimax_cam_track;

typedef struct pimax_cam_track_params
{
	double max_speed_m_s;        /* faster between two poses = jump (4) */
	double max_rot_mismatch_deg; /* tracker vs gyro relative rotation between two poses (8) */
	uint32_t acquire_poses;      /* consecutive consistent poses to (re)acquire (12) */
	uint32_t reserved;
	uint64_t stale_ns;       /* no new pose for this long = lost (350 ms) */
	uint64_t reset_after_ns; /* lost this long = ask for a reset (3 s, 0 = never) */
	uint64_t reset_every_ns; /* at most one request per interval (6 s) */
} pimax_cam_track_params;

typedef struct pimax_cam_track_sample
{
	uint64_t ts_ns;
	double pos[3];   /* IMU position in the tracker world (y up) */
	double q[4];     /* IMU orientation in the tracker world, x y z w */
	double q3dof[4]; /* 3DoF IMU orientation at ts_ns, x y z w */
} pimax_cam_track_sample;

#define PIMAX_CAM_TRACK_INIT 0u
#define PIMAX_CAM_TRACK_TRACKING 1u
#define PIMAX_CAM_TRACK_LOST 2u
#define PIMAX_CAM_TRACK_EV_ACQUIRED 1u
#define PIMAX_CAM_TRACK_EV_LOST 2u
#define PIMAX_CAM_TRACK_EV_RESET_WANTED 4u
#define PIMAX_CAM_TRACK_EV_RECENTERED 8u
#define PIMAX_CAM_TRACK_EV_RESNAPPED 16u /* tracking back in the same tracker world: old room alignment kept */
#define PIMAX_CAM_TRACK_EV_REZEROED 32u /* drifted while lying still: tracker reset, output held, no loss */

typedef struct pimax_cam_track_out
{
	uint32_t state;  /* PIMAX_CAM_TRACK_* */
	uint32_t events; /* PIMAX_CAM_TRACK_EV_* raised by this update */
	double world_from_slam_pos[3];
	double world_from_slam_q[4]; /* yaw only, x y z w */
	double fix3dof_q[4];         /* correction of the 3DoF orientation while not tracking (yaw; full on a re-zero) */
	double held_pos[3];          /* head position while not tracking */
	uint64_t acquisitions, losses, resets, rejects;
	char reason[48];
} pimax_cam_track_out;

void
pimax_cam_track_default_params(pimax_cam_track_params *out);
/* params NULL = defaults; head_in_imu NULL = origin. */
pimax_cam_track *
pimax_cam_track_create(const pimax_cam_track_params *params, const double head_in_imu[3]);
void
pimax_cam_track_destroy(pimax_cam_track *t);
/* Head to the origin at the next update, heading unchanged. */
void
pimax_cam_track_recenter(pimax_cam_track *t);
/* sample NULL = no tracker pose. 0 or -1. (Phase-1 entry point, no velocity; prefer _update_vio.) */
int
pimax_cam_track_update(pimax_cam_track *t, uint64_t now_ns, const pimax_cam_track_sample *sample,
                       pimax_cam_track_out *out);

/* Phase 2 (src/headpose.rs): the handle also predicts the head pose at IMU rate from the newest tracker pose and
 * blends every correction (new tracker pose, acquire/lose, re-snap, recenter) so the output never jumps:
 * the difference goes into an offset that decays (PIMAX_SLAM_TAU_MS, default 150 ms position / 100 ms rotation),
 * rate limited (0.1 m/s + half the head speed; 3 deg/s + 0.3 x angular speed). PIMAX_SLAM_SMOOTH=0 = raw.
 * PIMAX_SLAM_RESNAP=0 disables restoring the room after a loss in the same tracker world. */
typedef struct pimax_cam_track_vio
{
	uint64_t ts_ns;
	double pos[3];   /* IMU position in the tracker world (y up) */
	double q[4];     /* IMU orientation, x y z w */
	double vel[3];   /* IMU velocity in the tracker world */
	double q3dof[4]; /* 3DoF IMU orientation at ts_ns */
} pimax_cam_track_vio;

/* A relation in the y-up world; flags = xrt_space_relation_flags bits. */
typedef struct pimax_cam_rel
{
	double pos[3];
	double q[4];
	double lin_vel[3];
	double ang_vel[3];
	uint32_t flags;
	uint32_t reserved;
} pimax_cam_rel;

typedef struct pimax_cam_track_info
{
	double gyro_bias[3]; /* learned from the tracker, rad/s, IMU frame */
	double corr_pos[3];  /* current correction offset, m */
	double corr_rot[3];  /* rad (rotation vector, world) */
	double max_corr_pos, max_corr_rot;
	uint64_t blended, snapped;
	int64_t last_imu_ns;
} pimax_cam_track_info;

/* One output correction step of at least PIMAX_SLAM_BIG_CORR_MM (default 20 mm), y-up world ([1] = vertical). */
typedef struct pimax_cam_track_corr
{
	int64_t ts_ns;
	double step[3];   /* this change, m */
	double offset[3]; /* the whole pending offset after it, m */
	double rot;       /* rotation offset, rad */
	double speed;     /* head speed, m/s */
	double ang_speed; /* rad/s */
	uint32_t pose_only; /* 1 = a new tracker pose only, 0 = supervisor change */
	uint32_t reserved;
} pimax_cam_track_corr;

/* Oldest queued big correction (the last 64 are kept): 1 = filled, 0 = none, -1 = error. */
int
pimax_cam_track_take_big_corr(pimax_cam_track *t, pimax_cam_track_corr *out);

/* Every IMU sample (IMU frame, m/s^2, rad/s, raw, same clock as the tracker). */
void
pimax_cam_track_push_imu(pimax_cam_track *t, int64_t ts_ns, const double accel[3], const double gyro[3]);
/* Supervisor + predictor update with the newest tracker pose (sample NULL = none). 0 or -1. */
int
pimax_cam_track_update_vio(pimax_cam_track *t, uint64_t now_ns, const pimax_cam_track_vio *sample,
                           pimax_cam_track_out *out);
/* Head pose at at_ns; rel3dof = the 3DoF relation for the same time. Returns PIMAX_CAM_TRACK_* or -1. */
int
pimax_cam_track_head(pimax_cam_track *t, uint64_t now_ns, int64_t at_ns, const pimax_cam_rel *rel3dof,
                     pimax_cam_rel *out);
int
pimax_cam_track_get_info(const pimax_cam_track *t, pimax_cam_track_info *out);

/* ---- Auto-exposure (src/ae.rs, src/capi_ae.rs) -----------------------------------------------------------------
 * Tracking-camera AE: histogram of the four tiles (pairs 0,1 and 2,3 pooled, fisheye circle only, top 2 % left
 * out), damped multiplicative step relative to the frame's own metadata exposure*gain, exposure first up to a
 * motion-blur cap, then gain. The result is the payload of the HID exposure push Windows sends at 12.5 Hz:
 *   58 19 51 00 | 01 32 | e0 e1 e2 e3 | g0 g1 g2 g3 | led_e led_g   (u16 LE; LED pair constant 40/62)
 * This library never sends anything: the driver decides (PIMAX_CAM_AE=1 sends, "dry" only logs, default off).
 * Tuning: PIMAX_CAM_AE_TARGET, _EXP_MIN, _EXP_MAX, _GAIN_MIN, _GAIN_MAX, _PERIOD_MS, _K, _DEADBAND, _UPPER, _EXTEND (0 = no automatic
 * extended range in low light), _MODEL (dream-air | crystal-super: the headset's ladder, gain codes and limits). */

#define PIMAX_CAM_AE_CMD 0x51
#define PIMAX_CAM_AE_OPT 0x00
#define PIMAX_CAM_AE_PAYLOAD_LEN 22

typedef struct pimax_cam_ae pimax_cam_ae;

typedef struct pimax_cam_ae_out
{
	uint32_t send;    /* 1 = send payload now (cmd 0x51 opt 0) */
	uint32_t changed; /* 1 = target changed in this update */
	uint64_t frames, sends, changes;
	uint8_t payload[PIMAX_CAM_AE_PAYLOAD_LEN];
	uint8_t pad[2];
	uint16_t exposure[4], gain[4]; /* target */
	uint16_t led_exposure, led_gain;
	uint16_t frame_exposure[4], frame_gain[4]; /* what the analysed frame was taken with (metadata) */
	float mean[2];                             /* trimmed mean, lower / upper pair (0..255) */
	float sat[2];                              /* saturated fraction per pair */
	float err[2];                              /* log error per pair, 0 inside the deadband */
	uint32_t light; /* low light: bits 0..1 range stage (0 normal, 1 gain extended, 2 gain+exposure extended),
	                   bit 2 = "tracking light low" warning (the library already logs and toasts it) */
} pimax_cam_ae_out;

/* Defaults of the headset on USB (34a4:0040 = Crystal Super, else Dream Air; PIMAX_CAM_AE_MODEL=dream-air|crystal-super
 * overrides) + PIMAX_CAM_AE_* overrides. Logs "PIMAX_AE_MODEL: <id> (...)". NULL on error. */
pimax_cam_ae *
pimax_cam_ae_create(void);
/* The same for a named model ("dream-air", "crystal-super"; NULL or "" = detect as above). The Crystal Super uses the
 * Windows ranges of its capture (lower pair 2200..10410 us / gain 13..449, upper pair 2200..8941 / 13..439) and never
 * hands out a payload outside them. NULL on an unknown name. */
pimax_cam_ae *
pimax_cam_ae_create_model(const char *model);
void
pimax_cam_ae_destroy(pimax_cam_ae *ae);
/* After pimax_cam_next returned a frame: 1 = send out->payload now, 0 = nothing (LED frame, replay, no metadata,
 * not due), -1 = error. */
int
pimax_cam_ae_update(pimax_cam_ae *ae, const pimax_cam *cam, uint64_t now_ns, pimax_cam_ae_out *out);

#ifdef __cplusplus
}
#endif

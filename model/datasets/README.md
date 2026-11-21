# model/datasets/

Recorded datasets for the golden models (Stage 1). The frames live outside git; this folder holds the format spec and, later, `manifest.json`, which lists each session with where it's stored and what it covers.

## Recording with the CPU pipeline

The CPU pipeline records when `GOMPEI_RECORD_DIR` is set in the environment of the `GompeiVision` service (see [`DatasetRecorder.h`](../../src/include/io/DatasetRecorder.h)):

```bash
GOMPEI_RECORD_DIR=/mnt/ssd/datasets   # where sessions go (needs its own disk; ~1 MB per 1280x800 frame)
GOMPEI_RECORD_EVERY=2                 # optional: keep 1 frame in N (default 1)
```

- **Never slows the tag path.** Recording happens on its own thread. If the disk can't keep up, frames are dropped and counted rather than slowing detection; the count is logged when the pipeline stops.
- **Disk-safe.** It stops by itself when free space falls below 2 GiB.
- **Volume.** At 1 MB per frame, 30 FPS is about 30 MB/s, or roughly 1.8 GB per minute.

What to record for Stage 1:
- **Motion:** robot still, driving, and fast turns.
- **Distance:** near and far tags.
- **Lighting:** field lighting and practice-room lighting.
- **Hard cases worth keeping:** tags against dark backgrounds, partly hidden tags, and steep angles.

## Session format (version 1)

```
<hardware_id>_<role>_<UTC time>/
  session.json
  frames.jsonl
  frames/000000.pgm, 000001.pgm, ...
```

**`session.json`** holds:
- `hardware_id` and `role`;
- `apriltag_version`, plus every `detector` setting (`quad_decimate`, `nthreads`, `refine_edges`, the `qtp` thresholds and so on);
- `tag_family` (`tag36h11`), `tag_size_m` and `record_every`;
- `field`: the tag map at session start, as `{id: {x, y, z, qw, qx, qy, qz}}`.

**`frames.jsonl`** has one JSON object per line, of two kinds:

- **Calibration**, written before the first frame and again whenever it changes:
  `{"calibration": {"camera_matrix": [[...]], "distortion_coefficients": [[...]]}}`
- **Frame:**

  ```json
  {"file": "frames/000123.pgm", "timestamp": 123456789, "processed_index": 246,
   "width": 1280, "height": 800,
   "detections": [{"id": 7, "corners": [x1, y1, x2, y2, x3, y3, x4, y4]}],
   "single_tag_poses": [{"id": 7, "pose_0": {...}, "error_0": 0.01, "distance_m": 2.3,
                         "pose_1": {...}, "error_1": 0.02}],
   "camera_pose": {"tags": [7, 8], "pose_0": {...}, "error_0": 0.01}}
  ```

  - `timestamp` is `nt::Now()` in microseconds, taken when the frame was captured.
  - `processed_index` counts every frame the pipeline processed, so gaps show skipped (`record_every`) or dropped frames.
  - `detections` are AprilTag's output in its own corner order and pixel convention.
  - The pose fields are present only when the pipeline computed poses (tags seen and calibration loaded).

**`frames/*.pgm`** are binary 8-bit grayscale: exactly the image passed to `apriltag_detector_detect`, so the golden models replay bit for bit. The model tools read them with `gv::model::read_pgm`.

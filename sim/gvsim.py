"""Shared helpers for the cocotb testbenches: PGM files, the golden model's
`gv-model dump`, and test images. Every expected result comes from the golden
model (model/), which is itself checked against AprilTag."""

import csv
import os
import random
import subprocess
import tempfile
from pathlib import Path


def gv_model() -> str:
    return os.environ["GV_MODEL"]


def write_pgm(path: Path, w: int, h: int, pixels: bytes) -> None:
    path.write_bytes(b"P5\n%d %d\n255\n" % (w, h) + pixels)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    parts = path.read_bytes().split(maxsplit=4)
    assert parts[0] == b"P5" and parts[3] == b"255"
    w, h = int(parts[1]), int(parts[2])
    return w, h, parts[4][: w * h]


def dump(w: int, h: int, pixels: bytes, *args: str) -> dict:
    """Run `gv-model dump` on an image; returns its outputs."""
    with tempfile.TemporaryDirectory() as tmp:
        t = Path(tmp)
        write_pgm(t / "in.pgm", w, h, pixels)
        subprocess.run([gv_model(), "dump", str(t / "in.pgm"), str(t / "out"), *args], check=True, capture_output=True)
        _, _, thresh = read_pgm(t / "out" / "threshold.pgm")
        with open(t / "out" / "components.csv", newline="") as f:
            components = list(csv.DictReader(f))
        return {"threshold": thresh, "components": components}


def noise(rng: random.Random, w: int, h: int, lo: int = 0, hi: int = 255) -> bytes:
    return bytes(rng.randint(lo, hi) for _ in range(w * h))


def blocks(rng: random.Random, w: int, h: int, n: int = 30) -> bytes:
    img = bytearray([rng.randint(0, 255)]) * (w * h)
    for _ in range(n):
        x0, y0 = rng.randrange(w), rng.randrange(h)
        x1, y1 = min(w, x0 + rng.randint(1, max(1, w // 3))), min(h, y0 + rng.randint(1, max(1, h // 3)))
        v = rng.randint(0, 255)
        for y in range(y0, y1):
            img[y * w + x0 : y * w + x1] = bytes([v]) * (x1 - x0)
    return bytes(img)


def gradient_with_squares(rng: random.Random, w: int, h: int) -> bytes:
    """A lighting gradient with dark squares on light frames, roughly tag-like."""
    img = bytearray(w * h)
    for y in range(h):
        for x in range(w):
            img[y * w + x] = max(0, min(255, 90 + (x * 60) // w + (y * 40) // h + rng.randint(-4, 4)))
    for _ in range(6):
        s = rng.randint(8, max(9, min(w, h) // 3))
        x0, y0 = rng.randrange(max(1, w - s)), rng.randrange(max(1, h - s))
        for y in range(y0, min(h, y0 + s)):
            for x in range(x0, min(w, x0 + s)):
                inner = s // 6 <= x - x0 < s - s // 6 and s // 6 <= y - y0 < s - s // 6
                img[y * w + x] = rng.randint(20, 50) if inner else rng.randint(190, 230)
    return bytes(img)


def synth_scene(path: Path, seed: int, w: int, h: int, tags: int, tag_px: tuple[float, float]) -> None:
    """A synthetic tag36h11 scene from the golden model's generator."""
    subprocess.run(
        [gv_model(), "synth", str(path), "--seed", str(seed), "--size", str(w), str(h), "--tags", str(tags),
         "--tag-px", str(tag_px[0]), str(tag_px[1])],
        check=True, capture_output=True,
    )


def edge_jobs(image: Path) -> list[dict]:
    """Every edge job the detector gives the PL edge sampler, with its expected
    points, at the Stage 4 fixed-point widths (`gv-model edges`)."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "edges.csv"
        subprocess.run([gv_model(), "edges", str(image), str(out)], check=True, capture_output=True)
        with open(out, newline="") as f:
            rows = list(csv.DictReader(f))
    jobs = []
    for r in rows:
        job = {k: int(v) for k, v in r.items() if k != "points"}
        job["points"] = [tuple(int(c) for c in p.split(":")) for p in r["points"].split()] if r["points"] else []
        jobs.append(job)
    return jobs


def lookup(codes: list[int], max_bits: int = 3) -> list[tuple | None]:
    """The golden ID lookup (Tag36h11Lookup::decode) for each 36-bit read:
    (id, rotation, hamming), or None for no match."""
    out = subprocess.run(
        [gv_model(), "lookup", "--max-bits", str(max_bits)],
        input="".join(f"{c:09x}\n" for c in codes), check=True, capture_output=True, text=True,
    ).stdout.split("\n")
    return [None if line == "-" else tuple(int(v) for v in line.split()) for line in out if line]


def code_reads(image: Path) -> list[dict]:
    """Every code read the detector makes at the Stage 4 widths (the PL
    projector's and tag sampler's work): the projector's 8 coefficients
    (h0..h5 Q.14, h6, h7 Q.24), its 100 sample points (Q.8) and the expected
    ok, code and margin (Q.4), from `gv-model decodes`. Reads whose projector
    the RPU would reject (no points) are left out."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "decodes.csv"
        subprocess.run([gv_model(), "decodes", str(image), str(out)], check=True, capture_output=True)
        with open(out, newline="") as f:
            rows = list(csv.DictReader(f))
    reads = []
    for r in rows:
        if int(r["projected"]) != 1:
            continue
        pts = [tuple(int(c) for c in p.split(":")) for p in (r["border"] + " " + r["cells"]).split()]
        reads.append({"h": [int(v) for v in r["h"].split()], "points": pts, "ok": int(r["ok"]) == 1, "code": int(r["code"], 16), "margin": int(r["margin"])})
    return reads


def scorecard_quads(records: list[tuple]) -> list[tuple]:
    """The golden 4B corner scorecard (`gv-model quads`) for each component
    record (area, x_min, x_max, y_min, y_max, 16 (x, y) extremes): (verdict,
    4 corners, twice the area), or (1, None, None) for too few corners."""
    lines = [" ".join(map(str, [*r[:5], *(v for p in r[5] for v in p)])) for r in records]
    out = subprocess.run([gv_model(), "quads"], input="\n".join(lines) + "\n", check=True, capture_output=True,
                         text=True).stdout.split("\n")
    results = []
    for line in out:
        if not line:
            continue
        v = [int(x) for x in line.split()]
        results.append((v[0], None, None) if v[0] == 1 else (v[0], list(zip(v[1:9:2], v[2:9:2])), v[9]))
    assert len(results) == len(records)
    return results


def read_tags(image: Path, coeffs: list[list[int]]) -> list[dict | None]:
    """The golden decode (projector, tag sampler, ID lookup) for homographies as
    8 coefficients each (h0..h5 Q.14, h6, h7 Q.24), from `gv-model read`: ok,
    code, margin (Q.4) and match (id, rotation, hamming) or None. None for a
    projector the RPU wouldn't send."""
    out = subprocess.run([gv_model(), "read", str(image)], check=True, capture_output=True, text=True,
                         input="".join(" ".join(map(str, c)) + "\n" for c in coeffs)).stdout.split("\n")
    res = []
    for line in out:
        f = line.split()
        if not f:
            continue
        res.append(None if f[0] == "invalid" else {
            "ok": f[0] == "1", "code": int(f[1], 16), "margin": int(f[2]),
            "match": None if f[3] == "-" else tuple(int(v) for v in f[3:6])})
    assert len(res) == len(coeffs)
    return res


JOB_FIELDS = ["a_x", "a_y", "d_x", "d_y", "n_x", "n_y", "t0", "spacing", "count", "half_steps", "window_steps",
              "check_direction"]


def sample_edges(image: Path, jobs: list[dict]) -> list[list[tuple]]:
    """The golden edge sampler's points for arbitrary jobs (`gv-model sample`)."""
    out = subprocess.run([gv_model(), "sample", str(image)], check=True, capture_output=True, text=True,
                         input="".join(" ".join(str(j[k]) for k in JOB_FIELDS) + "\n" for j in jobs)).stdout
    lines = out.split("\n")[:-1]
    assert len(lines) == len(jobs)
    return [[] if line == "-" else [tuple(int(c) for c in p.split(":")) for p in line.split()] for line in lines]


def detect(image: Path) -> list[tuple]:
    """The golden detector's tags at the Stage 4 widths, without 4C: (id,
    hamming, margin Q.4, cut, 4 corners Q.8), from `gv-model detect`."""
    out = subprocess.run([gv_model(), "detect", str(image)], check=True, capture_output=True, text=True).stdout
    tags = []
    for line in out.splitlines():
        v = [int(x) for x in line.split()]
        tags.append((*v[:4], list(zip(v[4::2], v[5::2]))))
    return tags

"""
Assemble the PNG frames recorded by run_twin.py into one mp4 per camera (host side, OpenCV).

Usage
-----
    python simulation/isaac/frames_to_mp4.py --out_dir output/twin_res5 [--fps 10]
    # -> out_dir/recording/<cam>.mp4   (fps = run_twin --record_fps plays back in sim time)
"""

import argparse
import glob
import os

import cv2


def main():
    ap = argparse.ArgumentParser(description="out_dir/recording/<cam>/%05d.png -> out_dir/recording/<cam>.mp4")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--fps", type=float, default=10.0)
    args = ap.parse_args()
    for cam_dir in sorted(glob.glob(os.path.join(args.out_dir, "recording", "*", ""))):
        frames = sorted(glob.glob(os.path.join(cam_dir, "*.png")))
        if not frames:
            continue
        h, w = cv2.imread(frames[0]).shape[:2]
        out = cam_dir.rstrip(os.sep) + ".mp4"
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
        for f in frames:
            vw.write(cv2.imread(f))
        vw.release()
        print(f"[Video] {len(frames)} frames -> {out}")


if __name__ == "__main__":
    main()

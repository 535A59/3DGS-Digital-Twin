"""
Semantic Segmentation via SAM (Segment Anything Model).

Interactive 2D segmentation of video frames using point prompts.
Saves segmentation masks as JSON polygon files for downstream 3D lifting.

Usage:
    python -m src.semantic_segmentation.sam_segmentation \
        --input_folder /path/to/images \
        --output_folder /path/to/masks_json \
        --sam_checkpoint /path/to/sam_vit_h_4b8939.pth
"""

import cv2
import numpy as np
import os
import glob
import json
import argparse

try:
    import torch
    from segment_anything import sam_model_registry, SamPredictor
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False


# Global state for interactive segmentation
latest_mask = None
display_image = None


def show_mask(mask: np.ndarray, image: np.ndarray) -> np.ndarray:
    """Overlay a semi-transparent mask on the image for preview."""
    color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])

    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)

    mask_image_bgra = mask_image[..., [2, 1, 0, 3]] * 255
    mask_image_bgra = mask_image_bgra.astype(image.dtype)

    alpha = mask_image_bgra[..., 3] / 255.0
    alpha = alpha[..., np.newaxis]

    blended = (1 - alpha) * image + alpha * mask_image_bgra[..., :3]
    return blended.astype(image.dtype)


def process_image(image_path: str, predictor, output_folder: str) -> str:
    """
    Interactively segment a single image with SAM.

    Controls:
        Left click  - add foreground point (green)
        Right click - add background/exclusion point (red)
        's' / Space - save current mask as JSON
        'z'         - undo last point
        'n'         - skip to next image
        'q'         - quit

    Returns:
        "exit" if user pressed 'q', "continue" otherwise.
    """
    global latest_mask, display_image

    image = cv2.imread(image_path)
    if image is None:
        print(f"[WARN] Cannot read image: {image_path}")
        return "continue"

    display_image = image.copy()
    original_image = image.copy()

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    predictor.set_image(image_rgb)

    points: list = []
    labels: list = []
    latest_mask = None

    def mouse_callback(event, x, y, flags, param):
        global latest_mask, display_image

        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            points.append([x, y])
            labels.append(1)
        else:
            points.append([x, y])
            labels.append(0)

        masks, _scores, _logits = predictor.predict(
            point_coords=np.array(points),
            point_labels=np.array(labels),
            multimask_output=False,
        )
        latest_mask = masks[0]

        display_image = show_mask(latest_mask, original_image)
        for i, pt in enumerate(points):
            color = (0, 255, 0) if labels[i] == 1 else (0, 0, 255)
            cv2.circle(display_image, tuple(pt), 5, color, -1)

    window_name = "SAM Interactive Segmentation"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print(f"\n--- Image: {os.path.basename(image_path)} ---")
    print("  Left click : foreground point (green)")
    print("  Right click: background point (red)")
    print("  's'/Space  : save mask as JSON")
    print("  'z'        : undo last point")
    print("  'n'        : next image")
    print("  'q'        : quit")

    while True:
        cv2.imshow(window_name, display_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            cv2.destroyAllWindows()
            return "exit"

        if key == ord('n'):
            break

        elif key in (ord('s'), ord(' ')):
            if latest_mask is None:
                print("[INFO] No mask yet — click on the image first.")
                continue

            mask_uint8 = (latest_mask * 255).astype(np.uint8)
            contours, _ = cv2.findContours(
                mask_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            segmentation_polygons = []
            for contour in contours:
                if contour.shape[0] > 0:
                    segmentation_polygons.append(contour.squeeze(axis=1).tolist())

            h, w = original_image.shape[:2]
            base_filename = os.path.basename(image_path)
            filename, _ = os.path.splitext(base_filename)

            data_to_save = {
                "image_filename": base_filename,
                "image_height": h,
                "image_width": w,
                "input_points": np.array(points).tolist(),
                "input_labels": np.array(labels).tolist(),
                "segmentation_polygons": segmentation_polygons,
            }

            output_path = os.path.join(output_folder, f"{filename}_mask.json")
            with open(output_path, "w") as f:
                json.dump(data_to_save, f, indent=4)
            print(f"[INFO] Mask saved: {output_path}")

        elif key == ord('z'):
            if points:
                points.pop()
                labels.pop()
                print("[INFO] Undo last point.")

                if points:
                    masks, _, _ = predictor.predict(
                        point_coords=np.array(points),
                        point_labels=np.array(labels),
                        multimask_output=False,
                    )
                    latest_mask = masks[0]
                    display_image = show_mask(latest_mask, original_image)
                else:
                    latest_mask = None
                    display_image = original_image.copy()

                for i, pt in enumerate(points):
                    color = (0, 255, 0) if labels[i] == 1 else (0, 0, 255)
                    cv2.circle(display_image, tuple(pt), 5, color, -1)

    cv2.destroyAllWindows()
    return "continue"


class SAMSegmentor:
    """Wrapper around SAM for batch interactive segmentation."""

    def __init__(
        self,
        sam_checkpoint: str,
        model_type: str = "vit_h",
        device: str | None = None,
    ):
        if not SAM_AVAILABLE:
            raise ImportError(
                "segment_anything is not installed. "
                "Run: pip install git+https://github.com/facebookresearch/segment-anything.git"
            )

        if device is None:
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"

        print(f"[INFO] Loading SAM ({model_type}) on {device} ...")
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.to(device=device)
        self.predictor = SamPredictor(sam)
        print("[INFO] SAM loaded.")

    def run(self, input_folder: str, output_folder: str):
        """Interactively segment all images in input_folder."""
        os.makedirs(output_folder, exist_ok=True)

        extensions = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tiff")
        image_paths: list[str] = []
        for ext in extensions:
            image_paths.extend(glob.glob(os.path.join(input_folder, ext)))
        image_paths.sort()

        if not image_paths:
            print(f"[WARN] No images found in {input_folder}")
            return

        print(f"[INFO] Found {len(image_paths)} images.")

        for image_path in image_paths:
            status = process_image(image_path, self.predictor, output_folder)
            if status == "exit":
                print("[INFO] User exited.")
                break

        print("[INFO] Segmentation complete.")


def run_segmentation(
    input_folder: str,
    output_folder: str,
    sam_checkpoint: str,
    model_type: str = "vit_h",
    device: str | None = None,
):
    """Convenience entry point for running interactive segmentation."""
    segmentor = SAMSegmentor(sam_checkpoint, model_type, device)
    segmentor.run(input_folder, output_folder)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive SAM segmentation — saves masks as JSON polygons."
    )
    parser.add_argument("--input_folder", required=True, help="Folder with input images")
    parser.add_argument("--output_folder", required=True, help="Folder to save JSON masks")
    parser.add_argument(
        "--sam_checkpoint", default="sam_vit_h_4b8939.pth", help="Path to SAM weights"
    )
    parser.add_argument("--model_type", default="vit_h", help="SAM model type")
    parser.add_argument("--device", default=None, help="cuda / cpu (auto-detected if omitted)")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()

    if not os.path.exists(args.sam_checkpoint):
        print(
            f"[ERROR] SAM checkpoint not found: {args.sam_checkpoint}\n"
            "Download from https://github.com/facebookresearch/segment-anything"
        )
        raise SystemExit(1)

    if not os.path.isdir(args.input_folder):
        print(f"[ERROR] Input folder not found: {args.input_folder}")
        raise SystemExit(1)

    run_segmentation(
        args.input_folder,
        args.output_folder,
        args.sam_checkpoint,
        args.model_type,
        args.device,
    )

"""Supported instance-segmentation model registry."""
from pathlib import Path

MODEL_FAMILIES = {
    "yolo26-seg": "yolo26{size}-seg.pt",
    "yoloe-26-seg": "yoloe-26{size}-seg.pt",
    "yolo11-seg": "yolo11{size}-seg.pt",
    "yolov8-seg": "yolov8{size}-seg.pt",
}
MODEL_SIZES = ("n", "s", "m", "l", "x")
IMAGE_SIZES = (320, 416, 512, 640, 768, 960, 1024, 1280)


def resolve_model_checkpoint(family: str, size: str, custom_path: str = "") -> str:
    family = str(family).strip().lower()
    size = str(size).strip().lower()
    if size not in MODEL_SIZES:
        raise ValueError(f"model size must be one of {', '.join(MODEL_SIZES)}")
    custom_path = str(custom_path or "").strip()
    if family == "custom":
        if not custom_path:
            raise ValueError("Choose a custom segmentation checkpoint path")
        checkpoint = Path(custom_path).expanduser()
        if not checkpoint.is_file():
            raise ValueError(f"Custom checkpoint does not exist: {checkpoint}")
        return str(checkpoint)
    template = MODEL_FAMILIES.get(family)
    if template is None:
        raise ValueError("Unsupported segmentation model family")
    if custom_path:
        checkpoint = Path(custom_path).expanduser()
        if not checkpoint.is_file():
            raise ValueError(f"Checkpoint override does not exist: {checkpoint}")
        return str(checkpoint)
    # Return official Ultralytics model names. Ultralytics downloads these on first use.
    return template.format(size=size)

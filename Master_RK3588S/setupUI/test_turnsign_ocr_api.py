import argparse
import json
import os
import sys
import time

import cv2

from turnsign_ocr_api import (
    DEFAULT_MODEL,
    PaddleOcrReader,
    WenxinRouteInterpreter,
    crop_bbox,
    fixed_unknown_result,
)


def parse_bbox(text):
    if not text:
        return None
    parts = [item.strip() for item in str(text).split(",")]
    if len(parts) != 4:
        raise ValueError("--bbox must be left,top,right,bottom")
    return [float(item) for item in parts]


def read_image(path):
    image = cv2.imread(path)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    return image


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def print_json(payload):
    print(json.dumps(payload, ensure_ascii=False, indent=2))


def build_interpreter(args):
    return WenxinRouteInterpreter(
        access_token=args.access_token,
        model=args.model,
        timeout=args.timeout,
        temperature=args.temperature,
        log_func=lambda message: print(f"[turnsign-api] {message}", file=sys.stderr),
    )


def run_api_text(args):
    interpreter = build_interpreter(args)
    started = time.time()
    instruction = interpreter.interpret(args.text, ocr_confidence=args.ocr_confidence)
    return {
        "mode": "text_api",
        "elapsed_sec": round(time.time() - started, 3),
        "input_text": args.text,
        "instruction": instruction,
    }


def run_image_ocr_api(args):
    image = read_image(args.image)
    bbox = parse_bbox(args.bbox)
    crop = crop_bbox(image, bbox, pad_ratio=args.pad_ratio) if bbox else image
    if crop is None or crop.size == 0:
        raise RuntimeError("empty crop")

    if args.crop_out:
        ensure_parent_dir(args.crop_out)
        cv2.imwrite(args.crop_out, crop)

    reader = PaddleOcrReader(
        lang=args.ocr_lang,
        use_angle_cls=not args.no_angle_cls,
        model_root=args.ocr_model_root,
    )
    ocr_started = time.time()
    ocr = reader.read(crop)
    ocr_elapsed = round(time.time() - ocr_started, 3)

    instruction = fixed_unknown_result(ocr.get("text", ""), "api_not_called")
    api_elapsed = 0.0
    if not args.no_api:
        interpreter = build_interpreter(args)
        api_started = time.time()
        instruction = interpreter.interpret(
            ocr.get("text", ""),
            ocr_confidence=ocr.get("confidence", 0.0),
            context={
                "test_image": os.path.abspath(args.image),
                "bbox": bbox,
            },
        )
        api_elapsed = round(time.time() - api_started, 3)

    return {
        "mode": "image_ocr_api",
        "image": os.path.abspath(args.image),
        "bbox": bbox,
        "crop_out": os.path.abspath(args.crop_out) if args.crop_out else None,
        "ocr_elapsed_sec": ocr_elapsed,
        "api_elapsed_sec": api_elapsed,
        "ocr": ocr,
        "instruction": instruction,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Local TurnSign OCR + AI Studio API smoke test. Does not touch ar_receiver.py."
    )
    parser.add_argument("--image", help="Image or cropped TurnSign image to OCR.")
    parser.add_argument("--bbox", help="Optional crop bbox for --image: left,top,right,bottom.")
    parser.add_argument("--crop-out", help="Save cropped TurnSign image for inspection.")
    parser.add_argument("--text", help="Skip OCR and directly test AI Studio API with this text.")
    parser.add_argument("--no-api", action="store_true", help="Only run OCR; do not call AI Studio API.")
    parser.add_argument("--access-token", default=None, help="AI Studio access token. Can also use AISTUDIO_ACCESS_TOKEN.")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--ocr-confidence", type=float, default=1.0, help="Confidence used by --text mode.")
    parser.add_argument("--ocr-lang", default="ch")
    parser.add_argument("--no-angle-cls", action="store_true")
    parser.add_argument("--ocr-model-root", default=None, help="ASCII path containing PaddleOCR det/rec/cls model dirs.")
    parser.add_argument("--pad-ratio", type=float, default=0.12)
    parser.add_argument("--output-json", help="Write the full result to a JSON file.")
    args = parser.parse_args()

    if bool(args.image) == bool(args.text):
        parser.error("use exactly one of --image or --text")

    if not args.access_token:
        args.access_token = (
            os.environ.get("AISTUDIO_ACCESS_TOKEN")
            or os.environ.get("AR_TURNSIGN_ACCESS_TOKEN")
            or os.environ.get("EB_ACCESS_TOKEN")
            or os.environ.get("AI_STUDIO_ACCESS_TOKEN")
        )
    return args


def main():
    args = parse_args()
    try:
        if args.text:
            result = run_api_text(args)
        else:
            result = run_image_ocr_api(args)
    except Exception as exc:
        result = {
            "ok": False,
            "error": str(exc),
            "hint": (
                "Install dependencies with: pip install erniebot paddleocr. "
                "PaddleOCR also needs a compatible paddlepaddle wheel."
            ),
        }
        print_json(result)
        raise SystemExit(1)

    result["ok"] = True
    if args.output_json:
        ensure_parent_dir(args.output_json)
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
    print_json(result)


if __name__ == "__main__":
    main()

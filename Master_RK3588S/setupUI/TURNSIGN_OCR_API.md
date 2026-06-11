# TurnSign OCR/API Module

`turnsign_ocr_api.py` is a standalone module. It is not wired into `ar_receiver.py` yet.

## What It Does

1. Selects the best `TurnSign` detection from `perception["detections"]`.
2. Crops the sign region from the current frame.
3. Runs PaddleOCR on the cropped image.
4. Waits until OCR text is stable for consecutive frames.
5. Calls ERNIE SDK with `api_type="aistudio"` and an AI Studio access token.
6. Normalizes the model response into a fixed JSON instruction.

## Install

```bash
pip install -r requirements_turnsign.txt
```

If PaddleOCR fails to install on RK3588S, install a board-compatible PaddlePaddle wheel first.

For PC-side Windows testing, install PaddlePaddle explicitly first if PaddleOCR reports missing `paddle`:

```powershell
pip install paddlepaddle
pip install -r .\requirements_turnsign.txt
```

## Token

Recommended:

```bash
export AISTUDIO_ACCESS_TOKEN="your-token"
```

Compatible environment variables:

```bash
export AR_TURNSIGN_ACCESS_TOKEN="your-token"
export EB_ACCESS_TOKEN="your-token"
```

## Direct Text Test

```bash
python3 turnsign_ocr_api.py --text "左方有障碍，右方可通行"
```

Expected output is a JSON object with fields such as:

```json
{
  "valid": true,
  "instruction_type": "branch_choice",
  "action": "choose_branch",
  "preferred_branch": "right",
  "avoid_branches": ["left"]
}
```

## Image OCR Test

```bash
python3 turnsign_ocr_api.py --image sign.jpg
```

Crop a detected box manually:

```bash
python3 turnsign_ocr_api.py --image frame.jpg --bbox 120,80,280,180 --call-api
```

## Local Smoke Test Script

`test_turnsign_ocr_api.py` is intended for PC-side testing before wiring this into `ar_receiver.py`.

Windows PowerShell token setup:

```powershell
$env:AISTUDIO_ACCESS_TOKEN="your-token"
```

Test API only, without OCR:

```powershell
python .\test_turnsign_ocr_api.py --text "左方有障碍，右方可通行"
```

Test OCR + API on an already cropped sign image:

```powershell
python .\test_turnsign_ocr_api.py --image .\sign.jpg
```

If PaddleOCR reports that it cannot open model files under a Chinese user path, put the OCR models in an ASCII-only directory and pass it explicitly:

```powershell
python .\test_turnsign_ocr_api.py --image .\sign.jpg --ocr-model-root C:\paddleocr_models
```

Test OCR + API on a full frame and manually crop the TurnSign box:

```powershell
python .\test_turnsign_ocr_api.py --image .\frame.jpg --bbox 120,80,280,180 --crop-out .\debug_crop.jpg
```

Only test OCR and do not call AI Studio:

```powershell
python .\test_turnsign_ocr_api.py --image .\sign.jpg --no-api
```

Save full test output:

```powershell
python .\test_turnsign_ocr_api.py --image .\sign.jpg --output-json .\ocr_api_result.json
```

## Prompt Override

The built-in prompt is suitable for branch signs. To override it:

```bash
python3 turnsign_ocr_api.py --text "左方可通行" --prompt-file prompt.txt
```

## Runtime Protection

The processor includes:

- minimum detection score
- minimum sign area
- minimum OCR confidence
- OCR interval throttling
- stable text requirement
- API cooldown
- repeated result cache
- optional asynchronous API call

These guards are required because OCR and network inference should not block the video loop.

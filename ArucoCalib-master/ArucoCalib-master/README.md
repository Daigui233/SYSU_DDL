# ArucoCalib Locator

Windows overhead-camera localization for the RK3588S smart-car system.

## Runtime Flow

```text
USB global-shutter camera
  -> Windows ArucoCalibLocator.exe
  -> AprilTag 36h11 detection and homography localization
  -> meters + yaw
  -> UDP robot_position
  -> RK3588S:9005
```

Official UDP payload:

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

## Run

Use the packaged application:

```text
dist/ArucoCalibLocator.exe
```

The runtime UI only exposes settings needed during deployment:

- `Camera`: camera index. The laptop camera is usually `0`; the external camera is usually `1` or `2`.
- `Reconnect & Save`: reconnect the selected camera and remember the index.
- `Mirror`: horizontally correct a mirrored camera feed; changes apply immediately and are saved.
- `RK3588S IP`: RK3588S LAN address.
- `UDP Enable` and `Apply & Save`: enable output and remember the target.
- `Start Detection`, `Stop`, and `Reset Calibration`.

Fixed capture settings:

```text
DirectShow / 1920x1200 / YUY2 / 60 FPS
UDP port: 9005
```

The detector is fixed to `AprilTag 36h11`. The video overlay shows detected tags, vehicle direction, localization sequence, current send status, coordinates, yaw, and a trajectory inset.

## Field Configuration

Edit `dist/config.yaml` before formal deployment to define fixed-marker coordinates. Coordinates are stored in millimeters and converted to meters for UDP output.

Example for a 5 m x 4 m field:

```yaml
world_coordinates:
  1: [0.0, 0.0]
  2: [5000.0, 0.0]
  3: [5000.0, 4000.0]
  4: [0.0, 4000.0]

vehicle_id: 0
min_marker_count: 4
```

Keep the camera fixed after calibration. When enough fixed tags are visible, calibration is locked; temporary tag loss will not immediately discard it.

## Source Run

```powershell
pip install -r requirements.txt
python main.py
```

Core files:

- `aruco_app/ui_main.py`: runtime UI, localization overlay, trajectory display.
- `aruco_core/aruco_detector.py`: ArUco detection.
- `aruco_core/coordinate_transformer.py`: pixel-to-world homography.
- `aruco_core/udp_sender.py`: official UDP packet sender.
- `config.yaml`: source-run configuration.

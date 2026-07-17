"""
Configuration loader module for ArUco coordinate system.
Loads YAML config with fallback search paths.
"""
from pathlib import Path
import sys
import yaml
import cv2
import numpy as np


class Config:
    """Configuration class that loads settings from YAML file"""
    
    def __init__(self, config_file='config.yaml'):
        """
        Initialize configuration from YAML file
        
        Args:
            config_file: Path to YAML configuration file (default: 'config.yaml')
        """
        config_path = self._resolve_config_path(config_file)
        self.CONFIG_PATH = config_path

        # Load YAML configuration
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
        
        # Parse ArUco settings
        aruco_config = config_data.get('aruco', {})
        dict_type_name = aruco_config.get('dict_type', 'DICT_APRILTAG_36h11')
        
        # Convert string to cv2 constant
        self.ARUCO_DICT = getattr(cv2.aruco, dict_type_name)
        self.MARKER_SIZE = aruco_config.get('marker_size', 0.20)
        max_correction_bits = aruco_config.get('max_correction_bits', None)
        self.ARUCO_MAX_CORRECTION_BITS = (
            None if max_correction_bits is None else max(0, int(max_correction_bits))
        )

        # Optional: ArUco detector params overrides
        self.ARUCO_PARAMS = config_data.get("aruco_params", {}) or {}

        # Parse world coordinates
        world_coords = config_data.get('world_coordinates', {})
        self.WORLD_COORDINATES = {}
        for marker_id, coords in world_coords.items():
            # Convert to int key and numpy array
            self.WORLD_COORDINATES[int(marker_id)] = np.array(coords, dtype=np.float32)
        
        # Parse minimum marker count
        self.MIN_MARKER_COUNT = config_data.get('min_marker_count', 4)

        # Optional UI settings
        ui_config = config_data.get("ui", {}) or {}
        self.TRACE_WINDOW_MS = int(ui_config.get("trace_window_ms", 500))

        # Vehicle marker ID (ArUco marker_id on the car)
        # Prefer `vehicle_id`; keep `car_id` for backward compatibility.
        self.VEHICLE_ID = int(config_data.get("vehicle_id", config_data.get("car_id", 0)))
        camera_config = config_data.get("camera", {}) or {}
        self.CAMERA_INDEX = int(camera_config.get("index", 0))
        self.CAMERA_BACKEND = str(camera_config.get("backend", "DSHOW")).upper()
        self.CAMERA_WIDTH = int(camera_config.get("width", 1920))
        self.CAMERA_HEIGHT = int(camera_config.get("height", 1200))
        self.CAMERA_FPS = float(camera_config.get("fps", 60.0))
        self.CAMERA_FOURCC = str(camera_config.get("fourcc", "YUY2")).upper()
        self.CAMERA_MIRROR = bool(camera_config.get("mirror", True))
        self.CAMERA_AUTO_EXPOSURE = bool(camera_config.get("auto_exposure", True))
        self.CAMERA_EXPOSURE = float(camera_config.get("exposure", -6.0))
        self.CAMERA_GAIN = float(camera_config.get("gain", 0.0))
        self.CAMERA_BRIGHTNESS = float(camera_config.get("brightness", 0.0))
        self.CAMERA_CONTRAST = float(camera_config.get("contrast", 0.0))
        self.CAMERA_SATURATION = float(camera_config.get("saturation", 0.0))
        self.CAMERA_HUE = float(camera_config.get("hue", 0.0))
        self.CAMERA_SHARPNESS = float(camera_config.get("sharpness", 0.0))

        udp_config = config_data.get("udp", {}) or {}
        self.UDP_ENABLED = bool(udp_config.get("enabled", False))
        self.UDP_TARGET_IP = str(udp_config.get("target_ip", "127.0.0.1"))
        self.UDP_TARGET_PORT = int(udp_config.get("target_port", 9005))

        gamepad_config = config_data.get("gamepad_control", {}) or {}
        self.GAMEPAD_TARGET_PORT = int(gamepad_config.get("target_port", 9010))
        self.GAMEPAD_MAX_SPEED_MPS = float(gamepad_config.get("max_speed_mps", 1.0))
        self.GAMEPAD_STEER_ERROR_SCALE = float(gamepad_config.get("steer_error_scale", 210.0))

        output_config = config_data.get("output", {}) or {}
        self.OUTPUT_COORD_SCALE = float(output_config.get("coord_scale", 0.001))
        self.OUTPUT_HEIGHT_M = float(output_config.get("height_m", 0.0))
        self.OUTPUT_X_SIGN = float(output_config.get("x_sign", 1.0))
        self.OUTPUT_Z_SIGN = float(output_config.get("z_sign", 1.0))
        self.OUTPUT_YAW_SIGN = float(output_config.get("yaw_sign", 1.0))
        self.OUTPUT_YAW_OFFSET_DEG = float(output_config.get("yaw_offset_deg", 0.0))

        pose_filter_config = config_data.get("pose_filter", {}) or {}
        self.POSE_FILTER_ENABLED = bool(pose_filter_config.get("enabled", True))
        self.POSE_FILTER_POSITION_ALPHA = float(pose_filter_config.get("position_alpha", 0.35))
        self.POSE_FILTER_YAW_ALPHA = float(pose_filter_config.get("yaw_alpha", 0.35))
        self.POSE_FILTER_RESET_GAP_S = float(pose_filter_config.get("reset_gap_s", 0.35))

        rectification = config_data.get("vehicle_detection_rectification", {}) or {}
        self.VEHICLE_RECTIFICATION_ENABLED = bool(rectification.get("enabled", True))
        self.VEHICLE_RECTIFICATION_SCALE_PX_PER_MM = max(
            0.05,
            float(rectification.get("scale_px_per_mm", 0.30)),
        )
        self.VEHICLE_RECTIFICATION_MARGIN_MM = max(
            0.0,
            float(rectification.get("margin_mm", 150.0)),
        )
        self.VEHICLE_RECTIFICATION_MAX_DIMENSION = max(
            320,
            int(rectification.get("max_dimension", 1600)),
        )

    @staticmethod
    def _resolve_config_path(config_file: str) -> Path:
        """
        Resolve configuration file with compatibility:
        1) project root override: <cwd>/config.yaml
        2) package default: aruco_core/config.yaml
        """
        cwd_candidate = Path.cwd() / config_file
        executable_candidate = Path(sys.executable).resolve().parent / config_file
        project_candidate = Path(__file__).resolve().parents[1] / config_file
        package_candidate = Path(__file__).resolve().parent / config_file

        if getattr(sys, "frozen", False):
            candidates = (executable_candidate, cwd_candidate, project_candidate, package_candidate)
        else:
            candidates = (cwd_candidate, project_candidate, package_candidate)

        for candidate in candidates:
            if candidate.exists():
                return candidate

        tried = [str(candidate) for candidate in candidates]
        raise FileNotFoundError(
            "Config file not found. Tried paths: " + ", ".join(tried)
        )


# Global configuration instance
_config_instance = None


def get_config():
    """
    Get the global configuration instance
    
    Returns:
        Config: Global configuration object
    """
    global _config_instance
    if _config_instance is None:
        _config_instance = Config()
    return _config_instance


# For backward compatibility with old import style
def load_config():
    """Load and return configuration (creates singleton if needed)"""
    return get_config()


# Module-level attributes for direct access (backward compatibility)
def __getattr__(name):
    """
    Allow module-level attribute access like 'config.ARUCO_DICT'
    This provides backward compatibility with the old config.py
    """
    cfg = get_config()
    if hasattr(cfg, name):
        return getattr(cfg, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


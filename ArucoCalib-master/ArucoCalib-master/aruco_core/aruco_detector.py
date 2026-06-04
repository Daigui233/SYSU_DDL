"""
ArUco marker detection module
"""
import cv2
import numpy as np
from . import config_loader as config


class ArUcoDetector:
    """ArUco marker detector"""
    
    def __init__(self):
        """Initialize ArUco detector"""
        cfg = config.get_config()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cfg.ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self._apply_detector_params_overrides(getattr(cfg, "ARUCO_PARAMS", {}) or {})
        # Try to use ArucoDetector (OpenCV 4.7+), fallback to old API
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            self.use_new_api = False

    @staticmethod
    def _corner_refinement_method_from_cfg(v):
        """
        Map config value to OpenCV cornerRefinementMethod integer.

        Supported strings: NONE / SUBPIX / CONTOUR / APRILTAG
        """
        if v is None:
            return None
        if isinstance(v, (int, float)):
            return int(v)
        s = str(v).strip().upper()
        mapping = {
            "NONE": 0,
            "SUBPIX": 1,
            "CONTOUR": 2,
            "APRILTAG": 3,
        }
        return mapping.get(s, None)

    def _apply_detector_params_overrides(self, overrides: dict) -> None:
        """
        Apply selected DetectorParameters overrides from config.
        Only sets attributes that exist on the current OpenCV build.
        """
        if not overrides:
            return

        # Corner refinement needs string->int mapping if provided as string.
        if "cornerRefinementMethod" in overrides:
            method = self._corner_refinement_method_from_cfg(overrides.get("cornerRefinementMethod"))
            if method is not None and hasattr(self.aruco_params, "cornerRefinementMethod"):
                self.aruco_params.cornerRefinementMethod = int(method)

        # Generic attribute pass-through (numbers/bools).
        passthrough_keys = [
            "minMarkerPerimeterRate",
            "maxMarkerPerimeterRate",
            "adaptiveThreshWinSizeMin",
            "adaptiveThreshWinSizeMax",
            "adaptiveThreshWinSizeStep",
            "adaptiveThreshConstant",
            "polygonalApproxAccuracyRate",
            "minCornerDistanceRate",
            "minDistanceToBorder",
            "minMarkerDistanceRate",
            "cornerRefinementWinSize",
            "cornerRefinementMaxIterations",
            "cornerRefinementMinAccuracy",
        ]
        for k in passthrough_keys:
            if k not in overrides:
                continue
            if hasattr(self.aruco_params, k):
                try:
                    setattr(self.aruco_params, k, overrides[k])
                except Exception:
                    # Ignore invalid values to keep detector functional.
                    pass

    def detect_markers(self, image):
        """
        Detect ArUco markers in the image
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            tuple: (corners, ids, image_with_markers)
                - corners: List of marker corners
                - ids: Array of marker IDs
                - image_with_markers: Image with detected markers drawn
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect markers
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        
        # Draw detected markers
        image_with_markers = image.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(image_with_markers, corners, ids)
        
        return corners, ids, image_with_markers
    
    def get_marker_centers(self, corners, ids):
        """
        Get center coordinates of detected markers
        
        Args:
            corners: List of marker corners
            ids: Array of marker IDs
            
        Returns:
            dict: Dictionary mapping marker ID to center pixel coordinates (x, y)
        """
        marker_centers = {}
        
        if ids is not None and corners is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # Get the four corners of the marker
                marker_corners = corners[i][0]
                # Calculate center as average of four corners
                center = np.mean(marker_corners, axis=0)
                marker_centers[int(marker_id)] = center.astype(np.float32)
        
        return marker_centers
    
    def get_required_markers(self, corners, ids):
        """
        Get all detected markers that have known world coordinates
        
        Args:
            corners: List of marker corners
            ids: Array of marker IDs
            
        Returns:
            dict: Dictionary mapping marker ID to center pixel coordinates
                  Returns None if fewer than MIN_MARKER_COUNT markers are detected
        """
        cfg = config.get_config()
        marker_centers = self.get_marker_centers(corners, ids)
        
        # Find markers that have known world coordinates
        known_marker_ids = set(cfg.WORLD_COORDINATES.keys())
        detected_ids = set(marker_centers.keys())
        valid_marker_ids = known_marker_ids.intersection(detected_ids)
        
        # Check if we have enough markers for calibration
        if len(valid_marker_ids) >= cfg.MIN_MARKER_COUNT:
            # Return all valid markers
            return {marker_id: marker_centers[marker_id] for marker_id in valid_marker_ids}
        else:
            return None

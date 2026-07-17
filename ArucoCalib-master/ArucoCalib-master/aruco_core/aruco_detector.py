"""
ArUco marker detection module
"""
import cv2
import itertools
import numpy as np
from . import config_loader as config


class ArUcoDetector:
    """ArUco marker detector"""
    
    def __init__(self):
        """Initialize ArUco detector"""
        cfg = config.get_config()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cfg.ARUCO_DICT)
        max_correction_bits = getattr(cfg, "ARUCO_MAX_CORRECTION_BITS", None)
        if max_correction_bits is not None:
            # OpenCV ships DICT_APRILTAG_36h11 with zero correction bits in
            # this runtime. Allow a bounded amount of decode noise so the
            # moving vehicle tag remains detectable when small or blurred.
            self.aruco_dict.maxCorrectionBits = int(max_correction_bits)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self._apply_detector_params_overrides(getattr(cfg, "ARUCO_PARAMS", {}) or {})
        self.exposure_compensation_enabled = bool(getattr(
            cfg, "VEHICLE_EXPOSURE_COMPENSATION_ENABLED", True))
        clahe_grid = int(getattr(
            cfg, "VEHICLE_EXPOSURE_CLAHE_TILE_GRID_SIZE", 8))
        self.exposure_clahe = cv2.createCLAHE(
            clipLimit=float(getattr(
                cfg, "VEHICLE_EXPOSURE_CLAHE_CLIP_LIMIT", 3.0)),
            tileGridSize=(clahe_grid, clahe_grid),
        )
        self.exposure_bright_mean_threshold = float(getattr(
            cfg, "VEHICLE_EXPOSURE_BRIGHT_MEAN_THRESHOLD", 180.0))
        self.exposure_dark_mean_threshold = float(getattr(
            cfg, "VEHICLE_EXPOSURE_DARK_MEAN_THRESHOLD", 75.0))
        self.exposure_bright_lut = self._gamma_lut(float(getattr(
            cfg, "VEHICLE_EXPOSURE_BRIGHT_GAMMA", 2.2)))
        self.exposure_dark_lut = self._gamma_lut(float(getattr(
            cfg, "VEHICLE_EXPOSURE_DARK_GAMMA", 0.65)))
        # Try to use ArucoDetector (OpenCV 4.7+), fallback to old API
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            self.use_new_api = False

    @staticmethod
    def _gamma_lut(gamma):
        gamma = max(0.1, float(gamma))
        values = np.arange(256, dtype=np.float32) / 255.0
        return np.clip(
            np.power(values, gamma) * 255.0 + 0.5,
            0.0,
            255.0,
        ).astype(np.uint8)

    @staticmethod
    def _as_gray(image):
        if len(image.shape) == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return np.asarray(image, dtype=np.uint8)

    def _detect_gray(self, gray):
        if self.use_new_api:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

    def exposure_compensated_gray(self, image):
        """Normalize a missed vehicle ROI without altering the display frame."""
        gray = self._as_gray(image)
        mean_value = float(np.mean(gray)) if gray.size else 0.0
        if mean_value >= self.exposure_bright_mean_threshold:
            # Gamma > 1 restores separation between washed-out black cells and
            # the white paper.  Saturated white stays white.
            gray = cv2.LUT(gray, self.exposure_bright_lut)
        elif mean_value <= self.exposure_dark_mean_threshold:
            gray = cv2.LUT(gray, self.exposure_dark_lut)
        return self.exposure_clahe.apply(gray)

    def detect_markers_exposure_compensated(self, image):
        """Fallback marker pass for locally over/under-exposed vehicle tags."""
        if self.exposure_compensation_enabled:
            gray = self.exposure_compensated_gray(image)
        else:
            gray = self._as_gray(image)
        corners, ids, _ = self._detect_gray(gray)
        return corners, ids, image.copy()

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
            "aprilTagQuadDecimate",
            "aprilTagQuadSigma",
            "aprilTagMinClusterPixels",
            "aprilTagMaxNmaxima",
            "aprilTagCriticalRad",
            "aprilTagMaxLineFitMse",
            "aprilTagMinWhiteBlackDiff",
            "aprilTagDeglitch",
            "errorCorrectionRate",
            "maxErroneousBitsInBorderRate",
            "minOtsuStdDev",
            "perspectiveRemovePixelPerCell",
            "perspectiveRemoveIgnoredMarginPerCell",
            "minSideLengthCanonicalImg",
            "detectInvertedMarker",
            "useAruco3Detection",
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
        gray = self._as_gray(image)
        corners, ids, _ = self._detect_gray(gray)
        
        # Drawing is handled by the UI so locked fixed tags can stay stable
        # while the vehicle tag remains live.
        image_with_markers = image.copy()
        
        return corners, ids, image_with_markers
    
    def get_marker_centers(self, corners, ids):
        """
        Get center coordinates keyed by marker ID (legacy helper).

        Duplicate IDs overwrite each other; new calibration/vehicle code must
        use get_marker_instances() instead.
        
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

    def get_marker_instances(self, corners, ids):
        """Return every detection separately, including duplicate marker IDs.

        A dictionary keyed by marker ID cannot represent two physical tags that
        use the same printed ID.  Calibration and vehicle selection therefore
        use this instance list as their source of truth.
        """
        marker_instances = []
        if ids is None or corners is None:
            return marker_instances

        for detection_index, marker_id in enumerate(ids.flatten()):
            marker_corners = np.asarray(corners[detection_index][0], dtype=np.float32)
            if marker_corners.shape != (4, 2):
                continue
            marker_instances.append(
                {
                    "id": int(marker_id),
                    "index": int(detection_index),
                    "corners": marker_corners,
                    "center": np.mean(marker_corners, axis=0).astype(np.float32),
                }
            )
        return marker_instances

    @staticmethod
    def _calibration_quad_area(marker_instances):
        """Return the convex quadrilateral area, or -1 for an invalid layout."""
        if len(marker_instances) != 4:
            return -1.0
        points = np.asarray(
            [marker["center"] for marker in marker_instances],
            dtype=np.float32,
        )
        hull = cv2.convexHull(points)
        if hull is None or len(hull) != 4:
            return -1.0
        return float(abs(cv2.contourArea(hull)))

    def select_calibration_markers(self, marker_instances, required_count=4):
        """Select four spatially separated fixed-tag detections for calibration.

        Marker IDs are intentionally ignored.  When more than four detections
        are present, the widest convex quadrilateral is used; this lets the four
        field-corner tags win over an extra tag inside the field.
        """
        required_count = int(required_count)
        if required_count != 4 or len(marker_instances) < required_count:
            return None

        if len(marker_instances) == required_count:
            selected = list(marker_instances)
            return selected if self._calibration_quad_area(selected) > 1.0 else None

        # A normal frame contains only four or five tags.  Bound pathological
        # false-positive frames before evaluating combinations.
        candidates = list(marker_instances)
        if len(candidates) > 12:
            centers = np.asarray([m["center"] for m in candidates], dtype=np.float32)
            centroid = np.mean(centers, axis=0)
            distances = np.linalg.norm(centers - centroid, axis=1)
            keep = np.argsort(distances)[-12:]
            candidates = [candidates[int(i)] for i in keep]

        best_markers = None
        best_area = -1.0
        for combination in itertools.combinations(candidates, required_count):
            area = self._calibration_quad_area(combination)
            if area > best_area:
                best_area = area
                best_markers = list(combination)
        return best_markers if best_area > 1.0 else None
    
    def get_required_markers(self, corners, ids):
        """
        Get four fixed-tag detection instances for spatial calibration.
        
        Args:
            corners: List of marker corners
            ids: Array of marker IDs
            
        Returns:
            list[dict]: Four detection instances. Marker IDs may repeat.
                        Returns None if a valid four-corner layout is unavailable.
        """
        cfg = config.get_config()
        marker_instances = self.get_marker_instances(corners, ids)
        return self.select_calibration_markers(
            marker_instances,
            required_count=getattr(cfg, "MIN_MARKER_COUNT", 4),
        )

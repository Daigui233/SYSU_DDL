"""
Coordinate transformation module using homography
"""
import cv2
import numpy as np
from . import config_loader as config


class CoordinateTransformer:
    """Transform pixel coordinates to world coordinates using homography"""
    
    def __init__(self):
        """Initialize coordinate transformer"""
        self.homography_matrix = None
        self.homography_matrix_inv = None
        self.is_calibrated = False
    
    def calibrate(self, pixel_coords_dict):
        """
        Calibrate the coordinate system using all detected ArUco markers
        
        Args:
            pixel_coords_dict: Dictionary mapping marker ID to pixel coordinates (x, y)
                              Must contain at least MIN_MARKER_COUNT markers that are
                              defined in WORLD_COORDINATES
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        # Get configuration
        cfg = config.get_config()
        
        # Find markers that are both detected and have known world coordinates
        known_marker_ids = set(cfg.WORLD_COORDINATES.keys())
        detected_marker_ids = set(pixel_coords_dict.keys())
        valid_marker_ids = known_marker_ids.intersection(detected_marker_ids)
        
        # Check if we have enough markers for calibration
        if len(valid_marker_ids) < cfg.MIN_MARKER_COUNT:
            self.is_calibrated = False
            return False
        
        # Prepare source points (pixel coordinates)
        src_points = []
        # Prepare destination points (world coordinates)
        dst_points = []
        
        # Use all valid markers in sorted order for consistency
        for marker_id in sorted(valid_marker_ids):
            pixel_coord = pixel_coords_dict[marker_id]
            world_coord = cfg.WORLD_COORDINATES[marker_id]
            
            src_points.append([pixel_coord[0], pixel_coord[1]])
            dst_points.append([world_coord[0], world_coord[1]])
        
        src_points = np.array(src_points, dtype=np.float32)
        dst_points = np.array(dst_points, dtype=np.float32)
        
        # Calculate homography matrix
        self.homography_matrix, mask = cv2.findHomography(
            src_points, 
            dst_points, 
            method=cv2.RANSAC
        )
        
        if self.homography_matrix is not None:
            # Inverse mapping: world -> pixel (needed for drawing overlays).
            try:
                self.homography_matrix_inv = np.linalg.inv(self.homography_matrix)
            except np.linalg.LinAlgError:
                self.homography_matrix_inv = None
                self.is_calibrated = False
                return False
            self.is_calibrated = True
            return True
        else:
            self.is_calibrated = False
            self.homography_matrix_inv = None
            return False
    
    def pixel_to_world(self, pixel_x, pixel_y, z=0.0):
        """
        Transform pixel coordinates to world coordinates
        
        Args:
            pixel_x: Pixel x coordinate
            pixel_y: Pixel y coordinate
            z: Z coordinate in world space (default 0.0)
        
        Returns:
            tuple: (world_x, world_y) or None if not calibrated
        """
        if not self.is_calibrated or self.homography_matrix is None:
            return None
        
        # Prepare point in homogeneous coordinates
        pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32)
        
        # Transform using homography
        world_point = cv2.perspectiveTransform(
            pixel_point.reshape(-1, 1, 2), 
            self.homography_matrix
        )
        
        world_x = float(world_point[0, 0, 0])
        world_y = float(world_point[0, 0, 1])
        
        return (world_x, world_y)

    def world_to_pixel(self, world_x, world_y):
        """
        Transform world coordinates (x, y) to pixel coordinates
        
        Args:
            world_x: World x coordinate (unit: mm in this project)
            world_y: World y coordinate (unit: mm in this project)
        
        Returns:
            tuple: (pixel_x, pixel_y) or None if not calibrated
        """
        if not self.is_calibrated or self.homography_matrix_inv is None:
            return None

        world_point = np.array([[world_x, world_y]], dtype=np.float32)
        pixel_point = cv2.perspectiveTransform(
            world_point.reshape(-1, 1, 2),
            self.homography_matrix_inv,
        )

        pixel_x = float(pixel_point[0, 0, 0])
        pixel_y = float(pixel_point[0, 0, 1])
        return (pixel_x, pixel_y)
    
    def get_calibration_status(self):
        """
        Get calibration status
        
        Returns:
            bool: True if calibrated, False otherwise
        """
        return self.is_calibrated

    def reset_calibration(self):
        """Reset homography and calibration status (used by UI when markers are missing)."""
        self.homography_matrix = None
        self.homography_matrix_inv = None
        self.is_calibrated = False

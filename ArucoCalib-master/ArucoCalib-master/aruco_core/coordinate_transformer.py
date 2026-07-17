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

    @staticmethod
    def _point_array(pixel_coords):
        """Normalize legacy dict input or instance/point lists to four points."""
        if isinstance(pixel_coords, dict):
            values = list(pixel_coords.values())
        else:
            values = list(pixel_coords)

        points = []
        for value in values:
            if isinstance(value, dict):
                value = value.get("center")
            point = np.asarray(value, dtype=np.float32).reshape(-1)
            if point.size < 2:
                return None
            points.append([float(point[0]), float(point[1])])
        if len(points) != 4:
            return None
        return np.asarray(points, dtype=np.float32)

    @staticmethod
    def _order_image_corners(points):
        """Order image points as top-left, top-right, bottom-right, bottom-left."""
        points = np.asarray(points, dtype=np.float32)
        by_y = points[np.argsort(points[:, 1])]
        top = by_y[:2][np.argsort(by_y[:2, 0])]
        bottom = by_y[2:][np.argsort(by_y[2:, 0])]
        ordered = np.asarray([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)
        if len(cv2.convexHull(ordered)) != 4 or abs(cv2.contourArea(ordered)) <= 1.0:
            return None
        return ordered

    @staticmethod
    def _order_world_corners(world_coordinates):
        """Match current field axes to the four visible image-corner roles.

        In the existing convention +X points left and +Z points up, so image
        left uses the larger world X and image top uses the larger world Z.
        """
        points = np.asarray(list(world_coordinates.values()), dtype=np.float32)
        if points.shape != (4, 2):
            return None
        by_z = points[np.argsort(points[:, 1])]
        bottom = by_z[:2][np.argsort(by_z[:2, 0])[::-1]]
        top = by_z[2:][np.argsort(by_z[2:, 0])[::-1]]
        return np.asarray([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)
    
    def calibrate(self, pixel_coords_dict):
        """
        Calibrate the coordinate system from four visible fixed-tag instances.

        Tag IDs are not used. The four pixel centers are assigned to the current
        world corners by their on-screen top/left/right/bottom positions.
        
        Args:
            pixel_coords_dict: Legacy ID->center dict, a list of centers, or a
                               list of detection-instance dictionaries.
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        # Get configuration
        cfg = config.get_config()
        
        src_raw = self._point_array(pixel_coords_dict)
        src_points = self._order_image_corners(src_raw) if src_raw is not None else None
        dst_points = self._order_world_corners(cfg.WORLD_COORDINATES)
        if src_points is None or dst_points is None:
            self.is_calibrated = False
            return False
        
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

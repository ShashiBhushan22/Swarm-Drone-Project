#!/usr/bin/env python3
"""
Vision Processing Module

OpenCV-based computer vision processing for drone camera feeds.
Provides object detection, feature extraction, and image analysis capabilities.
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum


class DetectionType(Enum):
    """Types of visual detections"""
    OBJECT = "object"
    MARKER = "marker"
    LINE = "line"
    PERSON = "person"
    VEHICLE = "vehicle"
    LANDING_PAD = "landing_pad"


@dataclass
class Detection:
    """Visual detection result"""
    detection_type: DetectionType
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x, y, width, height)
    center: Tuple[int, int]  # (x, y)
    label: str
    metadata: Dict = None


class VisionProcessor:
    """
    Computer vision processing for drone camera feeds.
    Provides various detection and analysis algorithms.
    """
    
    def __init__(self):
        self.color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'yellow': [(20, 100, 100), (30, 255, 255)],
        }
    
    def detect_colors(self, image: np.ndarray, colors: List[str] = None) -> Dict[str, List[Detection]]:
        """
        Detect colored objects in image.
        
        Args:
            image: Input image (BGR format)
            colors: List of colors to detect (default: all)
        
        Returns:
            Dictionary mapping color name to list of detections
        """
        if colors is None:
            colors = list(self.color_ranges.keys())
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = {}
        
        for color in colors:
            if color not in self.color_ranges:
                continue
            
            lower, upper = self.color_ranges[color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            color_detections = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    detection = Detection(
                        detection_type=DetectionType.OBJECT,
                        confidence=min(area / 10000.0, 1.0),
                        bbox=(x, y, w, h),
                        center=(center_x, center_y),
                        label=f"{color}_object",
                        metadata={'area': area, 'color': color}
                    )
                    color_detections.append(detection)
            
            detections[color] = color_detections
        
        return detections
    
    def detect_aruco_markers(self, image: np.ndarray, dictionary_type=cv2.aruco.DICT_4X4_50) -> List[Detection]:
        """
        Detect ArUco markers in image.
        
        Args:
            image: Input image
            dictionary_type: ArUco dictionary to use
        
        Returns:
            List of marker detections
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        corners, ids, rejected = detector.detectMarkers(gray)
        
        detections = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                
                # Calculate bounding box
                x_coords = corner[:, 0]
                y_coords = corner[:, 1]
                x, y = int(x_coords.min()), int(y_coords.min())
                w = int(x_coords.max() - x_coords.min())
                h = int(y_coords.max() - y_coords.min())
                
                # Calculate center
                center_x = int(x_coords.mean())
                center_y = int(y_coords.mean())
                
                detection = Detection(
                    detection_type=DetectionType.MARKER,
                    confidence=1.0,
                    bbox=(x, y, w, h),
                    center=(center_x, center_y),
                    label=f"ArUco_{marker_id}",
                    metadata={'marker_id': int(marker_id), 'corners': corner.tolist()}
                )
                detections.append(detection)
        
        return detections
    
    def detect_edges(self, image: np.ndarray, low_threshold: int = 50, high_threshold: int = 150) -> np.ndarray:
        """
        Detect edges in image using Canny edge detection.
        
        Args:
            image: Input image
            low_threshold: Lower threshold for edge detection
            high_threshold: Upper threshold for edge detection
        
        Returns:
            Edge map (binary image)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, low_threshold, high_threshold)
        return edges
    
    def detect_lines(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect straight lines in image using Hough transform.
        
        Args:
            image: Input image
        
        Returns:
            List of lines as (x1, y1, x2, y2) tuples
        """
        edges = self.detect_edges(image)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100,
                                minLineLength=50, maxLineGap=10)
        
        if lines is None:
            return []
        
        return [(int(x1), int(y1), int(x2), int(y2)) for x1, y1, x2, y2 in lines[:, 0]]
    
    def detect_circles(self, image: np.ndarray, min_radius: int = 20, max_radius: int = 100) -> List[Tuple[int, int, int]]:
        """
        Detect circles in image using Hough circle transform.
        
        Args:
            image: Input image
            min_radius: Minimum circle radius
            max_radius: Maximum circle radius
        
        Returns:
            List of circles as (center_x, center_y, radius) tuples
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                   param1=100, param2=30, minRadius=min_radius, maxRadius=max_radius)
        
        if circles is None:
            return []
        
        circles = np.uint16(np.around(circles))
        return [(int(x), int(y), int(r)) for x, y, r in circles[0]]
    
    def extract_features(self, image: np.ndarray, method: str = 'orb') -> Tuple[List, np.ndarray]:
        """
        Extract keypoints and descriptors from image.
        
        Args:
            image: Input image
            method: Feature detection method ('orb', 'sift', 'surf')
        
        Returns:
            Tuple of (keypoints, descriptors)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if method.lower() == 'orb':
            detector = cv2.ORB_create()
        elif method.lower() == 'sift':
            detector = cv2.SIFT_create()
        else:
            detector = cv2.ORB_create()
        
        keypoints, descriptors = detector.detectAndCompute(gray, None)
        return keypoints, descriptors
    
    def match_features(self, desc1: np.ndarray, desc2: np.ndarray, threshold: float = 0.7) -> List:
        """
        Match features between two images.
        
        Args:
            desc1: Descriptors from first image
            desc2: Descriptors from second image
            threshold: Distance ratio threshold for matching
        
        Returns:
            List of good matches
        """
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        matches = bf.knnMatch(desc1, desc2, k=2)
        
        # Apply ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < threshold * n.distance:
                    good_matches.append(m)
        
        return good_matches
    
    def calculate_optical_flow(self, prev_gray: np.ndarray, curr_gray: np.ndarray) -> np.ndarray:
        """
        Calculate optical flow between two consecutive frames.
        
        Args:
            prev_gray: Previous grayscale frame
            curr_gray: Current grayscale frame
        
        Returns:
            Flow field (dx, dy for each pixel)
        """
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, curr_gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0
        )
        return flow


class VisualizationHelper:
    """Helper class for visualizing detection results"""
    
    @staticmethod
    def draw_detections(image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Draw detection bounding boxes and labels on image"""
        result = image.copy()
        
        for det in detections:
            x, y, w, h = det.bbox
            
            # Draw bounding box
            color = (0, 255, 0)  # Green
            cv2.rectangle(result, (x, y), (x+w, y+h), color, 2)
            
            # Draw center point
            cv2.circle(result, det.center, 5, (0, 0, 255), -1)
            
            # Draw label
            label = f"{det.label} ({det.confidence:.2f})"
            cv2.putText(result, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, color, 2)
        
        return result
    
    @staticmethod
    def draw_lines(image: np.ndarray, lines: List[Tuple[int, int, int, int]]) -> np.ndarray:
        """Draw detected lines on image"""
        result = image.copy()
        
        for x1, y1, x2, y2 in lines:
            cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        return result
    
    @staticmethod
    def draw_circles(image: np.ndarray, circles: List[Tuple[int, int, int]]) -> np.ndarray:
        """Draw detected circles on image"""
        result = image.copy()
        
        for x, y, r in circles:
            cv2.circle(result, (x, y), r, (0, 255, 0), 2)
            cv2.circle(result, (x, y), 2, (0, 0, 255), 3)
        
        return result
    
    @staticmethod
    def draw_keypoints(image: np.ndarray, keypoints: List) -> np.ndarray:
        """Draw keypoints on image"""
        return cv2.drawKeypoints(image, keypoints, None, 
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

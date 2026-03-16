#!/usr/bin/env python3
"""
YOLO26 Detector using NCNN backend
Optimized for Raspberry Pi 5
"""

import ncnn
import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from .coco_classes import COCO_CLASSES


@dataclass
class Detection:
    """Single detection result"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2 (normalized 0-1)
    bbox_pixels: Tuple[int, int, int, int]   # x1, y1, x2, y2 (pixels)


class Yolo26Detector:
    """YOLO26 NCNN Detector for ROS2"""
    
    def __init__(
        self,
        model_path: str,
        input_size: int = 416,
        confidence_threshold: float = 0.5,
        num_threads: int = 4,
        class_filter: Optional[List[int]] = None
    ):
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.class_filter = class_filter
        self.num_classes = 80  # COCO
        
        # Load NCNN model
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = num_threads
        self.net.opt.lightmode = True
        
        param_path = f"{model_path}/model.ncnn.param"
        bin_path = f"{model_path}/model.ncnn.bin"
        
        ret_param = self.net.load_param(param_path)
        ret_bin = self.net.load_model(bin_path)
        
        if ret_param != 0 or ret_bin != 0:
            raise RuntimeError(f"Failed to load model: param={ret_param}, bin={ret_bin}")
        
        self.input_name = "in0"
        self.output_name = "out0"
        
    def letterbox(
        self, 
        img: np.ndarray, 
        new_shape: Tuple[int, int],
        color: Tuple[int, int, int] = (114, 114, 114)
    ) -> Tuple[np.ndarray, float, Tuple[float, float]]:
        """Resize image with letterbox (maintain aspect ratio)"""
        shape = img.shape[:2]
        
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        
        dw = (new_shape[1] - new_unpad[0]) / 2
        dh = (new_shape[0] - new_unpad[1]) / 2
        
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                                  cv2.BORDER_CONSTANT, value=color)
        
        return img, r, (dw, dh)
    
    def preprocess(self, img: np.ndarray) -> Tuple[ncnn.Mat, float, Tuple[float, float]]:
        """Preprocess image for NCNN inference"""
        img_lb, ratio, pad = self.letterbox(img, (self.input_size, self.input_size))
        
        mat_in = ncnn.Mat.from_pixels(
            img_lb,
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            self.input_size,
            self.input_size
        )
        
        mat_in.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
        
        return mat_in, ratio, pad
    
    def postprocess(
        self, 
        output: np.ndarray, 
        img_shape: Tuple[int, int],
        ratio: float,
        pad: Tuple[float, float]
    ) -> List[Detection]:
        """
        Parse YOLO26 output tensor to detections
        
        YOLO26 NCNN output shape: (84, N) where:
        - 84 = 4 (cx, cy, w, h) + 80 (class scores)
        - N = number of anchors (e.g., 3549)
        
        Need to transpose to (N, 84) for processing
        """
        detections = []
        img_h, img_w = img_shape
        
        # Output shape is (84, N) - transpose to (N, 84)
        if output.shape[0] == 84:
            output = output.T  # Now (N, 84)
        
        # Each row: [cx, cy, w, h, class_scores...]
        for row in output:
            # Get class scores (indices 4-84)
            class_scores = row[4:]
            
            # Find best class
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            
            # Filter by confidence
            if confidence < self.confidence_threshold:
                continue
            
            # Filter by class if specified
            if self.class_filter and class_id not in self.class_filter:
                continue
            
            # Get bbox (cx, cy, w, h) in input_size coordinates
            cx, cy, w, h = row[:4]
            
            # Convert to x1, y1, x2, y2
            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2
            
            # Scale back to original image coordinates
            # Remove padding and scale
            x1 = (x1 - pad[0]) / ratio
            y1 = (y1 - pad[1]) / ratio
            x2 = (x2 - pad[0]) / ratio
            y2 = (y2 - pad[1]) / ratio
            
            # Clip to image bounds
            x1 = max(0, min(x1, img_w))
            y1 = max(0, min(y1, img_h))
            x2 = max(0, min(x2, img_w))
            y2 = max(0, min(y2, img_h))
            
            # Skip invalid boxes
            if x2 <= x1 or y2 <= y1:
                continue
            
            detections.append(Detection(
                class_id=int(class_id),
                class_name=COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}",
                confidence=float(confidence),
                bbox=(x1/img_w, y1/img_h, x2/img_w, y2/img_h),
                bbox_pixels=(int(x1), int(y1), int(x2), int(y2))
            ))
        
        # Sort by confidence (highest first)
        detections.sort(key=lambda x: x.confidence, reverse=True)
        
        return detections
    
    def detect(self, img: np.ndarray) -> Tuple[List[Detection], float]:
        """
        Run detection on image
        
        Args:
            img: BGR image (OpenCV format)
            
        Returns:
            detections: List of Detection objects
            inference_time: Time in seconds
        """
        import time
        
        img_shape = img.shape[:2]
        
        # Preprocess
        mat_in, ratio, pad = self.preprocess(img)
        
        # Inference
        t1 = time.perf_counter()
        ex = self.net.create_extractor()
        ex.input(self.input_name, mat_in)
        ret, mat_out = ex.extract(self.output_name)
        inference_time = time.perf_counter() - t1
        
        if ret != 0:
            return [], inference_time
        
        # Postprocess
        output = np.array(mat_out)
        detections = self.postprocess(output, img_shape, ratio, pad)
        
        return detections, inference_time
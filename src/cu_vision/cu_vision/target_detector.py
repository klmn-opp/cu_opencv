from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


@dataclass
class DetectionResult:
    center_x: float
    center_y: float
    area: float
    bbox_x: int
    bbox_y: int
    bbox_w: int
    bbox_h: int
    contour_points: int


def build_color_mask(hsv: np.ndarray, target_color: str) -> np.ndarray:
    if target_color == "red":
        lower1 = np.array([0, 90, 70], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([170, 90, 70], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    elif target_color == "blue":
        lower = np.array([95, 80, 60], dtype=np.uint8)
        upper = np.array([135, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
    else:
        raise ValueError(f"unsupported target_color: {target_color}")

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def pick_best_contour(contours: list[np.ndarray], min_area: float) -> Optional[np.ndarray]:
    best = None
    best_score = -1.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        x, y, w, h = cv2.boundingRect(contour)
        aspect = w / float(h) if h > 0 else 0.0
        aspect_score = 1.0 - min(abs(math.log(max(aspect, 1e-6))), 2.0) / 2.0
        score = area * (0.4 + 0.4 * circularity + 0.2 * aspect_score)
        if score > best_score:
            best_score = score
            best = contour
    return best


def contour_center(contour: np.ndarray) -> Optional[DetectionResult]:
    m = cv2.moments(contour)
    if abs(m["m00"]) < 1e-6:
        return None
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    x, y, w, h = cv2.boundingRect(contour)
    return DetectionResult(
        center_x=float(cx),
        center_y=float(cy),
        area=float(cv2.contourArea(contour)),
        bbox_x=int(x),
        bbox_y=int(y),
        bbox_w=int(w),
        bbox_h=int(h),
        contour_points=int(len(contour)),
    )


class TargetDetector(Node):
    def __init__(self) -> None:
        super().__init__("target_detector")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("min_area", 150.0)
        self.declare_parameter("display", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("preview_scale", 1.0)

        self.bridge = CvBridge()
        self.last_detection: Optional[DetectionResult] = None

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.target_color = self.get_parameter("target_color").get_parameter_value().string_value
        self.min_area = float(self.get_parameter("min_area").value)
        self.display = bool(self.get_parameter("display").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.preview_scale = float(self.get_parameter("preview_scale").value)

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(Float32MultiArray, "target_center_px", 10)
        self.mask_pub = self.create_publisher(Image, "target_mask", qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, "target_debug", qos_profile_sensor_data)

        self.get_logger().info(
            f"listening on {image_topic}, target_color={self.target_color}, min_area={self.min_area}"
        )

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = build_color_mask(hsv, self.target_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = pick_best_contour(contours, self.min_area)

        debug = frame.copy()
        out = Float32MultiArray()
        if contour is not None:
            result = contour_center(contour)
            if result is not None:
                self.last_detection = result
                out.data = [result.center_x, result.center_y, result.area]
                cv2.rectangle(
                    debug,
                    (result.bbox_x, result.bbox_y),
                    (result.bbox_x + result.bbox_w, result.bbox_y + result.bbox_h),
                    (0, 255, 0),
                    2,
                )
                cv2.circle(debug, (int(round(result.center_x)), int(round(result.center_y))), 5, (0, 255, 255), -1)
                cv2.putText(
                    debug,
                    f"({result.center_x:.1f}, {result.center_y:.1f}) area={result.area:.0f}",
                    (result.bbox_x, max(0, result.bbox_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                self.center_pub.publish(out)
                self.get_logger().debug(
                    f"detected center=({result.center_x:.1f}, {result.center_y:.1f}) area={result.area:.1f}"
                )
        else:
            out.data = []

        if self.publish_debug_image:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

        if self.display:
            if self.preview_scale > 0 and abs(self.preview_scale - 1.0) > 1e-6:
                debug_view = cv2.resize(debug, None, fx=self.preview_scale, fy=self.preview_scale, interpolation=cv2.INTER_AREA)
                mask_view = cv2.resize(mask, None, fx=self.preview_scale, fy=self.preview_scale, interpolation=cv2.INTER_NEAREST)
            else:
                debug_view = debug
                mask_view = mask
            cv2.imshow("target_debug", debug_view)
            cv2.imshow("target_mask", mask_view)
            cv2.waitKey(1)


def main() -> None:
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

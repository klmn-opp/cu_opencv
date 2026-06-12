from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

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
    vertex_count: int


def odd_kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return np.ones((size, size), np.uint8)


def build_color_mask(
    frame: np.ndarray,
    hsv: np.ndarray,
    target_color: str,
    hsv_s_min: int,
    hsv_v_min: int,
    channel_delta: int,
    close_kernel_size: int,
    close_iterations: int,
    open_kernel_size: int,
) -> np.ndarray:
    if target_color == "red":
        lower1 = np.array([0, hsv_s_min, hsv_v_min], dtype=np.uint8)
        upper1 = np.array([14, 255, 255], dtype=np.uint8)
        lower2 = np.array([166, hsv_s_min, hsv_v_min], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)
        hsv_mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        b, g, r = cv2.split(frame)
        channel_mask = (
            (r.astype(np.int16) > g.astype(np.int16) + channel_delta)
            & (r.astype(np.int16) > b.astype(np.int16) + channel_delta)
        )
        mask = hsv_mask | (channel_mask.astype(np.uint8) * 255)
    elif target_color == "blue":
        lower = np.array([92, hsv_s_min, hsv_v_min], dtype=np.uint8)
        upper = np.array([132, 255, 255], dtype=np.uint8)
        hsv_mask = cv2.inRange(hsv, lower, upper)

        b, g, r = cv2.split(frame)
        channel_mask = (
            (b.astype(np.int16) > g.astype(np.int16) + channel_delta)
            & (b.astype(np.int16) > r.astype(np.int16) + channel_delta)
        )
        mask = hsv_mask | (channel_mask.astype(np.uint8) * 255)
    else:
        raise ValueError(f"unsupported target_color: {target_color}")

    mask = cv2.medianBlur(mask, 3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, odd_kernel(close_kernel_size), iterations=close_iterations)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, odd_kernel(open_kernel_size), iterations=1)
    return mask


def contour_score(
    contour: np.ndarray,
    min_area: float,
    shape_epsilon_ratio: float,
    min_vertices: int,
    max_vertices: int,
    min_solidity: float,
) -> tuple[float, Optional[np.ndarray]]:
    area = cv2.contourArea(contour)
    if area < min_area:
        return -1.0, None

    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 1e-6:
        return -1.0, None

    approx = cv2.approxPolyDP(contour, shape_epsilon_ratio * perimeter, True)
    vertex_count = len(approx)
    if vertex_count < min_vertices or vertex_count > max_vertices:
        return -1.0, approx

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 1e-6:
        return -1.0, approx

    solidity = area / hull_area
    if solidity < min_solidity:
        return -1.0, approx

    x, y, w, h = cv2.boundingRect(approx)
    if h <= 0:
        return -1.0, approx

    aspect = w / float(h)
    aspect_penalty = abs(math.log(max(aspect, 1e-6)))
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    vertex_score = max(0.0, 1.0 - abs(vertex_count - 5) / 3.0)
    convex_score = 1.0 if cv2.isContourConvex(approx) else 0.55
    aspect_score = max(0.0, 1.5 - aspect_penalty)
    score = area * (0.45 + 0.3 * vertex_score + 0.15 * convex_score + 0.06 * circularity + 0.04 * aspect_score)
    return score, approx


def pick_best_contour(
    contours: list[np.ndarray],
    min_area: float,
    shape_epsilon_ratio: float,
    min_vertices: int,
    max_vertices: int,
    min_solidity: float,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    best_contour = None
    best_approx = None
    best_score = -1.0
    for contour in contours:
        score, approx = contour_score(contour, min_area, shape_epsilon_ratio, min_vertices, max_vertices, min_solidity)
        if score > best_score:
            best_score = score
            best_contour = contour
            best_approx = approx
    return best_contour, best_approx


def contour_center(contour: np.ndarray, approx: Optional[np.ndarray]) -> Optional[DetectionResult]:
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
        vertex_count=int(len(approx)) if approx is not None else 0,
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
        self.declare_parameter("shape_epsilon_ratio", 0.03)
        self.declare_parameter("hsv_s_min", 45)
        self.declare_parameter("hsv_v_min", 35)
        self.declare_parameter("channel_delta", 35)
        self.declare_parameter("close_kernel_size", 7)
        self.declare_parameter("close_iterations", 1)
        self.declare_parameter("open_kernel_size", 3)
        self.declare_parameter("min_vertices", 4)
        self.declare_parameter("max_vertices", 7)
        self.declare_parameter("min_solidity", 0.65)

        self.bridge = CvBridge()
        self.last_detection: Optional[DetectionResult] = None

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.target_color = self.get_parameter("target_color").get_parameter_value().string_value
        self.min_area = float(self.get_parameter("min_area").value)
        self.display = bool(self.get_parameter("display").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.preview_scale = float(self.get_parameter("preview_scale").value)
        self.shape_epsilon_ratio = float(self.get_parameter("shape_epsilon_ratio").value)
        self.hsv_s_min = int(self.get_parameter("hsv_s_min").value)
        self.hsv_v_min = int(self.get_parameter("hsv_v_min").value)
        self.channel_delta = int(self.get_parameter("channel_delta").value)
        self.close_kernel_size = int(self.get_parameter("close_kernel_size").value)
        self.close_iterations = int(self.get_parameter("close_iterations").value)
        self.open_kernel_size = int(self.get_parameter("open_kernel_size").value)
        self.min_vertices = int(self.get_parameter("min_vertices").value)
        self.max_vertices = int(self.get_parameter("max_vertices").value)
        self.min_solidity = float(self.get_parameter("min_solidity").value)

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(Float32MultiArray, "target_center_px", 10)
        self.mask_pub = self.create_publisher(Image, "target_mask", qos_profile_sensor_data)
        self.shape_mask_pub = self.create_publisher(Image, "target_shape_mask", qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, "target_debug", qos_profile_sensor_data)

        self.get_logger().info(
            f"listening on {image_topic}, target_color={self.target_color}, min_area={self.min_area}, "
            f"shape_epsilon_ratio={self.shape_epsilon_ratio}, hsv_s_min={self.hsv_s_min}, "
            f"hsv_v_min={self.hsv_v_min}, channel_delta={self.channel_delta}"
        )

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = build_color_mask(
            frame,
            hsv,
            self.target_color,
            self.hsv_s_min,
            self.hsv_v_min,
            self.channel_delta,
            self.close_kernel_size,
            self.close_iterations,
            self.open_kernel_size,
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour, approx = pick_best_contour(
            contours,
            self.min_area,
            self.shape_epsilon_ratio,
            self.min_vertices,
            self.max_vertices,
            self.min_solidity,
        )

        debug = frame.copy()
        shape_mask = np.zeros(mask.shape, dtype=np.uint8)
        out = Float32MultiArray()
        if contour is not None:
            result = contour_center(contour, approx)
            if result is not None:
                cv2.drawContours(shape_mask, [contour], -1, 255, thickness=cv2.FILLED)
                self.last_detection = result
                out.data = [result.center_x, result.center_y, result.area, float(result.vertex_count)]
                cv2.rectangle(
                    debug,
                    (result.bbox_x, result.bbox_y),
                    (result.bbox_x + result.bbox_w, result.bbox_y + result.bbox_h),
                    (0, 255, 0),
                    2,
                )
                cv2.circle(debug, (int(round(result.center_x)), int(round(result.center_y))), 5, (0, 255, 255), -1)
                if approx is not None and len(approx) >= 2:
                    cv2.polylines(debug, [approx], True, (255, 0, 0), 2)
                cv2.putText(
                    debug,
                    f"({result.center_x:.1f}, {result.center_y:.1f}) area={result.area:.0f} v={result.vertex_count}",
                    (result.bbox_x, max(0, result.bbox_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                self.center_pub.publish(out)
                self.get_logger().debug(
                    f"detected center=({result.center_x:.1f}, {result.center_y:.1f}) area={result.area:.1f} "
                    f"vertices={result.vertex_count}"
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

            shape_mask_msg = self.bridge.cv2_to_imgmsg(shape_mask, encoding="mono8")
            shape_mask_msg.header = msg.header
            self.shape_mask_pub.publish(shape_mask_msg)

        if self.display:
            if self.preview_scale > 0 and abs(self.preview_scale - 1.0) > 1e-6:
                debug_view = cv2.resize(
                    debug, None, fx=self.preview_scale, fy=self.preview_scale, interpolation=cv2.INTER_AREA
                )
                mask_view = cv2.resize(
                    mask, None, fx=self.preview_scale, fy=self.preview_scale, interpolation=cv2.INTER_NEAREST
                )
                shape_mask_view = cv2.resize(
                    shape_mask, None, fx=self.preview_scale, fy=self.preview_scale, interpolation=cv2.INTER_NEAREST
                )
            else:
                debug_view = debug
                mask_view = mask
                shape_mask_view = shape_mask
            cv2.imshow("target_debug", debug_view)
            cv2.imshow("target_color_mask", mask_view)
            cv2.imshow("target_shape_mask", shape_mask_view)
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

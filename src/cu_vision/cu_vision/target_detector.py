from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
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


@dataclass
class PnPResult:
    rvec: np.ndarray
    tvec: np.ndarray
    reprojection_error: float
    image_points: np.ndarray


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

    _, _, w, h = cv2.boundingRect(approx)
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


def find_pentagon_approx(contour: np.ndarray, base_epsilon_ratio: float) -> Optional[np.ndarray]:
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 1e-6:
        return None
    ratios = [
        base_epsilon_ratio,
        0.02,
        0.025,
        0.03,
        0.035,
        0.04,
        0.05,
        0.06,
    ]
    seen = set()
    for ratio in ratios:
        key = round(float(ratio), 4)
        if key in seen:
            continue
        seen.add(key)
        approx = cv2.approxPolyDP(contour, ratio * perimeter, True)
        if len(approx) == 5:
            return approx
    return None


def polygon_centroid(points: np.ndarray) -> np.ndarray:
    x = points[:, 0]
    y = points[:, 1]
    cross = x * np.roll(y, -1) - np.roll(x, -1) * y
    area = 0.5 * np.sum(cross)
    if abs(area) < 1e-9:
        return np.mean(points, axis=0)
    cx = np.sum((x + np.roll(x, -1)) * cross) / (6.0 * area)
    cy = np.sum((y + np.roll(y, -1)) * cross) / (6.0 * area)
    return np.array([cx, cy], dtype=np.float32)


def house_object_points(side_m: float) -> np.ndarray:
    tri_h = math.sqrt(3.0) * side_m / 2.0
    pts_2d = np.array(
        [
            [0.0, side_m + tri_h],
            [side_m / 2.0, side_m],
            [side_m / 2.0, 0.0],
            [-side_m / 2.0, 0.0],
            [-side_m / 2.0, side_m],
        ],
        dtype=np.float32,
    )
    centroid = polygon_centroid(pts_2d)
    pts_2d = pts_2d - centroid
    return np.column_stack([pts_2d, np.zeros(5, dtype=np.float32)]).astype(np.float32)


def pentagon_point_orders(approx: np.ndarray) -> list[np.ndarray]:
    pts = approx.reshape(-1, 2).astype(np.float32)
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ring = pts[np.argsort(angles)]
    apex_idx = int(np.argmax(np.linalg.norm(ring - center, axis=1)))
    order = np.roll(ring, -apex_idx, axis=0)
    reversed_order = np.concatenate([order[:1], order[:0:-1]], axis=0)
    return [order.astype(np.float32), reversed_order.astype(np.float32)]


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(rotation)))
        if idx == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


class TargetDetector(Node):
    def __init__(self) -> None:
        super().__init__("target_detector")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
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
        self.declare_parameter("filter_enable", True)
        self.declare_parameter("min_confirm_frames", 3)
        self.declare_parameter("max_missed_frames", 5)
        self.declare_parameter("filter_alpha", 0.35)
        self.declare_parameter("max_center_jump_px", 220.0)
        self.declare_parameter("enable_pnp", True)
        self.declare_parameter("target_side_m", 1.0)
        self.declare_parameter("max_reprojection_error_px", 8.0)
        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)

        self.bridge = CvBridge()
        self.last_detection: Optional[DetectionResult] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
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
        self.filter_enable = bool(self.get_parameter("filter_enable").value)
        self.min_confirm_frames = int(self.get_parameter("min_confirm_frames").value)
        self.max_missed_frames = int(self.get_parameter("max_missed_frames").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.max_center_jump_px = float(self.get_parameter("max_center_jump_px").value)
        self.enable_pnp = bool(self.get_parameter("enable_pnp").value)
        self.target_side_m = float(self.get_parameter("target_side_m").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)

        fx = float(self.get_parameter("camera_fx").value)
        fy = float(self.get_parameter("camera_fy").value)
        cx = float(self.get_parameter("camera_cx").value)
        cy = float(self.get_parameter("camera_cy").value)
        if fx > 0.0 and fy > 0.0:
            self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

        self.object_points = house_object_points(self.target_side_m)
        self.filtered_center: Optional[np.ndarray] = None
        self.filtered_tvec: Optional[np.ndarray] = None
        self.filtered_rvec: Optional[np.ndarray] = None
        self.hit_count = 0
        self.missed_count = 0

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )
        self.center_pub = self.create_publisher(Float32MultiArray, "target_center_px", 10)
        self.filtered_center_pub = self.create_publisher(Float32MultiArray, "target_center_px_filtered", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "target_pose_camera", 10)
        self.mask_pub = self.create_publisher(Image, "target_mask", qos_profile_sensor_data)
        self.shape_mask_pub = self.create_publisher(Image, "target_shape_mask", qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, "target_debug", qos_profile_sensor_data)

        self.get_logger().info(
            f"listening on {image_topic}, target_color={self.target_color}, min_area={self.min_area}, "
            f"filter_enable={self.filter_enable}, enable_pnp={self.enable_pnp}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1), dtype=np.float64)
        self.get_logger().info("camera intrinsics received from CameraInfo")

    def update_filter(self, result: Optional[DetectionResult], pnp: Optional[PnPResult]) -> bool:
        if result is None:
            self.missed_count += 1
            if self.missed_count > self.max_missed_frames:
                self.filtered_center = None
                self.filtered_tvec = None
                self.filtered_rvec = None
                self.hit_count = 0
            return False

        center = np.array([result.center_x, result.center_y], dtype=np.float64)
        if self.filtered_center is not None and self.max_center_jump_px > 0.0:
            jump = float(np.linalg.norm(center - self.filtered_center))
            if self.hit_count >= self.min_confirm_frames and jump > self.max_center_jump_px:
                self.missed_count += 1
                return False

        alpha = min(max(self.filter_alpha, 0.0), 1.0)
        if self.filtered_center is None or self.missed_count > self.max_missed_frames:
            self.filtered_center = center
        else:
            self.filtered_center = alpha * center + (1.0 - alpha) * self.filtered_center

        if pnp is not None:
            if self.filtered_tvec is None or self.filtered_rvec is None:
                self.filtered_tvec = pnp.tvec.astype(np.float64)
                self.filtered_rvec = pnp.rvec.astype(np.float64)
            else:
                self.filtered_tvec = alpha * pnp.tvec + (1.0 - alpha) * self.filtered_tvec
                self.filtered_rvec = alpha * pnp.rvec + (1.0 - alpha) * self.filtered_rvec

        self.hit_count += 1
        self.missed_count = 0
        return self.hit_count >= self.min_confirm_frames

    def solve_target_pnp(self, contour: np.ndarray) -> Optional[PnPResult]:
        if not self.enable_pnp or self.camera_matrix is None:
            return None

        pentagon = find_pentagon_approx(contour, self.shape_epsilon_ratio)
        if pentagon is None:
            return None

        best: Optional[PnPResult] = None
        for image_points in pentagon_point_orders(pentagon):
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                continue
            projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            projected = projected.reshape(-1, 2)
            error = float(np.mean(np.linalg.norm(projected - image_points, axis=1)))
            if error > self.max_reprojection_error_px:
                continue
            if best is None or error < best.reprojection_error:
                best = PnPResult(rvec=rvec, tvec=tvec, reprojection_error=error, image_points=image_points)
        return best

    def publish_filtered_center(self, result: DetectionResult) -> None:
        if self.filtered_center is None:
            return
        msg = Float32MultiArray()
        msg.data = [
            float(self.filtered_center[0]),
            float(self.filtered_center[1]),
            float(result.area),
            float(result.vertex_count),
            float(self.hit_count),
        ]
        self.filtered_center_pub.publish(msg)

    def publish_pose(self, header, pnp: Optional[PnPResult]) -> None:
        if pnp is None or self.filtered_tvec is None or self.filtered_rvec is None:
            return
        rotation, _ = cv2.Rodrigues(self.filtered_rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)

        msg = PoseStamped()
        msg.header = header
        if not msg.header.frame_id:
            msg.header.frame_id = "camera"
        msg.pose.position.x = float(self.filtered_tvec[0])
        msg.pose.position.y = float(self.filtered_tvec[1])
        msg.pose.position.z = float(self.filtered_tvec[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

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
        result = None
        pnp = None

        if contour is not None:
            result = contour_center(contour, approx)
            if result is not None:
                pnp = self.solve_target_pnp(contour)
                cv2.drawContours(shape_mask, [contour], -1, 255, thickness=cv2.FILLED)
                self.last_detection = result

                raw_msg = Float32MultiArray()
                raw_msg.data = [result.center_x, result.center_y, result.area, float(result.vertex_count)]
                self.center_pub.publish(raw_msg)

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
                if pnp is not None:
                    for point in pnp.image_points:
                        cv2.circle(debug, (int(point[0]), int(point[1])), 4, (255, 255, 0), -1)
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
                if pnp is not None:
                    cv2.putText(
                        debug,
                        f"pnp z={float(pnp.tvec[2]):.2f}m err={pnp.reprojection_error:.1f}px",
                        (result.bbox_x, result.bbox_y + result.bbox_h + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

        confirmed = self.update_filter(result, pnp) if self.filter_enable else result is not None
        if result is not None and confirmed:
            self.publish_filtered_center(result)
            self.publish_pose(msg.header, pnp)

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

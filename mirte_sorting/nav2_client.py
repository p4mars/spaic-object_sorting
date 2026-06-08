"""
nav2_client.py
──────────────
Nav2 navigation wrapper with three safety layers:
  1. Pre-flight sensor checks before moving
  2. In-flight sensor monitoring every 50 ms
  3. Recovery manoeuvres (leverages MIRTE mecanum wheels for strafing)

The mission planner calls only:
    nav.warm_up(3.0)
    ok = nav.go_to(pose, "Source Station")
    pose = nav.get_current_pose_from_tf()
    nav.spin_sensors(0.05)

MIRTE Master hardware topics:
    /mirte_base_controller/cmd_vel_unstamped  — velocity commands
    /scan       — RPLidar C1 (360°)
    /odom       — wheel encoder odometry
    /imu/data   — MPU-9250
    /mirte/distance/left|right  — HC-SR04 sonars
"""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Float32

# ── MIRTE topic names ──────────────────────────────────────────────────────────
CMD_VEL_TOPIC = "/mirte_base_controller/cmd_vel_unstamped"
SCAN_TOPIC    = "/scan"
ODOM_TOPIC    = "/odom"
IMU_TOPIC     = "/imu/data"
SONAR_L_TOPIC = "/mirte/distance/left"
SONAR_R_TOPIC = "/mirte/distance/right"

# ── Safety thresholds ──────────────────────────────────────────────────────────
LIDAR_HARD_STOP  = 0.30   # m — emergency stop
LIDAR_WARN       = 0.55   # m — slow-down warning
LIDAR_FRONT_ARC  = 45     # degrees — ±45° cone ahead
SONAR_HARD_STOP  = 0.20   # m
SONAR_MAX_RANGE  = 4.0    # m — HC-SR04 reliable limit
NAV_TIMEOUT_SEC  = 120    # s
SENSOR_STALE_SEC = 1.5    # s
ODOM_STALE_SEC   = 2.0    # s
MAX_LEG_RETRIES  = 3

# ── Recovery speeds ────────────────────────────────────────────────────────────
BACKUP_SPEED = 0.10   # m/s
STRAFE_SPEED = 0.12   # m/s  (mecanum: sideways)
SPIN_SPEED   = 0.45   # rad/s


class _SensorMonitor(Node):
    """Internal sensor subscriber node — never used directly by mission planner."""

    def __init__(self):
        super().__init__("nav2_sensor_monitor")

        _qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.scan: Optional[LaserScan] = None;  self.scan_t = 0.0
        self.odom: Optional[Odometry]  = None;  self.odom_t = 0.0
        self.imu:  Optional[Imu]       = None;  self.imu_t  = 0.0
        self.sl: float = float("inf");           self.sl_t   = 0.0
        self.sr: float = float("inf");           self.sr_t   = 0.0

        self.create_subscription(LaserScan, SCAN_TOPIC,   self._scan_cb, _qos)
        self.create_subscription(Odometry,  ODOM_TOPIC,   self._odom_cb, 10)
        self.create_subscription(Imu,       IMU_TOPIC,    self._imu_cb,  _qos)
        self.create_subscription(Range,   SONAR_L_TOPIC, self._sl_range_cb, _qos)
        self.create_subscription(Range,   SONAR_R_TOPIC, self._sr_range_cb, _qos)
        self.create_subscription(Float32, SONAR_L_TOPIC, self._sl_float_cb, _qos)
        self.create_subscription(Float32, SONAR_R_TOPIC, self._sr_float_cb, _qos)

    def _scan_cb(self, m): self.scan  = m; self.scan_t = time.time()
    def _odom_cb(self, m): self.odom  = m; self.odom_t = time.time()
    def _imu_cb(self,  m): self.imu   = m; self.imu_t  = time.time()

    def _sl_range_cb(self, m):
        if m.range > m.min_range: self.sl = float(m.range)
        self.sl_t = time.time()

    def _sr_range_cb(self, m):
        if m.range > m.min_range: self.sr = float(m.range)
        self.sr_t = time.time()

    def _sl_float_cb(self, m): self.sl = float(m.data); self.sl_t = time.time()
    def _sr_float_cb(self, m): self.sr = float(m.data); self.sr_t = time.time()

    def lidar_fresh(self): return time.time() - self.scan_t < SENSOR_STALE_SEC
    def odom_fresh(self):  return time.time() - self.odom_t < ODOM_STALE_SEC
    def imu_fresh(self):   return time.time() - self.imu_t  < ODOM_STALE_SEC

    def lidar_front_min(self) -> float:
        if self.scan is None:
            return float("inf")
        half = math.radians(LIDAR_FRONT_ARC)
        vals = []
        for i, r in enumerate(self.scan.ranges):
            a = self.scan.angle_min + i * self.scan.angle_increment
            a = (a + math.pi) % (2 * math.pi) - math.pi
            if abs(a) <= half and self.scan.range_min < r < 12.0:
                vals.append(r)
        return min(vals) if vals else float("inf")

    def lidar_all_min(self) -> float:
        if self.scan is None:
            return float("inf")
        valid = [r for r in self.scan.ranges if self.scan.range_min < r < 12.0]
        return min(valid) if valid else float("inf")

    def sonar_min(self) -> float:
        valid = [d for d in (self.sl, self.sr) if 0.01 < d < SONAR_MAX_RANGE]
        return min(valid) if valid else float("inf")

    def sonar_left_blocked(self)  -> bool: return 0.01 < self.sl < SONAR_HARD_STOP
    def sonar_right_blocked(self) -> bool: return 0.01 < self.sr < SONAR_HARD_STOP

    def tilt_ok(self) -> bool:
        if self.imu is None:
            return True
        q = self.imu.orientation
        roll  = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x**2 + q.y**2))
        pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        return abs(roll) < math.radians(20) and abs(pitch) < math.radians(20)

    def current_xy(self):
        if self.odom is None:
            return None, None
        p = self.odom.pose.pose.position
        return p.x, p.y


class Nav2Client:
    """
    Three-layer safety wrapper around Nav2's BasicNavigator.
    Only go_to() needs to be called from the mission planner.
    """

    def __init__(self, navigator: BasicNavigator):
        self._nav     = navigator
        self._sensors = _SensorMonitor()
        self._log     = navigator.get_logger()

        self._recovery_node = rclpy.create_node("nav2_recovery_driver")
        self._cmd_pub = self._recovery_node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    # ── Pre-flight checks ──────────────────────────────────────────────────────

    def _preflight(self, label: str) -> bool:
        self._log.info(f"  Pre-flight: '{label}'")
        if not self._sensors.lidar_fresh():
            self._log.error("  [FAIL] RPLidar stale — is /scan publishing?")
            return False
        self._log.info("  [OK] LiDAR fresh")

        if not self._sensors.odom_fresh():
            self._log.error("  [FAIL] Odometry stale — is /odom publishing?")
            return False
        self._log.info("  [OK] Odometry fresh")

        if self._sensors.imu_fresh() and not self._sensors.tilt_ok():
            self._log.error("  [FAIL] IMU tilt > 20°")
            return False
        self._log.info("  [OK] IMU tilt OK")

        front = self._sensors.lidar_front_min()
        if front < LIDAR_HARD_STOP:
            self._log.error(f"  [FAIL] Obstacle {front:.2f}m ahead (min {LIDAR_HARD_STOP}m)")
            return False
        if front < LIDAR_WARN:
            self._log.warn(f"  [WARN] Object at {front:.2f}m — Nav2 will route around")
        else:
            self._log.info(f"  [OK] Path clear ({front:.2f}m)")
        return True

    # ── In-flight monitoring ───────────────────────────────────────────────────

    def _monitor(self, nav_start: float) -> str:
        elapsed = time.time() - nav_start
        if elapsed > NAV_TIMEOUT_SEC:
            return "timeout"
        if not self._sensors.lidar_fresh():
            self._log.warn("  LiDAR went stale during navigation!")
            return "stale"
        if self._sensors.imu_fresh() and not self._sensors.tilt_ok():
            self._log.error("  IMU tilt > 20° — EMERGENCY STOP")
            return "tilt"
        front = self._sensors.lidar_front_min()
        if front < LIDAR_HARD_STOP:
            self._log.warn(f"  LIDAR STOP: {front:.2f}m ahead")
            return "obstacle"
        if front < LIDAR_WARN:
            self._log.warn(f"  Obstacle closing: {front:.2f}m")
        if self._sensors.sonar_min() < SONAR_HARD_STOP:
            self._log.warn(f"  SONAR STOP: L={self._sensors.sl:.2f}m R={self._sensors.sr:.2f}m")
            return "sonar"
        if self._nav.isTaskComplete():
            return "done"
        fb = self._nav.getFeedback()
        if fb:
            eta  = Duration.from_msg(fb.estimated_time_remaining).nanoseconds / 1e9
            dist = fb.distance_remaining
            x, y = self._sensors.current_xy()
            pos  = f"({x:.2f},{y:.2f})" if x is not None else "?"
            self._log.info(f"  → dist={dist:.2f}m  ETA={eta:.0f}s  pos={pos}  t={elapsed:.0f}s")
        return "ok"

    # ── Recovery ───────────────────────────────────────────────────────────────

    def _send_vel(self, vx: float, vy: float, wz: float, dur: float):
        twist = Twist()
        twist.linear.x  = float(vx)
        twist.linear.y  = float(vy)
        twist.angular.z = float(wz)
        end = time.time() + dur
        while time.time() < end:
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_pub.publish(Twist())
        time.sleep(0.1)

    def _stop(self):
        self._cmd_pub.publish(Twist())
        time.sleep(0.1)

    def _recover(self, reason: str):
        self._log.warn(f"  Recovery: {reason}")
        if not self._nav.isTaskComplete():
            self._nav.cancelTask()
            time.sleep(0.5)

        obs_l = self._sensors.sonar_left_blocked()
        obs_r = self._sensors.sonar_right_blocked()

        if reason in ("obstacle", "sonar"):
            self._stop()
            time.sleep(0.2)
            if obs_l and not obs_r:
                self._log.warn("  Escape: obstacle LEFT → strafe right + back + spin CW")
                self._send_vel(0.0, -STRAFE_SPEED, 0.0, 0.8)
                self._send_vel(-BACKUP_SPEED, 0.0, 0.0, 0.8)
                self._send_vel(0.0, 0.0, -SPIN_SPEED, 2.0)
            elif obs_r and not obs_l:
                self._log.warn("  Escape: obstacle RIGHT → strafe left + back + spin CCW")
                self._send_vel(0.0, STRAFE_SPEED, 0.0, 0.8)
                self._send_vel(-BACKUP_SPEED, 0.0, 0.0, 0.8)
                self._send_vel(0.0, 0.0, SPIN_SPEED, 2.5)
            else:
                self._log.warn("  Escape: back + spin CCW")
                self._send_vel(-BACKUP_SPEED, 0.0, 0.0, 1.2)
                self._send_vel(0.0, 0.0, SPIN_SPEED, 2.5)
        elif reason == "tilt":
            self._log.error("  TILT — stop only, do not move!")
            self._stop()
        else:
            self._log.warn("  Generic recovery: back + spin")
            self._send_vel(-BACKUP_SPEED, 0.0, 0.0, 0.8)
            self._send_vel(0.0, 0.0, SPIN_SPEED, 2.0)
        self._log.warn("  Recovery complete.")

    # ── Current pose ───────────────────────────────────────────────────────────

    def get_current_pose_from_tf(
            self,
            target_frame: str = "map",
            source_frame: str = "base_footprint",
            fallback_from_odom: bool = True) -> Optional[PoseStamped]:
        try:
            import tf2_ros
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, self._sensors)
            deadline = time.time() + 3.0
            while time.time() < deadline:
                rclpy.spin_once(self._sensors, timeout_sec=0.1)
                try:
                    t = buf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                    pose = PoseStamped()
                    pose.header.frame_id = target_frame
                    pose.header.stamp    = self._sensors.get_clock().now().to_msg()
                    pose.pose.position.x = t.transform.translation.x
                    pose.pose.position.y = t.transform.translation.y
                    pose.pose.position.z = t.transform.translation.z
                    pose.pose.orientation = t.transform.rotation
                    return pose
                except tf2_ros.TransformException:
                    pass
        except Exception as e:
            self._log.warn(f"TF lookup failed: {e}")

        if fallback_from_odom:
            rclpy.spin_once(self._sensors, timeout_sec=0.5)
            if self._sensors.odom is not None:
                pose = PoseStamped()
                pose.header = self._sensors.odom.header
                pose.pose   = self._sensors.odom.pose.pose
                self._log.warn("Using /odom for start pose (no SLAM correction).")
                return pose

        self._log.error("Cannot get current pose from TF or /odom.")
        return None

    # ── Main navigation call ───────────────────────────────────────────────────

    def go_to(self, pose: PoseStamped, label: str = "waypoint") -> bool:
        for attempt in range(MAX_LEG_RETRIES + 1):
            self._log.info(
                f"→ Navigation attempt {attempt + 1}/{MAX_LEG_RETRIES + 1}: '{label}'  "
                f"x={pose.pose.position.x:.2f}  y={pose.pose.position.y:.2f}")

            pre_ok = False
            for pf in range(5):
                rclpy.spin_once(self._sensors, timeout_sec=0.2)
                if self._preflight(label):
                    pre_ok = True
                    break
                self._log.warn(f"  Pre-flight not ready ({pf+1}/5) — waiting 1s...")
                time.sleep(1.0)

            if not pre_ok:
                self._log.error(f"  Pre-flight failed 5 times for '{label}'.")
                return False

            # Always refresh timestamp before sending to Nav2
            pose.header.stamp = self._nav.get_clock().now().to_msg()
            self._nav.goToPose(pose)
            nav_start = time.time()

            while rclpy.ok():
                rclpy.spin_once(self._sensors, timeout_sec=0.05)
                status = self._monitor(nav_start)

                if status == "ok":
                    time.sleep(0.05)
                    continue
                elif status == "done":
                    result = self._nav.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self._log.info(f"  ✓ Reached '{label}'")
                        return True
                    else:
                        self._log.error(f"  Nav2 {'CANCELED' if result == TaskResult.CANCELED else 'FAILED'} on '{label}'.")
                        self._recover("task_failed")
                        break
                else:
                    self._stop()
                    self._recover(status)
                    break

            if attempt < MAX_LEG_RETRIES:
                self._log.warn(f"  Retrying '{label}' in 2s ({MAX_LEG_RETRIES - attempt} left)...")
                time.sleep(2.0)

        self._log.error(f"All {MAX_LEG_RETRIES + 1} attempts failed for '{label}'.")
        return False

    # ── Utility ───────────────────────────────────────────────────────────────

    def warm_up(self, seconds: float = 3.0):
        self._log.info(f"Sensor warm-up ({seconds:.0f}s)...")
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self._sensors, timeout_sec=0.1)
        self._log.info(
            f"Warm-up done: lidar={self._sensors.lidar_fresh()}  "
            f"odom={self._sensors.odom_fresh()}  imu={self._sensors.imu_fresh()}")

    def spin_sensors(self, dt: float = 0.05):
        rclpy.spin_once(self._sensors, timeout_sec=dt)

#!/usr/bin/env python3
import math
import time
import requests
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ObstacleAvoidHTTP:
    """
    Commit:
    - add hysteresis param (kept for tuning)
    - add TURN state machine + turn_time for stable turning
    - add mild steering while moving forward based on left-right diff
    """

    def __init__(self):
        # ESP endpoint
        self.esp_url = rospy.get_param("~esp_url", "http://172.20.10.2/js")

        # Timeout semantics
        self.timeout = float(rospy.get_param("~http_timeout", 1.2))
        self.connect_timeout = float(rospy.get_param("~http_connect_timeout", 0.3))

        # Hard rate-limit for HTTP sends (seconds)
        self.min_send_dt = float(rospy.get_param("~min_send_dt", 0.15))
        self.last_sent_t = 0.0

        # Safety distances (meters)
        self.safe_front = float(rospy.get_param("~safe_front", 0.55))
        self.safe_side = float(rospy.get_param("~safe_side", 0.45))

        # Hysteresis (meters) to avoid jitter near thresholds
        self.hys = float(rospy.get_param("~hysteresis", 0.05))

        # Speeds
        self.fwd = float(rospy.get_param("~fwd_speed", 0.10))
        self.turn = float(rospy.get_param("~turn_speed", 0.12))

        # Sectors (degrees)
        self.front_deg = float(rospy.get_param("~front_sector_deg", 25.0))  # -25..+25
        self.left_from = float(rospy.get_param("~left_from_deg", 60.0))      # 60..120
        self.left_to = float(rospy.get_param("~left_to_deg", 120.0))
        self.right_from = float(rospy.get_param("~right_from_deg", -120.0))  # -120..-60
        self.right_to = float(rospy.get_param("~right_to_deg", -60.0))

        # Control loop rate (Hz)
        self.cmd_hz = float(rospy.get_param("~cmd_hz", 5.0))

        # Anti-spam threshold for commands
        self.cmd_eps = float(rospy.get_param("~cmd_eps", 0.01))
        self.last_sent = (None, None)

        # Mode
        self.mode = rospy.get_param("~mode", "STOP").upper()

        # Latest scan-derived distances
        self.last_front = float("inf")
        self.last_right = float("inf")
        self.last_left = float("inf")
        self.have_scan = False

        # Simple state machine for more stable turning
        self.state = "GO"          # GO or TURN
        self.turn_dir = 0          # +1 = turn left, -1 = turn right
        self.turn_until = 0.0
        self.turn_time = float(rospy.get_param("~turn_time", 0.45))  # seconds

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/robot_mode", String, self.on_mode, queue_size=5)

        # Timer-driven control loop
        rospy.Timer(rospy.Duration(1.0 / self.cmd_hz), self.control_loop)

        rospy.loginfo(
            f"[obstacle_http_node] Started. ESP={self.esp_url} mode={self.mode} cmd_hz={self.cmd_hz} "
            f"min_send_dt={self.min_send_dt}"
        )
        self.apply_mode_side_effects()

    def on_mode(self, msg: String):
        new_mode = (msg.data or "").strip().upper()
        if new_mode not in ("STOP", "MAP", "AUTO"):
            rospy.logwarn(f"[obstacle_http_node] Unknown mode '{msg.data}'. Use STOP/MAP/AUTO.")
            return
        if new_mode != self.mode:
            self.mode = new_mode
            rospy.loginfo(f"[obstacle_http_node] Mode -> {self.mode}")
            self.apply_mode_side_effects()

    def apply_mode_side_effects(self):
        if self.mode == "STOP":
            self.state = "GO"
            self.turn_dir = 0
            self.turn_until = 0.0
            self.send_lr(0.0, 0.0, force=True)

    def send_lr(self, left, right, force=False):
        left = float(left)
        right = float(right)

        # 1) anti-spam by "same command"
        if not force and self.last_sent[0] is not None:
            if abs(left - self.last_sent[0]) < self.cmd_eps and abs(right - self.last_sent[1]) < self.cmd_eps:
                return

        # 2) hard rate-limit
        now = time.time()
        if not force and (now - self.last_sent_t) < self.min_send_dt:
            return

        payload = {"T": 1, "L": left, "R": right}

        try:
            requests.post(
                self.esp_url,
                json=payload,
                timeout=(self.connect_timeout, self.timeout),
                headers={"Connection": "close"},
            )
            self.last_sent = (left, right)
            self.last_sent_t = time.time()
        except requests.RequestException as e:
            rospy.logwarn_throttle(2.0, f"[obstacle_http_node] HTTP error: {e}")

    def min_range_in_sector(self, scan: LaserScan, deg_from, deg_to):
        a_min = scan.angle_min
        a_inc = scan.angle_increment
        n = len(scan.ranges)

        if n == 0 or a_inc == 0.0:
            return float("inf")

        def idx_for_deg(deg):
            ang = math.radians(deg)
            i = int(round((ang - a_min) / a_inc))
            return clamp(i, 0, n - 1)

        i0 = idx_for_deg(deg_from)
        i1 = idx_for_deg(deg_to)
        if i0 > i1:
            i0, i1 = i1, i0

        best = None
        for r in scan.ranges[i0:i1 + 1]:
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue
            if best is None or r < best:
                best = r
        return best if best is not None else float("inf")

    def on_scan(self, scan: LaserScan):
        self.last_front = self.min_range_in_sector(scan, -self.front_deg, self.front_deg)
        self.last_left = self.min_range_in_sector(scan, self.left_from, self.left_to)
        self.last_right = self.min_range_in_sector(scan, self.right_from, self.right_to)
        self.have_scan = True

    def start_turn(self, direction):
        self.state = "TURN"
        self.turn_dir = 1 if direction >= 0 else -1
        self.turn_until = time.time() + self.turn_time

    def control_loop(self, _evt):
        if self.mode == "STOP":
            self.send_lr(0.0, 0.0)
            return

        if self.mode == "MAP":
            return

        if not self.have_scan:
            self.send_lr(0.0, 0.0)
            return

        front, left, right = self.last_front, self.last_left, self.last_right
        rospy.loginfo_throttle(
            1.0, f"[AUTO] front={front:.2f} left={left:.2f} right={right:.2f} state={self.state}"
        )

        now = time.time()

        if self.state == "TURN":
            if now < self.turn_until:
                if self.turn_dir > 0:
                    self.send_lr(-self.turn, self.turn)   # turn left
                else:
                    self.send_lr(self.turn, -self.turn)   # turn right
                return
            else:
                self.state = "GO"
                self.turn_dir = 0

        # Turn if obstacle ahead
        if front < self.safe_front:
            self.start_turn(+1 if left > right else -1)
            return

        # Nudge away if too close to a side
        if right < self.safe_side:
            self.start_turn(+1)
            return
        if left < self.safe_side:
            self.start_turn(-1)
            return

        # Mild steering while moving forward
        diff = (left - right)
        steer = clamp(diff * 0.10, -0.04, 0.04)
        self.send_lr(self.fwd - steer, self.fwd + steer)


def main():
    rospy.init_node("obstacle_http_node")
    ObstacleAvoidHTTP()
    rospy.spin()

if name == "__main__":
    main()
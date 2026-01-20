#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ObstacleAvoidHTTP:
    """
    Commit 1 skeleton:
    - subscribe /scan
    - compute simple front min distance
    - decision: forward or stop (no HTTP yet)
    """

    def __init__(self):
        # Parameters
        self.safe_front = float(rospy.get_param("~safe_front", 0.55))
        self.fwd = float(rospy.get_param("~fwd_speed", 0.10))
        self.cmd_hz = float(rospy.get_param("~cmd_hz", 5.0))
        self.front_deg = float(rospy.get_param("~front_sector_deg", 25.0))

        self.last_front = float("inf")
        self.have_scan = False

        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.cmd_hz), self.control_loop)

        rospy.loginfo("[avoidance] Commit1 started (no HTTP yet)")

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
        self.have_scan = True

    def set_motors(self, left, right):
        # placeholder for later HTTP
        rospy.loginfo_throttle(1.0, f"[motors] L={left:.2f} R={right:.2f}")

    def control_loop(self, _evt):
        if not self.have_scan:
            self.set_motors(0.0, 0.0)
            return

        front = self.last_front
        rospy.loginfo_throttle(1.0, f"[scan] front={front:.2f}")

        if front < self.safe_front:
            self.set_motors(0.0, 0.0)
        else:
            self.set_motors(self.fwd, self.fwd)


def main():
    rospy.init_node("obstacle_http_node")
    ObstacleAvoidHTTP()
    rospy.spin()


if __name__ == "__main__":
    main()
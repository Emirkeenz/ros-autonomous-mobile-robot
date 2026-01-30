#!/usr/bin/env python3
import os
import subprocess
import rospy
from std_msgs.msg import String


class MapAutosaver:
    def __init__(self):
        self.out_dir = rospy.get_param("~out_dir", os.path.expanduser("~/maps"))
        os.makedirs(self.out_dir, exist_ok=True)
        rospy.Subscriber("/save_map", String, self.on_save, queue_size=5)
        rospy.loginfo(f"[map_autosaver] Ready. out_dir={self.out_dir}. Publish to /save_map to save.")

    def on_save(self, msg: String):
        name = (msg.data or "").strip()
        if not name:
            name = "room_map"
        path = os.path.join(self.out_dir, name)
        rospy.loginfo(f"[map_autosaver] Saving map to {path}.pgm/.yaml")

        try:
            subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", path])
            rospy.loginfo("[map_autosaver] Saved OK.")
        except Exception as e:
            rospy.logerr(f"[map_autosaver] Save failed: {e}")


def main():
    rospy.init_node("map_autosaver")
    MapAutosaver()
    rospy.spin()


if __name__ == "__main__":
    main()

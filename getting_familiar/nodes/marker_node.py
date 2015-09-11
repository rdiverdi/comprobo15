#!/usr/bin/env python

"""publishes message to put a marker at 1, 2"""

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

rospy.init_node('marker_node')

point_msg = Point(x=1.0, y=2.0, z=0.0)
pose_msg = Pose(position=point_msg)
scale_msg = Vector3(x=1, y=1, z=1)
color_msg = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

header_msg = Header(stamp=rospy.Time.now(),
				    frame_id="odom")

msg = Marker(header=header_msg, pose=pose_msg, scale=scale_msg, color=color_msg)
Marker.type=2
pub = rospy.Publisher("/my_point", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	pub.publish(msg)
	r.sleep()

#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

def create_cube_marker(marker_id, position, scale, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "variable_cubes"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.w = 1.0  # no rotation

    marker.scale.x = scale[0]  # length
    marker.scale.y = scale[1]  # width
    marker.scale.z = scale[2]  # height

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.3 # fully visible

    marker.lifetime = rospy.Duration()
    return marker

def main():
    rospy.init_node('visualize_variable_cubes')
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)

    # 三个 cube 的 [位置]，[尺寸]，[颜色]
    cube_data = [
        # marker_id, position(x,y,z), scale(x,y,z), color(r,g,b)
        (0, (-0.75, 0, 0.4), (0.6, 2, 0.8), (0.0, 1.0, 0.0)),  # 
        (1, (0, 1.3, 0.4), (2.1, 0.5, 0.8), (0.0, 1.0, 0.0)),  # 绿色长方体
        (2, (0, -1.3, 0.4), (2.1, 0.5, 0.8), (0.0, 1.0, 0.0)),  # 蓝色长方体
        (3, (0.75, 0, 0.4), (0.6, 2, 0.8), (1.0, 0.0, 0.0)),  # 
    ]

    while not rospy.is_shutdown():
        for marker_id, pos, scale, color in cube_data:
            marker = create_cube_marker(marker_id, pos, scale, color)
            pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    main()
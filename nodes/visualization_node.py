#!/usr/bin/env python3

#from tdcr_visualization import tdcr_visualization
#from cdcr_visualization import cdcr_visualization
import rospy 
import tf.transformations
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import numpy as np
import tf
import time
import math


NODE_NAME = "visualization_node"
TIP_NAME = "/tip"
BASE_NAME = "/base"
REFERENCE_FRAMES_NAME = "/cs{}"
WORLD_NAME = "/world"

CDCR = "cdcr"
TDCR = "tdcr"

class Visualization_node:

    def create_base(self, frame, index, scale):
        base = Marker()
        base.header.frame_id = "world"
        base.header.stamp = rospy.Time.now()
        base.ns = "base"
        base.id = index
        base.type = visualization_msgs.msg.Marker.CUBE
        base.action = visualization_msgs.msg.Marker.ADD
        base.pose.position.x = frame[0][0]
        base.pose.position.y = frame[0][1]
        base.pose.position.z = frame[0][2]

        e = tf.transformations.euler_from_quaternion(frame[1])
        q = tf.transformations.quaternion_from_euler(e[0], e[1]+math.pi/2, e[2])
        
        base.pose.orientation.x = q[0]
        base.pose.orientation.y = q[1]
        base.pose.orientation.z = q[2]
        base.pose.orientation.w = q[3]
        base.scale.x = 0.2*scale
        base.scale.y = 0.2*scale
        base.scale.z = 0.005
        base.color.a = 1.00
        base.color.r = 0.9
        base.color.g = 0.9
        base.color.b = 0.9
        return base
    
    def __init__(self, total_frames, sections, scale) -> None:
        self.pub = rospy.Publisher("visualization_marker_array", visualization_msgs.msg.MarkerArray, queue_size=10)
        self.total_frames = total_frames
        self.listener = tf.TransformListener()
        self.sections = sections
        self.scale = scale

def process_param(param, default):
    if rospy.has_param(param) == False:
        return default
    else:
        return rospy.get_param(param)

if __name__ == "__main__":
    try:
        total_frames = process_param("total_frames", 1)
        arm_type = process_param("arm_type", "cdcr")
        scale = process_param("scale", 1)
        sections = np.array(process_param("sections", [total_frames]))
        density = process_param("density", 1)


        rospy.init_node(NODE_NAME)
        from tdcr_visualization import tdcr_visualization
        from cdcr_visualization import cdcr_visualization
        if (arm_type == TDCR):
            node = tdcr_visualization(total_frames, sections, scale, density)
        elif (arm_type == CDCR):
            node = cdcr_visualization(total_frames, sections, scale)
        else:
            raise Exception("Invalid continuum arm type")
        node.draw()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


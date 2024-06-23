import rospy 
import tf.transformations
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import numpy as np
import tf
import time
import math
from visualization_node import *

class tdcr_visualization(Visualization_node):

    def __init__(self, total_frames, sections, scale, density) -> None:
        self.density = density
        super().__init__(total_frames, sections, scale) 
    
    def add_line(self, line_list, start_frame, end_frame, dx=0, dy=0, dz=0):
        point = Point()
        point.x = start_frame[0][0] + dx
        point.y = start_frame[0][1] + dy
        point.z = start_frame[0][2] + dz
        point2 = Point()
        point2.x = end_frame[0][0] + dx
        point2.y = end_frame[0][1] + dy
        point2.z = end_frame[0][2] + dz
        line_list.points.append(point)
        line_list.points.append(point2)
        return line_list

    def create_line_list(self):
        line_list = Marker()
        line_list.header.frame_id = "world"
        line_list.header.stamp = rospy.Time.now()
        line_list.ns = "line_list"
        line_list.id = 0
        line_list.type = visualization_msgs.msg.Marker.LINE_LIST
        line_list.action = visualization_msgs.msg.Marker.ADD
        line_list.pose.position.x = 0.0
        line_list.pose.position.y = 0.0
        line_list.pose.position.z = 0.0
        line_list.pose.orientation.x = 0.0
        line_list.pose.orientation.y = 0.0
        line_list.pose.orientation.z = 0.0
        line_list.pose.orientation.w = 1.0
        line_list.scale.x = 0.005
        line_list.scale.y = 0.005
        line_list.scale.z = 0.005
        line_list.color.a = 1.0
        line_list.color.r = 1.0
        line_list.color.g = 1.0
        line_list.color.b = 1.0
        return line_list

    def create_tip(self, frame, index, scale):
        tip = Marker()
        tip.header.frame_id = "world"
        tip.header.stamp = rospy.Time.now()
        tip.ns = "tip"
        tip.id = index
        tip.type = visualization_msgs.msg.Marker.CUBE
        tip.action = visualization_msgs.msg.Marker.ADD
        tip.pose.position.x = frame[0][0]
        tip.pose.position.y = frame[0][1]
        tip.pose.position.z = frame[0][2]

        e = tf.transformations.euler_from_quaternion(frame[1])
        q = tf.transformations.quaternion_from_euler(e[0]+math.pi/4, e[1], e[2])
        
        tip.pose.orientation.x = q[0]
        tip.pose.orientation.y = q[1]
        tip.pose.orientation.z = q[2]
        tip.pose.orientation.w = q[3]
        tip.scale.x = 0.02
        tip.scale.y = 0.1*scale
        tip.scale.z = 0.1*scale
        tip.color.a = 1.00
        tip.color.r = 0.9
        tip.color.g = 0.9
        tip.color.b = 0.9
        return tip

    def create_cylinder(self, frame, index, scale, cur_section):
        cylinder = Marker()
        cylinder.header.frame_id = "world"
        cylinder.header.stamp = rospy.Time.now()
        cylinder.ns = "cylinders"
        cylinder.id = index
        cylinder.type = visualization_msgs.msg.Marker.CYLINDER
        cylinder.action = visualization_msgs.msg.Marker.ADD
        cylinder.pose.position.x = frame[0][0]
        cylinder.pose.position.y = frame[0][1]
        cylinder.pose.position.z = frame[0][2]

        e = tf.transformations.euler_from_quaternion(frame[1])
        q = tf.transformations.quaternion_from_euler(e[0], e[1]+math.pi/2, e[2])
        
        cylinder.pose.orientation.x = q[0]
        cylinder.pose.orientation.y = q[1]
        cylinder.pose.orientation.z = q[2]
        cylinder.pose.orientation.w = q[3]
        cylinder.scale.x = 0.1*scale
        cylinder.scale.y = 0.1*scale
        cylinder.scale.z = 0.01
        cylinder.color.a = 1.00
        cylinder.color.r = 0.9/(cur_section+1)
        cylinder.color.g = 0.9/(cur_section+1)
        cylinder.color.b = 0.9/(cur_section+1)
        return cylinder

    def draw(self):
        markerArray = MarkerArray()
        time.sleep(1.0)
        now = rospy.Time.now()
        self.listener.waitForTransform(WORLD_NAME, REFERENCE_FRAMES_NAME.format(1), now, rospy.Duration(5.0))

        while not rospy.is_shutdown():
            time.sleep(0.01)
            line_list = self.create_line_list()

            # processing base
            frame = self.listener.lookupTransform(WORLD_NAME, BASE_NAME, now)
            base = self.create_base(frame, 0, self.scale)
            markerArray.markers.append(base)
            previous_frame = frame
            cur_section = 0
            # processing frames
            for i in range(1, self.total_frames+1):
                if i == self.total_frames:
                    frame = self.listener.lookupTransform(WORLD_NAME, TIP_NAME, now)
                    tip = self.create_tip(frame, self.total_frames+2, self.scale)
                    markerArray.markers.append(tip)
                elif i%(1/self.density)==0:
                    frame = self.listener.lookupTransform(WORLD_NAME, REFERENCE_FRAMES_NAME.format(i), now)
                    cylinder = self.create_cylinder(frame, i, self.scale, cur_section)
                    markerArray.markers.append(cylinder)
                
                line_list = self.add_line(line_list, previous_frame, frame)
                line_list = self.add_line(line_list, previous_frame, frame, 0, 0.03*self.scale)
                line_list = self.add_line(line_list, previous_frame, frame, 0, -0.03*self.scale)
                line_list = self.add_line(line_list, previous_frame, frame, 0, 0, 0.03*self.scale)
                line_list = self.add_line(line_list, previous_frame, frame, 0, 0, -0.03*self.scale)
                previous_frame = frame

                if (cur_section < len(self.sections) and i > self.sections[cur_section]):
                    cur_section += 1
            
            markerArray.markers.append(line_list)

            # processing tip
            
            line_list = self.add_line(line_list, previous_frame, frame)

            # publish marker array
            self.pub.publish(markerArray)
            markerArray.markers.clear()
 
import rospy 
import tf.transformations
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import numpy as np
import tf
import time
import math
from visualization_node import *

class cdcr_visualization(Visualization_node):

    def __init__(self, total_frames, sections, scale) -> None:
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
        tip.scale.x = 0.01
        tip.scale.y = 0.03*scale
        tip.scale.z = 0.03*scale
        tip.color.a = 1.00
        tip.color.r = 0.9
        tip.color.g = 0.9
        tip.color.b = 0.9
        return tip
    
    def create_line_list(self, i):
        line_list = Marker()
        line_list.header.frame_id = "world"
        line_list.header.stamp = rospy.Time.now()
        line_list.ns = "line_list"
        line_list.id = i
        line_list.type = visualization_msgs.msg.Marker.LINE_LIST
        line_list.action = visualization_msgs.msg.Marker.ADD
        line_list.pose.position.x = 0.0
        line_list.pose.position.y = 0.0
        line_list.pose.position.z = 0.0
        line_list.pose.orientation.x = 0.0
        line_list.pose.orientation.y = 0.0
        line_list.pose.orientation.z = 0.0
        line_list.pose.orientation.w = 1.0
        line_list.scale.x = 0.01
        line_list.scale.y = 0.01
        line_list.scale.z = 0.01
        line_list.color.a = 1.0
        line_list.color.r = 1.0
        line_list.color.g = 1.0
        line_list.color.b = 1.0
        return line_list

    def draw(self):
        markerArray = MarkerArray()
        time.sleep(1.0)
        now = rospy.Time.now()
        self.listener.waitForTransform(WORLD_NAME, REFERENCE_FRAMES_NAME.format(1), now, rospy.Duration(5.0))
        while not rospy.is_shutdown():
            time.sleep(0.01)
            scale = self.scale

            # processing base
            frame = self.listener.lookupTransform(WORLD_NAME, BASE_NAME, now)
            base = self.create_base(frame, 0, self.scale)
            markerArray.markers.append(base)
            
            previous_frame = frame
            cur_section = -1
            line_list = None
            for i in range(1, self.total_frames+1):
                if cur_section == -1 or (cur_section < len(self.sections) and i > self.sections[cur_section]):
                    cur_section += 1
                    line_list = self.create_line_list(i) 
                    markerArray.markers.append(line_list)
                    line_list.scale.x = (1/((cur_section+1)*20))*scale
                    line_list.color.r = 0.9/(cur_section+1)
                    line_list.color.g = 0.9/(cur_section+1)
                    line_list.color.b = 0.9/(cur_section+1)

                if i == self.total_frames:
                    frame = self.listener.lookupTransform(WORLD_NAME, TIP_NAME, now)
                    tip = self.create_tip(frame, self.total_frames+2, self.scale)
                    line_list = self.add_line(line_list, previous_frame, frame)
                    markerArray.markers.append(tip)
                else:
                    id = REFERENCE_FRAMES_NAME.format(i)
                    frame = self.listener.lookupTransform(WORLD_NAME, id, now)
                    line_list = self.add_line(line_list, previous_frame, frame)

                previous_frame = frame
            self.pub.publish(markerArray)
            markerArray.markers.clear()
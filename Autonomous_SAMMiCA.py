#!/usr/bin/env python

from collections import deque
from ai28.msgs import BBox3d, Waypoint, Accident, Perception, weather

import rospy


class Wrapper():
    def init(self):
        self.bbox3d_deque = deque(30)
        self.waypoint_deque = deque(30)
        self.accident_deque = deque(30)
        self.weather_deque = deque(30)

        self.bbox3d_subscriber = rospy.Subscriber(
            "/ai28/bbox3d",
            BBox3d,
            self.bbox3d_callback,
            queue_size=1
        )
        
        self.waypoint_subscriber = rospy.Subscriber(
            "/ai28/waypoint",
            Waypoint,
            self._waypont_callback,
            queue_size=1
        )

        self.accident_subscriber = rospy.Subscriber(
            "/ai28/accident",
            Accident,
            self._accident_callback,
            queue_size=1
        )
        self.perception_msg_publisher = rospy.Publisher(
            "/ai28/weather",
            weather,
            self._weather_callback,
            queue_size=1
        )
        self.perception_msg_publisher = rospy.Publisher(
            "/ai28/perception",
            Perception,
            queue_size=1
        )

    def _bbox3d_callback(self, msg):
        self.bbox3d_deque.append(msg)
    
    def _wayponit_callback(self, msg):
        self.wayponit_deque.append(msg)

    def accident_callback(self, msg):
        self.accident_deque.append(msg)
    
    def weather_callback(self, msg):
        self.weather_deque.append(msg)

    def run_step():
        if len(self.bbox3d_deque==0) or len(self.waypoint_deque==0) or len(self.accident_deque==0):
            return
        
        last_bbox3d_msg = self.bbox3d_deque.pop()
        last_waypoint_msg = self.waypoint_deque.pop()
        last_accident_msg = self.accident_deque.pop()

        perception_msg = Perception(
            last_bbox3d_msg,
            last_waypoint_msg,
            last_accident_msg
        )

        self.perception_msg_publisher.publish(perception_msg)


if __name__ == "__main__":
    rospy.init_node("vehicle_controller_node")
    print("SAMMiCA autonomous module start")
    print("Insert module to CARLA-ROS autonomous vehicle")

    try:
        wrapper = Wrapper()
        frame_rate = rospy.get_param("~frame_rate", 200)

        rate = rospy.Rate(frame_rate)
        while not rospy.is_shutdown():
            wrapper.run_step()
            rate.sleep()

    except (rospy.ROSInterruptException, rospy.ROSException) as e:
        if not rospy.is_shutdown():
            rospy.logwarn("ROS Error during execution: {}".format(e))
    
    except KeyboardInterrupt:
        rospy.loginfo("User requested shut down.")
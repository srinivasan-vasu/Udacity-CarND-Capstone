#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
REFRESH_RATE = 10 # Refresh final waypoints at a rate of 10 Hz
MAX_DECEL = 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        self.stopline_wp_idx = -1
        self.base_waypoint_count = None
        self.waypoints_tree = None
        self.velocity = 0
        self.max_velocity = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(REFRESH_RATE)
        while not rospy.is_shutdown():

            if self.current_pose and self.base_waypoints:
                self.update_speedLimit()
                self.publish_waypoints()
            rate.sleep()

    def find_closest_waypoint_idx(self):
        # find the index of the base waypoint which is closest to the vehicle
        vehicle_coords = (self.current_pose.position.x, self.current_pose.position.y)
        closest_waypoint_idx = self.waypoints_tree.query(vehicle_coords, 1)[1]

        # verify that the vehicle is behind the detected nearest waypoint
        waypoint_coords = self.unpack_waypoint_coords(self.base_waypoints.waypoints[closest_waypoint_idx])
        previous_waypoint_coords = self.unpack_waypoint_coords(self.base_waypoints.waypoints[closest_waypoint_idx-1])
        if not self.is_behind(vehicle_coords, waypoint_coords, previous_waypoint_coords):
            # detected waypoint is behind vehicle, therefore select next waypoint
            closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.base_waypoints.waypoints)

        return closest_waypoint_idx

    def update_speedLimit(self):
        temp_velocity = 0.95*(self.kmph2mps(rospy.get_param('/waypoint_loader/velocity')))
        if self.max_velocity != temp_velocity:
                    self.max_velocity = temp_velocity
                    for waypoint in self.base_waypoints.waypoints:
                        waypoint.twist.twist.linear.x = self.max_velocity


    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.find_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
            # rospy.logwarn("GO")
        else:
            lane.waypoints = self.deceleration_waypoint(base_waypoints, closest_idx)
            # rospy.logwarn("STOP : %s, %s", self.stopline_wp_idx, closest_idx)
            # rospy.logwarn("velocity : %s", self.get_waypoint_velocity(lane.waypoints[-1]))
        
        return lane

    def deceleration_waypoint(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)# -2 to keep the car before the stop line not on it.
            dist = self.distance(waypoints, i, stop_idx)
            vel = 0.2*(MAX_DECEL* dist)

            if vel  < 1.0:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def aceleration_waypoint(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)# -2 to keep the car before the stop line not on it.
            dist = self.distance(waypoints, i, stop_idx)
            vel = 0.2*(MAX_DECEL* dist)

            if vel  < 1.0:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


    def unpack_waypoint_coords(self, waypoint):
        return (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)

    def is_behind(self, ref_point_coords, waypoint_coords, previous_waypoint_coords):
        # is reference point behind waypoint?
        (x, y) = ref_point_coords
        (wx, wy) = waypoint_coords
        (wprev_x, wprev_y) = previous_waypoint_coords

        # evaluate dot product defining plane perpendicular to road
        (v1x, v1y) = (wx - wprev_x, wy - wprev_y)
        (v2x, v2y) = (x - wx, y - wy)
        normal = v1x * v2x + v1y * v2y
        return normal < 0

    def construct_final_waypoints(self, start_idx):
        end_idx = start_idx + LOOKAHEAD_WPS
        final_waypoints = Lane()
        final_waypoints.header = self.base_waypoints.header
        final_waypoints.waypoints = self.base_waypoints.waypoints[start_idx:end_idx]
        return final_waypoints

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def velocity_cb(self,msg):        
        self.velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        if not self.base_waypoints:
          self.max_velocity = 0.95*(self.kmph2mps(rospy.get_param('/waypoint_loader/velocity')))

          self.base_waypoints = waypoints
          for waypoint in self.base_waypoints.waypoints:
                        waypoint.twist.twist.linear.x = self.max_velocity

          self.base_waypoint_count = len(waypoints.waypoints)
          waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
          self.waypoints_tree = KDTree(waypoints_xy)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

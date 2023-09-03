#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from  waypoint_follower.srv import AddWaypoint

lookahead_distance = 0.10
speed = 1.0

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

waypoints = [(0.0, 0.0)] 

class navigationControl():
    def __init__(self):
        rospy.init_node('Navigation', anonymous=True)
        rospy.Subscriber('/ground_truth', Odometry, self.info_callback)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path = waypoints
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.add_waypoint_callback)
        self.i = 0
        self.x = 0
        self.y = 0
        self.yaw = 0
        print("Waiting for waypoints...")

    def add_waypoint_callback(self, request):
        self.path.append((request.x, request.y))
        return {'success': True, 'message': 'Waypoint added.'}

    def timer_callback(self):
        twist = Twist()
        twist.linear.x , twist.angular.z, self.i = pure_pursuit(self.x, self.y, self.yaw, self.path, self.i)
        if abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1]) < 0.05:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

    def info_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.yaw = euler_from_quaternion(quaternion)

    def spin(self):
        rate = rospy.Rate(100)  
        while not rospy.is_shutdown():
            self.timer_callback()
            rate.sleep()

def main():
    navigation_control = navigationControl()
    try:
        navigation_control.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

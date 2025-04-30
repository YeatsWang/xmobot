#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class AckermannDriveNode:
    def __init__(self):
        rospy.init_node('ackermann_drive_node')

        # Robot parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.5)          # 前后轮轴距
        self.track_width = rospy.get_param('~track_width', 0.4)      # 左右轮间距
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.08)   # 轮子半径
        self.steer_mode = rospy.get_param('~steer_mode', 'front')    # 'front' or 'rear'

        # Publishers for wheel velocities (front/rear, left/right)
        self.wheel_pubs = {
            'front_left':  rospy.Publisher('/front_left_wheel_velocity_controller/command',  Float64, queue_size=1),
            'front_right': rospy.Publisher('/front_right_wheel_velocity_controller/command', Float64, queue_size=1),
            'rear_left':   rospy.Publisher('/rear_left_wheel_velocity_controller/command',   Float64, queue_size=1),
            'rear_right':  rospy.Publisher('/rear_right_wheel_velocity_controller/command',  Float64, queue_size=1),
        }

        # Publishers for steering angles (position controller)
        self.steer_pubs = {
            'front_left':  rospy.Publisher('/front_left_steering_controller/command',  Float64, queue_size=1),
            'front_right': rospy.Publisher('/front_right_steering_controller/command', Float64, queue_size=1),
            'rear_left':   rospy.Publisher('/rear_left_steering_controller/command',   Float64, queue_size=1),
            'rear_right':  rospy.Publisher('/rear_right_steering_controller/command',  Float64, queue_size=1),
        }

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.loginfo("AckermannDriveNode started with '%s' steering mode. Listening to /cmd_vel", self.steer_mode)
        rospy.spin()

    def cmd_callback(self, msg):
        vx = msg.linear.x   # m/s
        wz = msg.angular.z  # rad/s

        if abs(wz) < 1e-5:
            # Straight driving
            for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                if side in self.steer_pubs:
                    self.steer_pubs[side].publish(Float64(0.0))
            for side in self.wheel_pubs:
                self.wheel_pubs[side].publish(Float64(vx / self.wheel_radius))
        else:
            # Turning (Ackermann steering)
            radius = vx / wz
            if self.steer_mode == 'front':
                delta_left  = math.atan(self.wheelbase / (radius - self.track_width / 2))
                delta_right = math.atan(self.wheelbase / (radius + self.track_width / 2))
                self.steer_pubs['front_left'].publish(Float64(delta_left))
                self.steer_pubs['front_right'].publish(Float64(delta_right))
                # rear wheels drive
                R_rl = radius - self.track_width/2
                R_rr = radius + self.track_width/2
                v_rl = wz * R_rl
                v_rr = wz * R_rr
                self.wheel_pubs['rear_left'].publish(Float64(v_rl / self.wheel_radius))
                self.wheel_pubs['rear_right'].publish(Float64(v_rr / self.wheel_radius))
                # front wheels follow
                v_fl = vx
                v_fr = vx
                self.wheel_pubs['front_left'].publish(Float64(v_fl / self.wheel_radius))
                self.wheel_pubs['front_right'].publish(Float64(v_fr / self.wheel_radius))
            elif self.steer_mode == 'rear':
                delta_left  = math.atan(self.wheelbase / (radius - self.track_width / 2))
                delta_right = math.atan(self.wheelbase / (radius + self.track_width / 2))
                self.steer_pubs['rear_left'].publish(Float64(delta_left))
                self.steer_pubs['rear_right'].publish(Float64(delta_right))
                # front wheels drive
                R_fl = radius - self.track_width/2
                R_fr = radius + self.track_width/2
                v_fl = wz * R_fl
                v_fr = wz * R_fr
                self.wheel_pubs['front_left'].publish(Float64(v_fl / self.wheel_radius))
                self.wheel_pubs['front_right'].publish(Float64(v_fr / self.wheel_radius))
                # rear wheels follow
                v_rl = vx
                v_rr = vx
                self.wheel_pubs['rear_left'].publish(Float64(v_rl / self.wheel_radius))
                self.wheel_pubs['rear_right'].publish(Float64(v_rr / self.wheel_radius))

if __name__ == '__main__':
    try:
        AckermannDriveNode()
    except rospy.ROSInterruptException:
        pass

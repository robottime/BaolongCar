#!/usr/bin/env python
from motor_interface import HubMotorInterface

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3

import math
import time

odom_frame = '/odom'
base_frame = '/base_link'

left_wheel_port = rospy.get_param('left_wheel_port', "/dev/ttyUSB0")
right_wheel_port = rospy.get_param('right_wheel_port', "/dev/ttyUSB1")
left_wheel_debug = rospy.get_param('left_wheel_debug', False)
right_wheel_debug = rospy.get_param('right_wheel_debug', False)

wheel_diam = rospy.get_param('wheel_diameter', 0.2)
wheel_rr = rospy.get_param('wheel_reduction_ratio', 4)
motor_max_vel = rospy.get_param('motor_max_vel', 340)
robot_width = rospy.get_param('robot_width', 0.3)
enc_reso = rospy.get_param('encoder_resolution', 4096)

motors = dict(
    left = dict(
        motor = HubMotorInterface("left-motor",left_wheel_port, debug = left_wheel_debug),
        dir = -1,
        enc_cnt = 0,
    ),

    right = dict(
        motor = HubMotorInterface("right-motor",right_wheel_port, debug = right_wheel_debug),
        dir = 1,
        enc_cnt = 0,
    ),
)

def _callback(msg):
    set_vel(msg.linear.x,msg.angular.z)

def _terminate():
    motors['left']['motor'].end()
    motors['right']['motor'].end()

# TODO change to brake()
def stop():
    set_vel(0, 0)

_wheel_mpr = math.pi * wheel_diam / wheel_rr / 2.0 / math.pi
_motor_max_avel = motor_max_vel * 2 * math.pi / 60
# v: m/s
# w: rad/s
def set_vel(v, w):
    vl = (v - w * robot_width / 2) / _wheel_mpr
    vr = (v + w * robot_width / 2) / _wheel_mpr
    
    if math.fabs(vl) > _motor_max_avel or math.fabs(vr) > _motor_max_avel:
        return

    avel2speed = lambda v, dir=1: dir * v / 2.0 / math.pi * 60.0
    motors['left']['motor'].set_speed(avel2speed(vl, motors['left']['dir']))
    motors['right']['motor'].set_speed(avel2speed(vr, motors['right']['dir']))

_last_calc_time = 0
_wheel_mps = math.pi * wheel_diam / enc_reso / wheel_rr
_robot_pos = dict(x = 0, y = 0, a = 0)
def calc_odom():
    global _last_calc_time

    left_cnt = motors['left']['dir'] * motors['left']['motor'].get_posi_fee()
    right_cnt = motors['right']['dir'] * motors['right']['motor'].get_posi_fee()
    
    dt = time.time() - _last_calc_time
    last_calc_time = time.time()

    left_dcnt = left_cnt - motors['left']['enc_cnt']
    right_dcnt = right_cnt - motors['right']['enc_cnt']
    motors['left']['enc_cnt'] = left_cnt
    motors['right']['enc_cnt'] = right_cnt

    xx = (left_dcnt + right_dcnt) / 2 * _wheel_mps
    da = (right_dcnt - left_dcnt) / robot_width * _wheel_mps
    dx = xx * math.cos(_robot_pos['a'])
    dy = xx * math.sin(_robot_pos['a'])

    vx = dx / dt
    vy = dy / dt
    va = da / dt
    _robot_pos['x'] += dx
    _robot_pos['y'] += dy
    _robot_pos['a'] += da

    quat = tf.transformations.quaternion_from_euler(0, 0, _robot_pos['a'])
    return [
        Pose(Point(_robot_pos['x'], _robot_pos['y'], 0.), Quaternion(*quat)), 
        Twist(Vector3(vx,vy,0), Vector3(0,0,va))
    ]

def _pub_odom(odom_pub, odom_broadcaster):
    pose, twist = calc_odom()
    translation = (pose.position.x, pose.position.y, pose.position.z)
    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    odom_broadcaster.sendTransform(translation, rotation, rospy.Time.now(), base_frame,odom_frame)
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = odom_frame
    odom.child_frame_id = base_frame
    odom.pose.pose = pose
    odom.twist.twist = twist
    odom_pub.publish(odom)


if __name__=='__main__':
    rospy.init_node("two_wheel_diff_driver_node")
    rospy.Subscriber("/cmd_vel",Twist,_callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
    odom_broadcaster = tf.TransformBroadcaster()

    motors['left']['motor'].begin()
    motors['right']['motor'].begin()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        _pub_odom(odom_pub, odom_broadcaster)
        rate.sleep()

    rospy.spin()
    _terminate()

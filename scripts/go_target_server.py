#! /usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from sensor_msgs.msg import LaserScan

from go_target.srv import GoTarget
from go_target.srv import GoTargetResponse

import numpy as np
import math

server_is_on = True
start_point_received = False
start_point = [0.0, 0.0]
robot_point = [0.0, 0.0]
target_point = [0.0, 0.0]
s_r_vec = [0.0, 0.0]
s_t_vec = [0.0, 0.0]
r_pose_vec = [0.0, 0.0]
r_t_vec = [0.0, 0.0]
stop_distance = 0

vel_limit = 0.3
ang_vel_limit = 0.7

center_scan_val = 100000

def scanCallback(msg,degree=5):
  one_data_rad = (msg.angle_max - msg.angle_min)/len(msg.ranges)
  n = math.ceil(math.pi * degree / 180 / one_data_rad)
  start = int(len(msg.ranges)/2-n)
  finish = int(len(msg.ranges)/2+n+1)

  number_list = range(start, finish)
  data_list = []
  for i in number_list:
    data_list.append(msg.ranges[i])

  global center_scan_val
  center_scan_val = min(data_list)

def get_limit_vel(vel):
  if vel > vel_limit:
    vel = vel_limit
  if vel < -vel_limit:
    vel = -vel_limit
  return vel

def get_limit_ang_vel(ang_vel):
  if ang_vel > ang_vel_limit:
    ang_vel = ang_vel_limit
  if ang_vel < -ang_vel_limit:
    ang_vel = -ang_vel_limit
  return ang_vel
    
def set_vel(v, w):

  vel = Twist()
 
  vel.linear.x = v
  vel.angular.z = w

  vel_pub.publish(vel)

def calcAngle(v1, v2):
  v1size = np.sqrt(v1[0] ** 2 + v1[1] ** 2)
  v2size = np.sqrt(v2[0] ** 2 + v2[1] ** 2)

  inner = (v1[0] * v2[0]) + (v1[1] * v2[1])
  cos = inner / (v1size * v2size)

  rad = math.acos(cos)

  outer = v1[0] * v2[1] - v1[1] * v2[0]
  if outer < 0:
    rad *= -1

  return rad

def go_for_target():
  global r_pose_vec, r_t_vec

  while(1):
    print("debug")
    ang_vel = calcAngle(r_pose_vec, r_t_vec) * 2
    ang_vel = get_limit_ang_vel(ang_vel)
    set_vel(0, ang_vel)
    if abs(ang_vel) < 0.1:
      break

  while(1):
    ang_vel = calcAngle(r_pose_vec, r_t_vec) * 2
    ang_vel = get_limit_ang_vel(ang_vel)

    vel = (ang_vel_limit - abs(ang_vel))
    vel = get_limit_vel(vel)

    set_vel(vel, ang_vel)

    if center_scan_val < stop_distance + 0.3 and ang_vel < 0.3: 
      vel = 0.2

    if center_scan_val < stop_distance and ang_vel < 0.1:
      set_vel(0, 0)
      break 

def handle_go_target(req):
  global target_point, r_t_vec, stop_distance
  server_is_on = True
  start_point_received = False

  target_point[0] = req.target_point.x
  target_point[1] = req.target_point.y
  stop_distance = req.stop_distance

  go_for_target()

  server_is_on = False
  start_point_received = False
  return True

if __name__ == '__main__':
  rospy.init_node('go_target_server')
  listener = tf.TransformListener()
  vel_pub = rospy.Publisher('icart_mini/cmd_vel', Twist, queue_size=10)
  scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallback)
  s = rospy.Service('go_target', GoTarget, handle_go_target)
  rate = rospy.Rate(10.0)

  while not rospy.is_shutdown() and server_is_on:
    try:
      (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

      if start_point_received is False:
        start_point = [trans[0], trans[1]]
        start_point_received = True
      else:
        robot_point = [trans[0], trans[1]]
      angle = tf.transformations.euler_from_quaternion(rot)[2] 
      r_pose_vec = [math.cos(angle), math.sin(angle)]
      r_t_vec = [target_point[0] - robot_point[0], target_point[1] - robot_point[1]]

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    rate.sleep()
    

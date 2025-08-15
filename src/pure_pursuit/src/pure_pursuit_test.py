#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import pathReader,findLocalPath,purePursuit
from std_msgs.msg import Int64

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)


# 아이오닉 5 -> 조향값(servo_msg) 0일 때 직진 양수이면 좌회전 음수이면 우회전

DEFAULT_LFD = 18

# Mission Types for better organization
class MissionType:
    START = "START"
    STRAIGHT = "STRAIGHT"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    RAMP_UP = "RAMP_UP"
    RAMP_DOWN = "RAMP_DOWN"
    T_COURSE = "T_COURSE"
    TRAFFIC_LIGHT = "TRAFFIC_LIGHT"
    S_CURVE = "S_CURVE"
    PARALLEL_PARKING = "PARALLEL_PARKING"
    ACCELERATION = "ACCELERATION"
    DYNAMIC_OBSTACLE = "DYNAMIC_OBSTACLE"
    END = "END"

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # self.path_name = 'first'
        # self.path_name = 'second'
        self.path_name = 'new_test_path'

        self.passed_curve = False

        self.cnt = 0
        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', Bool, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size = 1)

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscriber
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCB)
        rospy.Subscriber("/cam_steer", CtrlCmd, self.laneCB) # Drive for camera
        rospy.Subscriber("/curve_cmd", CtrlCmd, self.curveCB) # curve Mission
        rospy.Subscriber("/dy_obs_info", Float64MultiArray, self.dyObsCB) # Drive for camera

        
        self.steering_angle_to_servo_offset = 0.0 ## servo moter offset
        self.target_x = 0.0
        self.target_y = 0.0

        self.ctrl_cmd_msg = CtrlCmd()

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link1'
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        self.yaw = 0.0
        self.yaw_rear = False

        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0
        self.accel_msg = 0

        self.traffic_signal = 0

        self.original_longitude = 0
        self.original_latitude = 0

        self.camera_servo_msg = 0.0
        self.camera_motor_msg = 10.0

        self.steering_offset = 0.015

        self.clear_stop_mission = False
        self.clear_start_mission = False


        self.dynamic_flag = False
        self.dynamic_done = False

        self.curve_servo_msg = 0.0
        self.curve_motor_msg = 0.0

        self.Mission = "line_drive"
        
        # Enhanced Mission Tracking
        self.current_mission = MissionType.START
        self.mission_completed = {
            MissionType.START: False,
            MissionType.RAMP_UP: False,
            MissionType.RAMP_DOWN: False,
            MissionType.T_COURSE: False,
            MissionType.TRAFFIC_LIGHT: False,
            MissionType.S_CURVE: False,
            MissionType.PARALLEL_PARKING: False,
            MissionType.ACCELERATION: False,
            MissionType.DYNAMIC_OBSTACLE: False
        }

        self.green_count = 0
        self.red_count = 0

        self.dy_obs_info = [0, 0, 0, 0]

        self.T_mission = False

        self.x, self.y = None, None
        self.br = tf.TransformBroadcaster()

        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)
        

        ######## For Service ########
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()

        self.forward_mode()
        ################################


        # Class
        self.path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import
        
        # Read path
        self.global_path = self.path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        # Time var
        count = 0
        rate = rospy.Rate(20)
        
        # Call the main loop
        self.main_loop()
        
    def get_current_mission_by_waypoint(self, waypoint):
        """Determine current mission based on waypoint for better organization"""
        if self.path_name == 'first':
            if waypoint <= 72:
                return MissionType.START
            elif 81 <= waypoint <= 165:
                return MissionType.RAMP_UP
            elif 163 <= waypoint <= 170:
                return MissionType.RAMP_UP  # Stop mission
            elif 280 <= waypoint <= 499:
                return MissionType.T_COURSE
            elif 585 <= waypoint <= 590 or 1049 <= waypoint <= 1055:
                return MissionType.TRAFFIC_LIGHT
            elif 690 <= waypoint <= 975:
                return MissionType.S_CURVE
            elif 975 <= waypoint <= 1410:
                return MissionType.DYNAMIC_OBSTACLE
            else:
                return MissionType.STRAIGHT
        elif self.path_name == 'second':
            if 30 <= waypoint <= 39:
                return MissionType.TRAFFIC_LIGHT
            elif 416 <= waypoint <= 507:
                return MissionType.ACCELERATION
            elif waypoint >= 825:
                return MissionType.END
            else:
                return MissionType.STRAIGHT
        else:
            return MissionType.STRAIGHT
    
    def log_mission_transition(self, new_mission, waypoint):
        """Log mission transitions for debugging"""
        if new_mission != self.current_mission:
            print(f"[MISSION CHANGE] {self.current_mission} -> {new_mission} at waypoint {waypoint}")
            self.current_mission = new_mission
            
    def setMotorMsgWithVel(self, velocity):
        """Set motor message with velocity - moved here for better organization"""
        self.motor_msg = velocity

    def setServoMsgWithLfd(self, lfd):
        """Set servo message with look-ahead distance - moved here for better organization"""
        self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(lfd)
        self.servo_msg = self.steering*self.steering_offset

    def setBrakeMsgWithNum(self, brake):
        """Set brake message with number - moved here for better organization"""
        self.brake_msg = brake
        
    def main_loop(self):
        """Main driving loop - organized for better readability"""
        rate = rospy.Rate(20)
                                           
        while not rospy.is_shutdown():
            # Initialize default values
            self.Mission = "line_drive"
            self.accel_msg = 0
  
            # Publish global path
            self.global_path_pub.publish(self.global_path)

            # Get current position and local path
            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)
            self.convertLL2UTM()
            self.next_start_waypoint = current_waypoint
            
            # Mission tracking and logging
            current_mission = self.get_current_mission_by_waypoint(current_waypoint)
            self.log_mission_transition(current_mission, current_waypoint)
            
            print(f"Current Waypoint: {current_waypoint}, Mission: {current_mission}")
           
            # Pure pursuit algorithm setup
            self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용

            if self.T_mission == False : 
                self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False) 
            else :
                if self.yaw_rear == True :
                    self.pure_pursuit.getEgoStatus(self.odom_msg, 5/2 +0.5, True)
                else :
                    self.pure_pursuit.getEgoStatus(self.odom_msg, 5/2 +0.5, False)

            self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)
            if self.yaw_rear == True :
                self.steering, self.target_x, self.target_y = self.pure_pursuit.rear_steering_angle()

            # Publish waypoint and mission status
            self.waypoint_pub.publish(current_waypoint)
            self.mission_pub.publish(self.passed_curve)

        ############################ 일반 주행 조향값 및 속도 설정 ##################################

            # 조향 값 확인 : rostopic echo /sensors/s            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)


            # cv2.imshow("original", cv2_image)ervo_position_command -> data
            # range : 0.0 ~ 1.0 (straight 0.5)

            self.ctrl_cmd_msg.longlCmdType = 2
            self.servo_msg = self.steering*self.steering_offset #+ self.steering_angle_to_servo_offset
            self.motor_msg = 19.0
            self.brake_msg = 0

        ###################################################################### 출발 미션 ###################################################################### 

            if self.path_name == 'first':
                if  current_waypoint <= 72:
                    self.setServoMsgWithLfd(8)
                    self.servo_msg /= 10
                    self.drive_left_signal()
                    continue
                elif current_waypoint <= 80  and self.clear_start_mission == False:
                    self.forward_mode()
                    self.clear_start_mission = True

        ###################################################################### 오르막길 정지 미션 ######################################################################

            if self.path_name == 'first':
                if 81 <= current_waypoint <= 165: # 오르막길 엑셀 밟기
                    self.setMotorMsgWithVel(19)

                if 163 <= current_waypoint <= 170 and self.clear_stop_mission == False:
                    self.ctrl_cmd_msg.accel = 0
                    self.ctrl_cmd_msg.acceleration = 0
                    self.brake()

                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    self.clear_stop_mission = True
                    rospy.sleep(4) # 4초 동안 정지
                    continue

        ###################################################################### 직각 코스 미션 ######################################################################

            if self.path_name == 'first':
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve == False :
                    self.motor_msg = self.curve_motor_msg
                    self.servo_msg = self.curve_servo_msg
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                    # #print("CURVE")


                if 280 <= current_waypoint <= 330: # Curve Slow Down
                    # #print("CURVE_START")
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
                    self.passed_curve = False
                    

                elif 331 <= current_waypoint <= 400 :  # 직각 코스 진입 Slow Down
                    # #print("CURVE_SLOW")
                    self.setMotorMsgWithVel(5)
                    self.setServoMsgWithLfd(3)

                if 450 <= current_waypoint <=  499 : # 직각 코스 종료 jikhu Slow Down
                    # #print("FINSH_CURVE")
                    self.setMotorMsgWithVel(9)
                    self.setServoMsgWithLfd(12)
                    self.passed_curve = True # 임시              
                    
                if 500 <= current_waypoint <=  555 : # 직각 코스 종료 Slow Down
                    # #print("FINSH_CURVE")
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
                    self.passed_curve = True # 임시

                if 556 <= current_waypoint <=  590 : # 직각 코스 종료 및 첫번째 신호등 전 Slow Down
                    # #print("FINSH_CURVE")
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(12)

                # if 597 <= current_waypoint :
                #     self.passed_curve = True

        # ###################################################################### S자 코스 미션  ######################################################################

        #     if self.path_name == 'first' :
        #         if 746 <= current_waypoint <= 820 : # S자 코스 진입 Slow down
        #             self.passed_curve = True
        #             self.setMotorMsgWithVel(6)
        #             self.setServoMsgWithLfd(3)

        #     if self.path_name == 'first' and self.passed_curve == True :
        #         # #print(self.yaw)
        #         if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve : # GPS 음영구역 진입 시 Camera Steering으로 주행
        #             self.setMotorMsgWithVel(15 - min(7, abs(self.camera_servo_msg*20)))
        #             if self.yaw <= - 25 : 
        #                 self.setMotorMsgWithVel(5)
        #             self.servo_msg = self.camera_servo_msg
        #             self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
        #             continue


        #         if 690 <= current_waypoint <= 745 : # S자 코스 진입 Slow down
        #             self.setMotorMsgWithVel(12)
        #             self.setServoMsgWithLfd(3)
                    
        #         # elif 746 <= current_waypoint <= 820 : # S자 코스 진입 Slow down
        #         #     self.setMotorMsgWithVel(6)
        #         #     self.setServoMsgWithLfd(3)

            
        #     ####### s자 끝나고 차선 복귀할 때까지는 천천히 달리기 #######
        #         if 950 <= current_waypoint <=  975 or current_waypoint == 902:
        #             #print('yaw',self.yaw)
        #             self.setMotorMsgWithVel(5)
        #             self.setServoMsgWithLfd(10)
        #             self.servo_msg *= 2
        #             if -92 <= self.yaw < -10 and self.camera_motor_msg != 10:
        #                 self.servo_msg = self.camera_servo_msg
        #             elif 960 <= current_waypoint and -92 <= self.yaw < -10 and self.camera_motor_msg == 10 :
        #                 self.servo_msg = -1
        #             self.Mission = "S_MISSION"
        #             self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
        #             # #print("S_MISSION_FINSH")
        #             # continue
        #         # if 960 <= current_waypoint <=  970 :
        #         #     self.setMotorMsgWithVel(3)
        #         #     self.setServoMsgWithLfd(3)
    ###################################################################### S자 코스 미션  ######################################################################
                if self.path_name == 'first' and 746 <= current_waypoint <= 820 : # S자 코스 진입 Slow down
                        self.setMotorMsgWithVel(6)
                        self.setServoMsgWithLfd(3)
                        self.passed_curve = True
                if self.path_name == 'first' and self.passed_curve == True :
                    print(self.yaw)
                    if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve : # GPS 음영구역 진입 시 Camera Steering으로 주행
                        self.setMotorMsgWithVel(15 - min(7, abs(self.camera_servo_msg*20)))
                        self.servo_msg = self.camera_servo_msg
                        if self.yaw < -25 : # -25 -> answer
                            self.setMotorMsgWithVel(6)
                        if -80 < self.yaw < -20:
                            print("yaw: ", self.yaw)
                            self.servo_msg = self.camera_servo_msg
                            self.servo_msg *= 2
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                        continue
                        #     # self.servo_msg *= 2
                    

                    if 690 <= current_waypoint <= 745 : # S자 코스 진입 Slow down
                        self.passed_curve = True
                        self.setMotorMsgWithVel(12)
                        self.setServoMsgWithLfd(3)
                        
                    # elif 746 <= current_waypoint <= 820 : # S자 코스 진입 Slow down
                    #     self.setMotorMsgWithVel(6)
                    #     self.setServoMsgWithLfd(3)
                    #     self.passed_curve = True

                
                ####### s자 끝나고 차선 복귀할 때까지는 천천히 달리기 ########################
                    if 903 <= current_waypoint <=  975: 
                        self.setMotorMsgWithVel(6)
                        self.setServoMsgWithLfd(2) # 2 -> answer? 
                        if -80 < self.yaw < -20:
                            # print("@@@@@@@@@@@@@@@")
                            self.servo_msg = self.camera_servo_msg
                            self.servo_msg *= 2
                        self.Mission = "S_MISSION"
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                        # #print("S_MISSION_FINSH")
                        continue

        ###################################################################### T자 주차 미션  ######################################################################

            # 후진 할 때는 조향값 반대, 속도는 느리게
            if (self.path_name == 'parking_1' or 
                self.path_name == 'parking_2' or 
                self.path_name == 'parking_4'):

                self.setMotorMsgWithVel(8)
                self.setServoMsgWithLfd(5)
                self.servo_msg *= 2

                self.T_mission = True
                if (self.path_name == 'parking_1' and current_waypoint + 20 >= len(self.global_path.poses)) : 
                    self.setMotorMsgWithVel(2)
                    self.setServoMsgWithLfd(5)
                    self.servo_msg *= 2
                elif (self.path_name == 'parking_4' and current_waypoint <= 24) :
                    self.setMotorMsgWithVel(2)
                    self.setServoMsgWithLfd(1)
                    self.servo_msg *= 2
                elif  (self.path_name == 'parking_4' and current_waypoint >= 25) :
                    self.setMotorMsgWithVel(8)
                    self.setServoMsgWithLfd(5)
                    self.servo_msg *= 2
            else:
                self.T_mission = False

            if self.path_name == 'second' and current_waypoint <= 25:
                self.setMotorMsgWithVel(8)
                self.setServoMsgWithLfd(5)
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue

            if self.yaw_rear == True :
                self.setMotorMsgWithVel(1)
                self.setServoMsgWithLfd(1)
                self.servo_msg *= -1
                if self.path_name == 'parking_3':
                    self.setServoMsgWithLfd(18)
                

            # T자 주차를 위한 path switching
            if current_waypoint + 5  >= len(self.global_path.poses) :
                if self.path_name == 'first':
                    self.path_name = 'parking_1'
                    self.global_path = self.path_reader.read_txt(self.path_name+".txt")
                    self.ctrl_cmd_msg.longlCmdType = 1
                    self.is_swith_path = False

                elif self.path_name == 'parking_1' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'parking_2'
                    self.global_path = self.path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.rear_mode()
                    continue

                elif self.path_name == 'parking_2' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'parking_3'
                    self.global_path = self.path_reader.read_txt(self.path_name+".txt")
      
                elif self.path_name == 'parking_3' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'parking_4'
                    self.global_path = self.path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.forward_mode()
                elif self.path_name == 'parking_4' and current_waypoint +5 >= len(self.global_path.poses): 
                    self.path_name = 'second'
                    self.global_path = self.path_reader.read_txt(self.path_name+".txt")
                    self.is_swith_path = False
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

            ###################################################################### 가속 미션 ######################################################################

            if self.path_name == 'second':
                if 374 <= current_waypoint <= 380 :
                    self.setMotorMsgWithVel(12+min(current_waypoint-374,6))
                    self.setServoMsgWithLfd(min(18,current_waypoint-371))
                   
                elif 381 <= current_waypoint <= 415 :
                    self.setMotorMsgWithVel(19)
                    self.setServoMsgWithLfd(18)
                    
                elif 416 <= current_waypoint <= 507: # 481 -> 461
                    self.setMotorMsgWithVel(40)
                    self.setServoMsgWithLfd(25)
                    self.servo_msg /= 4
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif 508 <= current_waypoint <= 545:
                    self.setMotorMsgWithVel(19)
                    self.setServoMsgWithLfd(25)
                    self.setBrakeMsgWithNum(0.95)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                else:
                    self.setMotorMsgWithVel(19)
                    self.setServoMsgWithLfd(20)
                    self.setBrakeMsgWithNum(0.0)

            ###################################################################### 종료 미션 ######################################################################

            if self.path_name == 'second':
                if  783 <= current_waypoint <= 825 :
                    self.drive_right_signal()

                if 825 <= current_waypoint:
                    self.brake()
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.parking()
                    continue


            ###################################################################### 곡선 코스 미션  ######################################################################

            if self.path_name == 'first':
                if (1215 <= current_waypoint <= 1260):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

                elif (1300 <= current_waypoint <= 1355):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
            
            elif self.path_name == 'second':
                if 39 <= current_waypoint <= 107:
                    self.setMotorMsgWithVel(10)
                    self.setServoMsgWithLfd(3)

                elif 212 <= current_waypoint <= 275:
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

                elif 325 <= current_waypoint <= 380:
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
                    
                elif 643 <= current_waypoint <= 707 :
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

            ##################################################################### 동적 장애물 미션 ######################################################################

            if self.path_name == 'first':
                if (975 <= current_waypoint <= 1040 or 
                    1160 <= current_waypoint <= 1214): 

                    if (0 < self.dy_obs_info[0] < 7.0 and 2.6 >= abs(self.dy_obs_info[1]) and not self.dynamic_done): # Dynamic Obstacle Detected
                        while(True):
                            self.brake()
                            self.emergency_mode()
                            self.dynamic_flag = True
                            # print("BRAKE")
                            print(abs(self.dy_obs_info[1]))
                            if (2 <= abs(self.dy_obs_info[1]) or self.dy_obs_info[1] == 0):
                                break
                        continue

                    if (self.dynamic_flag == True):
                        self.dynamic_done = True

                    if (self.dynamic_done == False):
                        self.setMotorMsgWithVel(10)
                        self.forward_mode()

                    else:
                        self.setMotorMsgWithVel(19)
                        self.forward_mode()

                elif (1350 <= current_waypoint <= 1410):
                    if (0 < self.dy_obs_info[0] < 7.0 and -3.5 <= self.dy_obs_info[1] <= 3.0 and not self.dynamic_done): # Dynamic Obstacle Detected
                        while(True):
                            self.brake()
                            self.emergency_mode()
                            self.dynamic_flag = True
                            # print("BRAKE")
                            print(abs(self.dy_obs_info[1]))
                            if (2 <= abs(self.dy_obs_info[1]) or self.dy_obs_info[1] == 0):
                                break
                        continue

                    if (self.dynamic_flag == True):
                        self.dynamic_done = True

                    if (self.dynamic_done == False):
                        self.setMotorMsgWithVel(8) 
                        self.forward_mode()

                    else:
                        self.setMotorMsgWithVel(19)
                        self.forward_mode()

            elif self.path_name == 'second': 
                if (107 <= current_waypoint <= 160 or 
                    575 <= current_waypoint <= 642):

                    if (0 < self.dy_obs_info[0] < 7.0 and 2.6 >= abs(self.dy_obs_info[1]) and not self.dynamic_done): # Dynamic Obstacle Detected
                        while(True):
                            self.brake()
                            self.emergency_mode()
                            self.dynamic_flag = True
                            # print("BRAKE")
                            print(abs(self.dy_obs_info[1]))
                            if (2 <= abs(self.dy_obs_info[1]) or self.dy_obs_info[1] == 0):
                                break
                        continue

                    if (self.dynamic_flag == True):
                        self.dynamic_done = True

                    if (self.dynamic_flag == False):
                        self.setMotorMsgWithVel(10)
                        self.setServoMsgWithLfd(3)
                        self.forward_mode()

                    else:
                        self.setMotorMsgWithVel(19)
                        self.forward_mode()
  
            ###################################################################### 신호등 미션 ######################################################################
            # 빨간불 -> 정지
            # 구간 출력을 위한 조건문  
            # if (self.path_name == 'final_path_second' and 70 <= current_waypoint <= 130) or (self.path_name == 'final_path_first' and (535 <= current_waypoint <= 542 or 965 <= current_waypoint <= 971)) :
            #     #print("Traffic Mission", current_waypoint)
            ########################################################################################################################################################
            
            if self.path_name == 'first':
                if 585 <= current_waypoint <= 590 and self.green_count - self.red_count <500: # 첫번째 신호등
                    self.brake()
            
                if  1049 <= current_waypoint <= 1055 and self.green_count - self.red_count <500: # 두번째 신호등
                    self.brake()
            

            if self.path_name == 'second': 
                if 34 <= current_waypoint <= 39 and (self.green_count - self.red_count < 600 or self.red_count > 150) : # 세번째 신호등
                    self.brake()
            
                if 30 <= current_waypoint <= 107: 
                    self.drive_left_signal()

                if 108 < current_waypoint <= 116 : 
                    self.forward_mode()

            ########################################################################################################################################################
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            # self.forward_mode()
            rate.sleep()
            ########################################################################################################################################################

###################################################################### Service Request  ######################################################################
    # option - 1 : ctrl_mode / 2 : gear / 4 : lamps / 6 : gear + lamps
    # gear - 1: P / 2 : R / 3 : N / 4 : D
##############################################################################################################################################################

    def forward_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def rear_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_left_signal(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_right_signal(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def emergency_mode(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.emergencySignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def parking(self) :
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 2
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 1.0
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
###################################################################### Call Back ######################################################################

    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude

        self.lat = msg.latitude
        self.lon = msg.longitude
        
        self.convertLL2UTM()

        self.br.sendTransform((self.x, self.y, 0.),
                         tf.transformations.quaternion_from_euler(0,0,0.),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        
        self.utm_msg = Float64MultiArray()

        self.utm_msg.data = [self.x, self.y]

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0

    def convertLL2UTM(self) :
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x, self.y = xy_zone[0], xy_zone[1]

    def ImuCB(self, msg) :
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw) 


    def trafficCB(self, msg):
        # Default : 0, Red : 1, Green :2
        self.green_count=msg.data[1]
        self.red_count=msg.data[0]

    def laneCB(self, msg) : 
        # #print(msg)
        self.camera_servo_msg = msg.steering
        self.camera_motor_msg = msg.velocity

    def curveCB(self, msg) :
        self.curve_servo_msg = msg.steering
        self.curve_motor_msg = msg.velocity

    def dyObsCB(self, msg):
        self.dy_obs_info = msg.data

###################################################################### Function ######################################################################

    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        # #print('pub', self.ctrl_cmd_msg.steering)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

# Helper functions moved to above for better organization


if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass
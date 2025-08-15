#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from warper import Warper
from slide_window_faster import SlideWindow


### 
# from warper import Warper
# from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import Float64, Int64, Bool
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from math import *

# 빠른 차선 인식 노드
# - 구독: /image_jpeg/compressed(CompressedImage), /gps(GPSMessage), /imu(Imu), /waypoint(Int64), /mission(Bool)
# - 발행: /cam_steer(CtrlCmd) - 속도/조향/가감속 명령
# - 처리 파이프라인: BGR→HSV → 차선 마스크 → 블러/이진화 → 원근변환(Warper) → 슬라이딩 윈도우(SlideWindow)
# - 제어 로직: 차선 중앙 오차(error_lane)와 곡률각(angle) 기반 속도/조향값 계산
class Controller() :

    def __init__(self) :
        rospy.init_node("line_detector")

        # 센서/상태 구독자 설정
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscriber

        # 영상/전처리/슬라이딩 윈도우 유틸 초기화
        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.original_img = []
        # 카메라/미션/웨이포인트 구독
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoint", Int64, self.waypointCB)
        self.mission_sub = rospy.Subscriber("/mission", Bool, self.mission_callback)
        self.yaw = 0.0
        self.waypoint = 0
        self.odom_msg = Odometry()

        # self.camera_steering_pub= rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # image_callback

        self.error_lane = 0
        self.steering_val = 0
        self.mission = False


        # gps 신호 받기 -> gps 끊길 때만 연산
        self.original_longitude = 0.0
        self.original_latitude = 0.0

        # For test
        self.motor_msg = 5.0
        # 제어 명령 주기 발행 루프 (20 Hz)
        rate = rospy.Rate(20) # 30hz
        while not rospy.is_shutdown():
            # Morai 시뮬레이터 제어 명령 퍼블리셔
            # self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
            self.ctrl_cmd_pub = rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)
            self.ctrl_cmd_msg = CtrlCmd()
            # self.motor_msg = 5.0
            self.servo_msg = 0.0
            self.brake_msg = 0.0
            self.ctrl_cmd_msg.longlCmdType = 2

            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            rate.sleep()




        # rospy.Subscriber("/image_jpeg/compressed", Image, self.image_callback)
        rospy.spin()
    
    # def runnung(self, _event) :

    # 미션 시작/종료 신호 수신
    def mission_callback(self, msg) :
        self.mission = msg.data
        # #print('lane CB', self.mission)

    # 현재 전역 웨이포인트 인덱스 수신
    def waypointCB(self, msg) :
        self.waypoint = msg.data
        # #print('way', self.waypoint)
        
        
    # GPS 수신: 기준 위치 저장(UTMK 사용 가정)
    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
    
    # IMU 쿼터니언을 yaw(도)로 변환
    def ImuCB(self, msg) :
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw) 


    def image_callback(self, _data) :
        # 실행 조건:
        # - 초기 위치가 유효하지 않고 미션 활성화, 또는 웨이포인트 범위(905~990)일 때 처리
        # #print(type(_data))
        # if self.initialized == False:
        #     cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
        #     cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
        #     self.initialized = True

    

        # 처리 활성화 조건 체크
        if (self.original_longitude  <= 1 and self.original_latitude <=  1 and self.mission == True) or  (905 <= self.waypoint <=  990):
            #print('lane')

            # CompressedImage → OpenCV BGR 이미지
            cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)
            # cv2.imshow("original", cv2_image)

            


            # low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


            # 차선(흰색) 추출을 위한 HSV 변환 및 범위 마스크
            hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
            # lower_lane = np.array([low_H, low_S, low_V]) 
            # upper_lane = np.array([high_H, high_S, high_V])

            # 흰색 차선 HSV 범위 (튜닝 필요 시 조정)
            lower_lane = np.array([0, 0, 126]) #0 46 170 / white lane: 0 0 126
            upper_lane = np.array([255, 64, 180]) #79 255 255 / white lane : 255, 64, 180


            lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)

            ksize = 5
            blur_lane = cv2.GaussianBlur(lane_mask, (ksize, ksize), 0)

            _, lane_image = cv2.threshold(blur_lane, 200, 255, cv2.THRESH_BINARY)

            # cv2.imshow("Lane Image", lane_image)

            # 시점 변환(탑뷰)으로 차선 검출 안정화
            warper_image = self.warper.warp(lane_image)

            # cv2.circle(cv2_image,  (0, 450),5,(255,0,0),5) #w h
            # cv2.circle(cv2_image, (160, 300),5,(0,0,255),5)

            # cv2.circle(cv2_image, (480, 300),5,(255,255,0),5)
            # cv2.circle(cv2_image, (640, 450),5,(0,255,255),5)

            # #print(cv2_image)
            # 슬라이딩 윈도우로 차선 위치(x)와 현재 추종 차선 방향 결정
            self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image, self.yaw)
            # cv2.imshow("slide_img", self.slide_img)
            

            gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)

            # low_B = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_G = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_R = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_B = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_G = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_R = cv2.getTrackbarPos('high_V', 'Simulator_Image')
            # 노란 커브/원뿔 등 특정 색상 영역 검출 후 HoughLinesP로 대표 선분 각도 산출
            lower_c = np.array([0, 156, 190]) 
            upper_c = np.array([219, 190, 255])

            s_image = cv2.inRange(cv2_image, lower_c, upper_c)
            # cv2.imshow("s", s_image)
            s_mask = cv2.inRange(s_image, 150, 255)
            blur_curve = cv2.GaussianBlur(s_mask, (ksize, ksize), 0)
            _, s_img = cv2.threshold(blur_curve, 200, 255, cv2.THRESH_BINARY)
            warper_s = self.warper.warp(s_img)

            self.curve_img, self.angle = self.slidewindow.curve(warper_s)
            # #print("angle", self.angle)
            # cv2.imshow("out", self.curve_img)





            # if self.yaw <= -20 :
            #     self.slide_x_location += 10

            # cv2.imshow("original", cv2_image)
            cv2.waitKey(1)
            # cv2.imshow("warper", warper_s)
            # cv2.waitKey(1)
        
            # #print("success")
            # try :
            #     cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)
            #     # cv2_image = self.bridge.imgmsg_to_cv2(_data)
            #     cv2_image = self.bridge.cv2_to_imgmsg(cv2_image, encoding="bgr8")
            #     # self.original_img = cv2_image
            #     cv2.imshow("original", cv2_image)
            #     #print("success")
            # except :
            #     #print("fail")
            #     pass



            # self.x_location : 현재 차선에서 중앙 값
            # middle x location : 320
        
            # 영상 중앙(320) 대비 차선 중심 x 오차
            self.error_lane = 320 - self.slide_x_location 

            # 차선 중심이 화면 좌우로 벗어나면 감속 및 큰 조향, 중앙 근처면 가속 및 작은 조향
            if self.slide_x_location < 270 or self.slide_x_location > 370:
                self.motor_msg = 3
                self.steering_val = self.error_lane * 0.003
            else:
                self.motor_msg = 5 
                self.steering_val = self.error_lane * 0.001

            # 큰 좌회전(yaw) 시, 검출된 선분 각도 기반으로 조향 강화
            if self.yaw < -25 and abs(self.angle) <= 75 :
                #print("####################")
                self.motor_msg = 3
                self.steering_val = (90 - abs(self.angle))*-0.7

        # self.camera_steering_pub.publish(self.steering_val)

    # Morai 제어 명령 발행 함수
    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):

        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = self.steering_val
        self.ctrl_cmd_msg.accel = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)






def nothing(x):
    pass


if __name__ == "__main__":
    control = Controller()
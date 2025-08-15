#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import numpy as np

class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)
         
        # 퍼스펙티브 변환 설정: src(입력) → dst(출력)

        # cv2.circle(cv2_image, (630,470),5,(255,0,0),5) #w h
        # cv2.circle(cv2_image, (480,250),5,(0,0,255),5)

        # cv2.circle(cv2_image, (10,470),5,(255,255,0),5)
        # cv2.circle(cv2_image, (160,250),5,(0,255,255),5)

        # 원본 영상에서의 4개 기준점 (픽셀 좌표)
        src = np.float32([ # 4개의 원본 좌표 점
            [0, 450],
            [160, 300],
            [480, 300],
            [640, 450]
        ])

        # 탑뷰(버드뷰) 형태로 펼쳐질 결과 좌표 4점
        dst = np.float32([ # 4개의 결과 좌표 점
            [160, h], # [416, 470.4] # 좌하
            [160, 300], # [224, 470.4] # 좌상
            [480,300], # [-192, 0] # 우상
            [480,h] # [832, 0] # 우하
        ])
        

        self.M = cv2.getPerspectiveTransform(src, dst) # self.M : 투시변환 행렬(src → dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # self.Minv : 투시변환 행렬(dst → src)

    # 입력 영상을 탑뷰로 변환
    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]), # img w, h
            flags=cv2.INTER_LINEAR
        )

    # 탑뷰 영상을 원본 시점으로 복원
    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )
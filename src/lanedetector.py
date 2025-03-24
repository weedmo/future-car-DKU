#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from ultralytics import YOLO
import os
import yaml
import numpy as np
import cv2
import time
from torchvision.transforms.functional import perspective
import torch

class LaneDetector:
    LANE_DOTTED = 0
    LANE_SOLID = 1

    def __init__(self, weight_fn, perspective_point_fn, yolo_size=(640,384), 
                       lane_width=500, draw=True, draw_warp=False, verbose=False,
                       crosswalk_mode=True):
        self.model = YOLO(weight_fn)
        self.load_points(perspective_point_fn)
        self.yolo_size = yolo_size   # [width, height]
        self.lane_width = lane_width
        self.draw = draw
        self.draw_warp = draw_warp
        self.verbose = verbose
        self.lst_y_test_points = (0, 50, 100, 150, 200, 250, 300)
        # self.yh = (self.lst_y_test_points[0]+self.lst_y_test_points[-1])/2
        # yh를 밑으로 내리면 xc가 카메라 바로 앞의 중앙이 될 수도 있을 것 같음.
        self.yh = self.lst_y_test_points[-1]
        self.image_square = [(0,0), (self.yolo_size[0],0), (self.yolo_size[0], self.yolo_size[1]), (0,self.yolo_size[1])]
        self.inv_perspective_mat = cv2.getPerspectiveTransform(np.float32(self.image_square), np.float32(self.perspective_points))

        self.idx=0
        self.points = []
        self.prev_xpos = 0
        self.prev_angle = 0

        self.crosswalk_mode = crosswalk_mode
        self.crosswalk_threshold = 75000
        self.crosswalk_detect_count = 0
        self.crosswalk_detect_count_threshold = 5

    def load_points(self, fn):
        if os.path.exists(fn):
            with open(fn) as f:
                saved_dict = yaml.load(f, Loader=yaml.FullLoader)
            
            self.perspective_points = []
            for i in range(4):
                self.perspective_points.append([saved_dict[i]['x'],saved_dict[i]['y']])

    def get_mask_img(self, img):
        img = cv2.resize(img.copy(), self.yolo_size)
        results = self.model(img,verbose=False)
        
        if results[0].cpu().masks == None:  # should be checked to omit .cpu() 
            return False, img, None

        mask_tensor = torch.zeros((3, self.yolo_size[1], self.yolo_size[0]), device='cuda')
        for i in range(len(results[0].masks)):
            cls_id = int(results[0].boxes[i].cls.item())
            mask_tensor[cls_id] += (results[0].masks[i].data).squeeze()
            
        mask_tensor *= 255
        warped_tensor = perspective(mask_tensor, self.perspective_points, self.image_square)

        mask_img = np.array(mask_tensor.to('cpu'), dtype=np.uint8)
        mask_img = np.ascontiguousarray(np.transpose(mask_img, (1,2,0)))
        
        return True, mask_img, warped_tensor

    def get_line_points(self, warped_tensor: torch.Tensor):
        found_x = []
        tangent = []
        for lane_type in range(2):
            nonzero = warped_tensor[lane_type].nonzero().cpu().numpy()
            nzx, nzy = nonzero[:,1], nonzero[:,0]
            found_x_points = []
            found_y_points = []
            for y in self.lst_y_test_points:
                good_inds = ((nzy >= y) & (nzy < y+50)).nonzero()[0]
                if len(good_inds) < 100:
                    continue
                x = np.mean(nzx[good_inds])
                found_x_points.append(x)
                found_y_points.append(y)
                
            if len(found_x_points) < 2:
                found_x.append(-1)
                tangent.append(0)
            else:
                fit = np.polyfit(found_x_points, found_y_points, 1)
                xh = self.yh / fit[0]
                norm_a = np.sqrt(xh**2+self.yh**2)
                if lane_type == self.LANE_DOTTED:
                    # 좌측통행인 경우는 아래 식을 else와 바꿔준다. 기본 우측통행 세팅
                    xc = self.yh*self.lane_width/norm_a + xh - fit[1]/fit[0]
                    yc = -xh*self.lane_width/norm_a + self.yh            
                else:
                    xc = -self.yh*self.lane_width/norm_a + xh - fit[1]/fit[0]
                    yc = +xh*self.lane_width/norm_a + self.yh          
                converted_point = self.inv_perspective_mat@np.array([[xc],[yc],[1]]).squeeze()
                found_x.append(converted_point[0] / converted_point[2])
                tangent.append(fit[0])
        
        return found_x, tangent
    
    def detect_crosswalk(self, warped_tensor):
        nonzero = warped_tensor[2].nonzero().cpu().numpy()
        if len(nonzero) > self.crosswalk_threshold:
            self.crosswalk_detect_count += 1
            if self.crosswalk_detect_count >= self.crosswalk_detect_count_threshold:
                return True
        else:
            self.crosswalk_detect_count = 0
            return False
        # print(len(nonzero))

    def do(self, img):
        t1 = time.time()
        img = cv2.resize(img, self.yolo_size)
        ret, mask_img, warped_tensor = self.get_mask_img(img)
        
        t2 = time.time()        
        found_crosswalk = False
        if ret:
            found_x, tangent = self.get_line_points(warped_tensor)
            if self.crosswalk_mode:
                found_crosswalk = self.detect_crosswalk(warped_tensor)
        else:
            found_x = [-1, -1]
        
        dotted_x, solid_x = found_x[0], found_x[1]
        xpos = self.prev_xpos
        angle = self.prev_angle

        if dotted_x == -1 and solid_x != -1:
            xpos = solid_x
            angle = tangent[1]
        elif dotted_x != -1 and solid_x == -1:
            xpos = dotted_x
            angle = tangent[0]
        elif dotted_x != -1 and solid_x != -1:
            if dotted_x > solid_x:
                # 점선과 실선을 모두 찾았는데 점선이 실선 우측이라면 실선은 반대 차선이라고 가정 (우측 주행인 경우)
                xpos = dotted_x
                angle = tangent[0]
            else:
                xpos = (dotted_x+solid_x)/2
                angle = tangent[1] # 두 개의 선을 찾으면 어차피 차선은 평행해야하니 실선의 기울기로

        angle = np.rad2deg(np.arctan(angle))
        angle = angle + 180 if angle < 0 else angle
        self.prev_xpos = xpos
        self.prev_angle = angle

        t3 = time.time()
        if self.draw:
            img = self.draw_result(img, ret, mask_img, xpos, angle, warped_tensor)
            ret = (int(xpos), int(angle), found_crosswalk, img)
        else:
            ret = (int(xpos), int(angle), found_crosswalk)
        t4 = time.time()

        if self.verbose:
            print(f'yolo+warping: {(t2-t1)*1000:0.1f}ms, xpos_point: {(t3-t2)*1000:0.1f}ms, drawing: {(t4-t3)*1000:0.1f}ms, total: {(t4-t1)*1000:0.1f}ms', end='  ')
            print(f'xpos={xpos:6.1f}, angle={angle:5.1f}      ', end='\r')
        
        return ret
        
    def draw_result(self, img, found, mask, xpos, angle, warped_tensor):
        img = cv2.polylines(img, [np.array(self.perspective_points)], True, (0,255,0), 2 )
        if found:
            img = cv2.addWeighted(img, 0.6, mask, 0.9, 0)
            # xc, yc = (int(xpos),int((self.perspective_points[0][1]+self.perspective_points[2][1])*0.5))
            xc, yc = (int(xpos), self.yolo_size[1])
            img = cv2.circle(img, (xc,yc), 10, (255,0,0), 5)
            xr = xc - int(np.cos(np.pi / 180 * angle)*100)
            yr = yc - int(np.sin(np.pi / 180 * angle)*100)
            img = cv2.line(img, (xc,yc), (xr,yr), (0,0,255), thickness=5)
            

            if self.draw_warp and warped_tensor != None:
                warped_img = np.array(warped_tensor.to('cpu'), dtype=np.uint8)
                warped_img = np.ascontiguousarray(np.transpose(warped_img, (1,2,0)))
                cv2.imshow('warped', warped_img)
                cv2.waitKey(1)
        return img


if __name__ == '__main__':
    os.system('clear')
    weight_fn = 'best.pt'
    persepctive_point_fn = 'perspective_points.yaml'
    ld = LaneDetector(weight_fn, persepctive_point_fn, draw=True, lane_width=590//2, verbose=True)

    cap = cv2.VideoCapture('test_videos/1stage.mp4')
    while(cap.isOpened()):
        ret, frame = cap.read()
        xpos, angle = ld.do(frame)
    cap.release()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time
from argparse import Namespace
import warnings
import yaml
import torch
import cv2
from imutils import paths
import rospy  # ROS Python 라이브러리
from std_msgs.msg import String  # ROS 메시지 타입

from lprnet import LPRNet, numpy2tensor, decode

warnings.filterwarnings("ignore")

class LPRNetInferenceNode:
    """LPRNet을 이용한 ROS Inference 노드 클래스"""

    def __init__(self):
        """초기화 메서드: 설정 파일 로드 및 모델 초기화"""
        rospy.init_node("lprnet_inference_node", anonymous=True)
        self.save_dir = '/home/jihoon/catkin_ws/ocr_ros/save_plate'
        self.save_path = os.path.join(self.save_dir, "plate_result.json")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.config_path = rospy.get_param("~config_path")
        self.load_config(self.config_path)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()
        self.result_pub = rospy.Publisher("lprnet_results", String, queue_size=10)
        self.plate_dictionary = {}

    def load_config(self, config_path):
        """YAML 설정 파일을 로드"""
        with open(config_path) as f:
            self.args = Namespace(**yaml.load(f, Loader=yaml.FullLoader))
        rospy.loginfo("Configuration loaded successfully")

    def load_model(self):
        """모델 로드 및 가중치 적용"""
        model = LPRNet(self.args).to(self.device).eval()
        model.load_state_dict(torch.load(self.args.pretrained)["state_dict"])
        rospy.loginfo("Model loaded successfully")
        return model
    
    def infer(self):
        """Inference 수행 메서드"""
        imgs = [el for el in paths.list_images(self.args.test_dir)]
        labels = [
            os.path.basename(n).split(".")[0].split("-")[0].split("_")[0]
            for n in imgs
        ]

        acc = []
        times = []

        for i, img in enumerate(imgs):
            im = numpy2tensor(cv2.imread(img), self.args.img_size).unsqueeze(0).to(self.device)

            # Inference 시작
            t0 = time.time()
            logit = self.model(im).detach().to("cpu")
            pred, _ = decode(logit, self.args.chars)
            t1 = time.time()

            acc.append(pred[0] == labels[i])
            times.append((t1 - t0) * 1000)

            # 결과 출력 및 퍼블리시
            result_msg = f"Image: {os.path.basename(img)}, Prediction: {pred[0]}, Label: {labels[i]}"
            self.plate_dictionary[os.path.basename(img)] = pred[0]

            rospy.loginfo(result_msg)
            self.result_pub.publish(result_msg)
        with open(self.save_path, 'w') as f:
            json.dump(self.plate_dictionary, f)
        # 정확도 및 시간 통계 출력
        self.log_statistics(acc, times)

    def log_statistics(self, acc, times):
        """정확도와 시간 통계 출력"""

        if len(acc) == 0:
            rospy.loginfo("no acc")
            return 

        rospy.loginfo("\n-----Accuracy-----")
        rospy.loginfo(
            f"correct: {sum(acc)}/{len(acc)}, "
            + f"incorrect: {len(acc) - sum(acc)}/{len(acc)}"
        )
        rospy.loginfo(f"accuracy: {sum(acc) / len(acc) * 100:.2f} %")
        rospy.loginfo("\n-----Inference Time-----")
        rospy.loginfo(f"mean: {sum(times) / len(times):.4f} ms")
        rospy.loginfo(f"max: {max(times):.4f} ms")
        rospy.loginfo(f"min: {min(times):.4f} ms")

    


if __name__ == "__main__":
    try:
        node = LPRNetInferenceNode()
        node.infer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion")
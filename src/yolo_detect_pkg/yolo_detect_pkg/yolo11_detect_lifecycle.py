#!/usr/bin/env python3

# Copyright (c) 2024，WuChao D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 注意: 此程序在RDK板端端运行
# Attention: This program runs on RDK board.

# 添加必要的导入
import cv2
import numpy as np
from scipy.special import softmax
from hobot_dnn import pyeasy_dnn as dnn
from time import time
import logging

# ROS 2 相关包
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State, Publisher
from lifecycle_msgs.msg import TransitionEvent

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 导入自定义消息需要的标准消息类型
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

# 日志模块配置
# logging configs
logging.basicConfig(
    level = logging.INFO,  # 调整为INFO级别，减少过多日志
    format = '[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S')
logger = logging.getLogger("RDK_YOLO")

class YOLO11_Detect():
    def __init__(self, model_path, classes_num=80, nms_thres=0.7, score_thres=0.25, reg=16):
        # 加载BPU的bin模型, 打印相关参数
        # Load the quantized *.bin model and print its parameters
        try:
            begin_time = time()
            self.quantize_model = dnn.load(model_path)
            logger.debug("\033[1;31m" + "Load D-Robotics Quantize model time = %.2f ms"%(1000*(time() - begin_time)) + "\033[0m")
        except Exception as e:
            logger.error("❌ Failed to load model file: %s"%(model_path))
            logger.error("You can download the model file from the following docs: ./models/download.md") 
            logger.error(e)
            raise e

        logger.info("\033[1;32m" + "-> input tensors" + "\033[0m")
        for i, quantize_input in enumerate(self.quantize_model[0].inputs):
            logger.info(f"intput[{i}], name={quantize_input.name}, type={quantize_input.properties.dtype}, shape={quantize_input.properties.shape}")

        logger.info("\033[1;32m" + "-> output tensors" + "\033[0m")
        for i, quantize_input in enumerate(self.quantize_model[0].outputs):
            logger.info(f"output[{i}], name={quantize_input.name}, type={quantize_input.properties.dtype}, shape={quantize_input.properties.shape}")

        # 将反量化系数准备好, 只需要准备一次
        # prepare the quantize scale, just need to generate once
        self.s_bboxes_scale = self.quantize_model[0].outputs[1].properties.scale_data[np.newaxis, :]
        self.m_bboxes_scale = self.quantize_model[0].outputs[3].properties.scale_data[np.newaxis, :]
        self.l_bboxes_scale = self.quantize_model[0].outputs[5].properties.scale_data[np.newaxis, :]
        logger.info(f"{self.s_bboxes_scale.shape=}, {self.m_bboxes_scale.shape=}, {self.l_bboxes_scale.shape=}")

        # DFL求期望的系数, 只需要生成一次
        # DFL calculates the expected coefficients, which only needs to be generated once.
        self.weights_static = np.array([i for i in range(16)]).astype(np.float32)[np.newaxis, np.newaxis, :]
        logger.info(f"{self.weights_static.shape = }")

        # anchors, 只需要生成一次
        self.s_anchor = np.stack([np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                            np.repeat(np.arange(0.5, 80.5, 1), 80)], axis=0).transpose(1,0)
        self.m_anchor = np.stack([np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                            np.repeat(np.arange(0.5, 40.5, 1), 40)], axis=0).transpose(1,0)
        self.l_anchor = np.stack([np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                            np.repeat(np.arange(0.5, 20.5, 1), 20)], axis=0).transpose(1,0)
        logger.info(f"{self.s_anchor.shape = }, {self.m_anchor.shape = }, {self.l_anchor.shape = }")

        # 输入图像大小, 一些阈值, 提前计算好
        self.input_image_size = 640
        self.SCORE_THRESHOLD = score_thres
        self.NMS_THRESHOLD = nms_thres
        self.CONF_THRES_RAW = -np.log(1/self.SCORE_THRESHOLD - 1)
        logger.info("SCORE_THRESHOLD  = %.2f, NMS_THRESHOLD = %.2f"%(self.SCORE_THRESHOLD, self.NMS_THRESHOLD))
        logger.info("CONF_THRES_RAW = %.2f"%self.CONF_THRES_RAW)

        self.input_H, self.input_W = self.quantize_model[0].inputs[0].properties.shape[2:4]
        logger.info(f"{self.input_H = }, {self.input_W = }")

        self.REG = reg
        logger.info(f"{self.REG = }")

        self.CLASSES_NUM = classes_num
        logger.info(f"{self.CLASSES_NUM = }")

    def preprocess_yuv420sp(self, img):
        RESIZE_TYPE = 0
        LETTERBOX_TYPE = 1
        PREPROCESS_TYPE = LETTERBOX_TYPE
        logger.info(f"PREPROCESS_TYPE = {PREPROCESS_TYPE}")

        begin_time = time()
        self.img_h, self.img_w = img.shape[0:2]
        if PREPROCESS_TYPE == RESIZE_TYPE:
            # 利用resize的方式进行前处理, 准备nv12的输入数据
            begin_time = time()
            input_tensor = cv2.resize(img, (self.input_W, self.input_H), interpolation=cv2.INTER_NEAREST) # 利用resize重新开辟内存节约一次
            input_tensor = self.bgr2nv12(input_tensor)
            self.y_scale = 1.0 * self.input_H / self.img_h
            self.x_scale = 1.0 * self.input_W / self.img_w
            self.y_shift = 0
            self.x_shift = 0
            logger.info("\033[1;31m" + f"pre process(resize) time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        elif PREPROCESS_TYPE == LETTERBOX_TYPE:
            # 利用 letter box 的方式进行前处理, 准备nv12的输入数据
            begin_time = time()
            self.x_scale = min(1.0 * self.input_H / self.img_h, 1.0 * self.input_W / self.img_w)
            self.y_scale = self.x_scale
            
            if self.x_scale <= 0 or self.y_scale <= 0:
                raise ValueError("Invalid scale factor.")
            
            new_w = int(self.img_w * self.x_scale)
            self.x_shift = (self.input_W - new_w) // 2
            x_other = self.input_W - new_w - self.x_shift
            
            new_h = int(self.img_h * self.y_scale)
            self.y_shift = (self.input_H - new_h) // 2
            y_other = self.input_H - new_h - self.y_shift
            
            input_tensor = cv2.resize(img, (new_w, new_h))
            input_tensor = cv2.copyMakeBorder(input_tensor, self.y_shift, y_other, self.x_shift, x_other, cv2.BORDER_CONSTANT, value=[127, 127, 127])
            input_tensor = self.bgr2nv12(input_tensor)
            logger.info("\033[1;31m" + f"pre process(letter box) time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        else:
            logger.error(f"illegal PREPROCESS_TYPE = {PREPROCESS_TYPE}")
            exit(-1)

        logger.debug("\033[1;31m" + f"pre process time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        logger.info(f"y_scale = {self.y_scale:.2f}, x_scale = {self.x_scale:.2f}")
        logger.info(f"y_shift = {self.y_shift:.2f}, x_shift = {self.x_shift:.2f}")
        return input_tensor

    def bgr2nv12(self, bgr_img):
        begin_time = time()
        height, width = bgr_img.shape[0], bgr_img.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.zeros_like(yuv420p)
        nv12[:height * width] = y
        nv12[height * width:] = uv_packed
        logger.debug("\033[1;31m" + f"bgr8 to nv12 time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        return nv12

    def forward(self, input_tensor):
        begin_time = time()
        quantize_outputs = self.quantize_model[0].forward(input_tensor)
        logger.debug("\033[1;31m" + f"forward time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        return quantize_outputs

    def c2numpy(self, outputs):
        begin_time = time()
        outputs = [dnnTensor.buffer for dnnTensor in outputs]
        logger.debug("\033[1;31m" + f"c to numpy time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")
        return outputs

    def postProcess(self, outputs):
        begin_time = time()
        # reshape
        s_clses = outputs[0].reshape(-1, self.CLASSES_NUM)
        s_bboxes = outputs[1].reshape(-1, self.REG * 4)
        m_clses = outputs[2].reshape(-1, self.CLASSES_NUM)
        m_bboxes = outputs[3].reshape(-1, self.REG * 4)
        l_clses = outputs[4].reshape(-1, self.CLASSES_NUM)
        l_bboxes = outputs[5].reshape(-1, self.REG * 4)

        # classify: 利用numpy向量化操作完成阈值筛选(优化版 2.0)
        s_max_scores = np.max(s_clses, axis=1)
        s_valid_indices = np.flatnonzero(s_max_scores >= self.CONF_THRES_RAW)  # 得到大于阈值分数的索引，此时为小数字
        s_ids = np.argmax(s_clses[s_valid_indices, : ], axis=1)
        s_scores = s_max_scores[s_valid_indices]

        m_max_scores = np.max(m_clses, axis=1)
        m_valid_indices = np.flatnonzero(m_max_scores >= self.CONF_THRES_RAW)  # 得到大于阈值分数的索引，此时为小数字
        m_ids = np.argmax(m_clses[m_valid_indices, : ], axis=1)
        m_scores = m_max_scores[m_valid_indices]

        l_max_scores = np.max(l_clses, axis=1)
        l_valid_indices = np.flatnonzero(l_max_scores >= self.CONF_THRES_RAW)  # 得到大于阈值分数的索引，此时为小数字
        l_ids = np.argmax(l_clses[l_valid_indices, : ], axis=1)
        l_scores = l_max_scores[l_valid_indices]

        # 3个Classify分类分支：Sigmoid计算
        s_scores = 1 / (1 + np.exp(-s_scores))
        m_scores = 1 / (1 + np.exp(-m_scores))
        l_scores = 1 / (1 + np.exp(-l_scores))

        # 3个Bounding Box分支：反量化
        s_bboxes_float32 = s_bboxes[s_valid_indices,:].astype(np.float32) * self.s_bboxes_scale
        m_bboxes_float32 = m_bboxes[m_valid_indices,:].astype(np.float32) * self.m_bboxes_scale
        l_bboxes_float32 = l_bboxes[l_valid_indices,:].astype(np.float32) * self.l_bboxes_scale

        # 3个Bounding Box分支：dist2bbox (ltrb2xyxy)
        s_ltrb_indices = np.sum(softmax(s_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        s_anchor_indices = self.s_anchor[s_valid_indices, :]
        s_x1y1 = s_anchor_indices - s_ltrb_indices[:, 0:2]
        s_x2y2 = s_anchor_indices + s_ltrb_indices[:, 2:4]
        s_dbboxes = np.hstack([s_x1y1, s_x2y2])*8

        m_ltrb_indices = np.sum(softmax(m_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        m_anchor_indices = self.m_anchor[m_valid_indices, :]
        m_x1y1 = m_anchor_indices - m_ltrb_indices[:, 0:2]
        m_x2y2 = m_anchor_indices + m_ltrb_indices[:, 2:4]
        m_dbboxes = np.hstack([m_x1y1, m_x2y2])*16

        l_ltrb_indices = np.sum(softmax(l_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        l_anchor_indices = self.l_anchor[l_valid_indices,:]
        l_x1y1 = l_anchor_indices - l_ltrb_indices[:, 0:2]
        l_x2y2 = l_anchor_indices + l_ltrb_indices[:, 2:4]
        l_dbboxes = np.hstack([l_x1y1, l_x2y2])*32

        # 大中小特征层阈值筛选结果拼接
        dbboxes = np.concatenate((s_dbboxes, m_dbboxes, l_dbboxes), axis=0)
        scores = np.concatenate((s_scores, m_scores, l_scores), axis=0)
        ids = np.concatenate((s_ids, m_ids, l_ids), axis=0)

        # xyxy 2 xyhw
        # xy = (dbboxes[:,2:4] + dbboxes[:,0:2])/2.0
        hw = (dbboxes[:,2:4] - dbboxes[:,0:2])
        # xyhw = np.hstack([xy, hw])

        xyhw2 = np.hstack([dbboxes[:,0:2], hw])

        # 分类别nms
        results = []
        for i in range(self.CLASSES_NUM):
            id_indices = ids==i
            indices = cv2.dnn.NMSBoxes(xyhw2[id_indices,:], scores[id_indices], self.SCORE_THRESHOLD, self.NMS_THRESHOLD)
            if len(indices) == 0:
                continue
            for indic in indices:
                x1, y1, x2, y2 = dbboxes[id_indices,:][indic]
                x1 = int((x1 - self.x_shift) / self.x_scale)
                y1 = int((y1 - self.y_shift) / self.y_scale)
                x2 = int((x2 - self.x_shift) / self.x_scale)
                y2 = int((y2 - self.y_shift) / self.y_scale)

                x1 = x1 if x1 > 0 else 0
                x2 = x2 if x2 > 0 else 0
                y1 = y1 if y1 > 0 else 0
                y2 = y2 if y2 > 0 else 0
                x1 = x1 if x1 < self.img_w else self.img_w
                x2 = x2 if x2 < self.img_w else self.img_w
                y1 = y1 if y1 < self.img_h else self.img_h
                y2 = y2 if y2 < self.img_h else self.img_h

                results.append((i, scores[id_indices][indic], x1, y1, x2, y2))

        logger.debug("\033[1;31m" + f"Post Process time = {1000*(time() - begin_time):.2f} ms" + "\033[0m")

        return results

# COCO数据集80个类别名称
coco_names = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", 
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", 
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", 
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", 
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", 
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", 
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", 
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

# 用于绘制检测框的颜色列表
rdk_colors = [
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),(49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
    (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0),(147, 69, 52), (255, 115, 100), (236, 24, 0), (255, 56, 132),
    (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255)]

def draw_detection(img, bbox, score, class_id) -> None:
    """
    Draws a detection bounding box and label on the image.

    Parameters:
        img (np.array): The input image.
        bbox (tuple[int, int, int, int]): A tuple containing the bounding box coordinates (x1, y1, x2, y2).
        score (float): The detection score of the object.
        class_id (int): The class ID of the detected object.
    """
    x1, y1, x2, y2 = bbox
    color = rdk_colors[class_id%20]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    label = f"{coco_names[class_id]}: {score:.2f}"
    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    label_x, label_y = x1, y1 - 10 if y1 - 10 > label_height else y1 + 10
    cv2.rectangle(
        img, (label_x, label_y - label_height), (label_x + label_width, label_y + label_height), color, cv2.FILLED
    )
    cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

# ROS 2生命周期节点类
class YoloDetectLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_detect_lifecycle_node')
        
        # 声明参数 - 这些在构造函数中声明，但在configure时使用
        self.declare_parameter('model_path', '/home/sunrise/rdk_model_zoo/demos/detect/YOLO11/YOLO11-Detect_YUV420SP/ptq_models/yolo11m_detect_bayese_640x640_nv12_modified.bin')
        self.declare_parameter('classes_num', 80)
        self.declare_parameter('nms_thres', 0.7)
        self.declare_parameter('score_thres', 0.4)
        self.declare_parameter('reg', 16)
        
        # 初始化为None，将在configure阶段创建
        self.yolo_detector = None
        self.image_sub = None
        self.image_pub = None
        self.detections_pub = None  # 新增：检测结果发布者
        self.cv_bridge = None
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """配置阶段：初始化资源，但不启动处理"""
        self.get_logger().info('正在配置YOLOv11检测生命周期节点...')
        
        try:
            # 获取参数
            model_path = self.get_parameter('model_path').get_parameter_value().string_value
            classes_num = self.get_parameter('classes_num').get_parameter_value().integer_value
            nms_thres = self.get_parameter('nms_thres').get_parameter_value().double_value
            score_thres = self.get_parameter('score_thres').get_parameter_value().double_value
            reg = self.get_parameter('reg').get_parameter_value().integer_value
            
            # 初始化YOLOv11检测器
            self.yolo_detector = YOLO11_Detect(
                model_path=model_path,
                classes_num=classes_num,
                nms_thres=nms_thres,
                score_thres=score_thres,
                reg=reg
            )
            
            # 创建生命周期发布者（在激活前不会发布消息）
            self.image_pub = self.create_lifecycle_publisher(
                Image,
                'camera/image_detected',  # 发布处理后的图像话题
                10
            )
            
            # 新增：创建检测结果发布者
            self.detections_pub = self.create_lifecycle_publisher(
                Detection2DArray,
                'camera/detections',  # 发布目标检测结果话题
                10
            )
            
            # 创建CV桥接器
            self.cv_bridge = CvBridge()
            
            self.get_logger().info('YOLOv11检测生命周期节点配置完成')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'配置失败: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """激活阶段：启动处理和通信"""
        self.get_logger().info('激活YOLOv11检测生命周期节点...')
        
        # 创建订阅者（只有在激活状态才订阅，避免处理不需要的消息）
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',  # 订阅摄像头图像话题
            self.image_callback,
            10
        )
        
        # 确保所有发布者处于活动状态
        self.image_pub.on_activate()
        self.detections_pub.on_activate()  # 激活检测结果发布者
        
        self.get_logger().info('YOLOv11检测生命周期节点已激活')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """停用阶段：停止处理，但保留资源"""
        self.get_logger().info('停用YOLOv11检测生命周期节点...')
        
        # 停止订阅消息
        self.destroy_subscription(self.image_sub)
        self.image_sub = None
        
        # 停用所有发布者
        self.image_pub.on_deactivate()
        self.detections_pub.on_deactivate()  # 停用检测结果发布者
        
        self.get_logger().info('YOLOv11检测生命周期节点已停用')
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """清理阶段：释放资源"""
        self.get_logger().info('清理YOLOv11检测生命周期节点资源...')
        
        # 销毁所有发布者
        self.destroy_lifecycle_publisher(self.image_pub)
        self.destroy_lifecycle_publisher(self.detections_pub)  # 销毁检测结果发布者
        self.image_pub = None
        self.detections_pub = None
        
        # 释放模型资源
        self.yolo_detector = None
        self.cv_bridge = None
        
        self.get_logger().info('YOLOv11检测生命周期节点资源已清理')
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """关闭阶段：在任何状态下紧急关闭"""
        self.get_logger().info('关闭YOLOv11检测生命周期节点...')
        
        # 确保资源正确释放
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            
        if self.image_pub is not None:
            self.destroy_lifecycle_publisher(self.image_pub)
            self.image_pub = None
            
        if self.detections_pub is not None:  # 清理检测结果发布者
            self.destroy_lifecycle_publisher(self.detections_pub)
            self.detections_pub = None
            
        self.yolo_detector = None
        self.cv_bridge = None
        
        self.get_logger().info('YOLOv11检测生命周期节点已关闭')
        return TransitionCallbackReturn.SUCCESS
    
    def image_callback(self, msg):
        """处理接收到的图像消息"""
        try:
            # 检查节点是否仍在活动状态
            if not self.image_pub.is_activated:
                self.get_logger().warning('发布者未激活，跳过处理')
                return
                
            # 将ROS图像消息转换为OpenCV格式
            begin_time = time()
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 准备输入数据
            input_tensor = self.yolo_detector.preprocess_yuv420sp(cv_image)
            
            # 执行推理
            outputs = self.yolo_detector.c2numpy(self.yolo_detector.forward(input_tensor))
            
            # 后处理
            results = self.yolo_detector.postProcess(outputs)
            
            # 创建检测结果消息
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # 在图像上绘制检测结果并填充检测消息
            for class_id, score, x1, y1, x2, y2 in results:
                # 绘制检测框
                draw_detection(cv_image, (x1, y1, x2, y2), score, class_id)
                
                # 创建单个检测消息
                detection = Detection2D()
                detection.header = msg.header
                
                # 设置中心点坐标和边界框大小
                detection.bbox.center.x = float((x1 + x2) / 2)
                detection.bbox.center.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                # 设置类别和置信度
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = str(class_id)
                hypothesis.score = float(score)
                hypothesis.pose.pose = Pose2D(x=x1, y=y1, theta=0.0)  # 使用左上角作为pose
                detection.results.append(hypothesis)
                
                # 在检测结果中存储完整坐标 (x1,y1,x2,y2)
                source_img = Pose2D(x=float(x2), y=float(y2), theta=0.0)  # 使用右下角作为source_img
                detection.source_img = source_img
                
                # 添加到检测数组
                detections_msg.detections.append(detection)
            
            # 发布检测结果
            self.detections_pub.publish(detections_msg)
            
            # 转换回ROS图像消息并发布
            result_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            result_msg.header = msg.header  # 保持相同的时间戳和帧ID
            self.image_pub.publish(result_msg)
            
            # 计算并显示总处理时间
            total_time = time() - begin_time
            if len(results) > 0:
                self.get_logger().info(f'检测到 {len(results)} 个目标，总处理时间: {total_time*1000:.2f}ms')
            else:
                self.get_logger().info(f'未检测到目标，总处理时间: {total_time*1000:.2f}ms')
                
        except Exception as e:
            self.get_logger().error(f'处理图像时发生错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # 创建生命周期节点
    lifecycle_node = YoloDetectLifecycleNode()
    
    # 使用多线程执行器，以便在处理回调的同时响应状态转换
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(lifecycle_node)
    
    try:
        # 运行执行器
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源 - 安全地销毁节点和关闭RCL
        try:
            lifecycle_node.destroy_node()
        except Exception as e:
            lifecycle_node.get_logger().warning(f'销毁节点时发生异常: {str(e)}')
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f'关闭RCL时发生异常: {str(e)}')

if __name__ == "__main__":
    main()
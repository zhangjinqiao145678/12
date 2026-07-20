# -*-coding:utf-8 -*-
"""camera_capture_controller.py
Time    :   2025/06/11
Author  :   机械臂控制系统
Version :   1.0
Contact :   aweidw@163.com
License :   (C)Copyright 2024, robottime / robodyno

Summary

  摄像头采集控制器
  - 初始化并启用末端执行器上的摄像头
  - 周期性采集图像数据
  - 通过中间文件将图像数据传输给机械臂控制器进行识别处理
"""

import os
import sys
import json
import time
from datetime import datetime
from controller import Robot


class 摄像头采集控制器:
    """摄像头采集控制器类

    负责管理末端执行器摄像头的初始化、图像采集与中间文件传输。
    """

    def __init__(self):
        """初始化摄像头采集控制器

        - 创建 Robot 实例
        - 获取通信中间文件目录路径
        - 初始化摄像头设备
        """
        self.robot = Robot()
        self.时间步长 = int(self.robot.getBasicTimeStep())

        # 设置通信中间文件目录（与机械臂控制器共享）
        当前文件所在目录 = os.path.dirname(os.path.abspath(__file__))
        上级目录 = os.path.dirname(当前文件所在目录)
        self.通信中间文件目录 = os.path.join(上级目录, "通信中间文件")

        # 确保通信目录存在
        if not os.path.exists(self.通信中间文件目录):
            os.makedirs(self.通信中间文件目录, exist_ok=True)

        # 输出文件路径
        self.图像文件路径 = os.path.join(self.通信中间文件目录, "camera_image.bin")
        self.状态文件路径 = os.path.join(self.通信中间文件目录, "摄像头状态.json")

        # 摄像头设备初始化
        self.摄像头 = None
        self.摄像头名称 = "camera"          # 对应 RobodynoCamera.proto 中的 cameraId 字段
        self.图像宽度 = 0
        self.图像高度 = 0
        self.通道数 = 4                      # Webots 默认返回 BGRA 4通道

        # 记录运行状态
        self.采集帧计数 = 0
        self.最近一次采集时间 = None
    # __init__() 结束

    def 初始化摄像头(self):
        """初始化并启用摄像头设备

        Returns:
            bool: 初始化成功返回 True，失败返回 False
        """
        # 1. 获取摄像头设备句柄
        self.摄像头 = self.robot.getDevice(self.摄像头名称)
        if self.摄像头 is None:
            print(f"[错误] 找不到摄像头设备: {self.摄像头名称}")
            return False

        # 2. 启用摄像头，设置采样周期为 2 个时间步长（约 48ms，即 ~20fps）
        采样周期 = self.时间步长 * 2
        self.摄像头.enable(采样周期)

        # 3. 获取图像参数
        self.图像宽度 = self.摄像头.getWidth()
        self.图像高度 = self.摄像头.getHeight()

        print(f"[信息] 摄像头初始化成功: {self.图像宽度}x{self.图像高度}")
        return True
    # 初始化摄像头() 结束

    def 采集图像(self):
        """采集一帧图像并写入中间文件

        Returns:
            bool: 采集并写入成功返回 True，否则返回 False
        """
        if self.摄像头 is None:
            return False

        # 1. 从摄像头读取原始字节数据（Webots 返回 BGRA 格式）
        图像字节数据 = self.摄像头.getImage()
        if 图像字节数据 is None or len(图像字节数据) == 0:
            return False

        # 2. 写入图像二进制文件（覆盖写入）
        try:
            with open(self.图像文件路径, "wb") as f:
                f.write(图像字节数据)
        except Exception as e:
            print(f"[错误] 写入图像文件失败: {e}")
            return False

        # 3. 更新状态信息 JSON
        self.最近一次采集时间 = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
        self.采集帧计数 += 1

        状态数据 = {
            "camera_name": self.摄像头名称,
            "width": self.图像宽度,
            "height": self.图像高度,
            "channels": self.通道数,
            "timestamp": self.最近一次采集时间,
            "frame_count": self.采集帧计数,
            "image_path": "camera_image.bin",
            "bytes_length": len(图像字节数据)
        }

        try:
            # 先写入临时文件，避免读取时读到不完整的 JSON
            临时文件 = self.状态文件路径 + ".tmp"
            with open(临时文件, "w", encoding="utf-8") as f:
                json.dump(状态数据, f, ensure_ascii=False, indent=2)
            os.replace(临时文件, self.状态文件路径)
        except Exception as e:
            print(f"[错误] 写入状态文件失败: {e}")
            return False

        return True
    # 采集图像() 结束

    def 运行(self):
        """主运行循环

        - 执行一步以等待摄像头稳定
        - 在主循环中周期性采集图像并写入中间文件
        """
        print("[信息] 摄像头采集控制器启动...")
        print(f"[信息] 通信中间文件目录: {self.通信中间文件目录}")

        if not self.初始化摄像头():
            print("[错误] 摄像头初始化失败，控制器退出")
            return

        # 先执行一步，让摄像头开始采集
        if self.robot.step(self.时间步长) == -1:
            return

        print("[信息] 开始图像采集循环")

        # 采集帧间隔计数器：每 2 步采集一帧（约 20fps）
        步数计数器 = 0
        采集间隔步数 = 2

        while self.robot.step(self.时间步长) != -1:
            步数计数器 += 1
            if 步数计数器 >= 采集间隔步数:
                步数计数器 = 0
                成功 = self.采集图像()
                if 成功 and (self.采集帧计数 % 100 == 0):
                    print(f"[信息] 已采集 {self.采集帧计数} 帧，最新时间: {self.最近一次采集时间}")
    # 运行() 结束


def main():
    """主函数

    创建摄像头采集控制器实例并启动运行循环。
    """
    控制器 = 摄像头采集控制器()
    控制器.运行()


if __name__ == "__main__":
    main()
# main() 结束

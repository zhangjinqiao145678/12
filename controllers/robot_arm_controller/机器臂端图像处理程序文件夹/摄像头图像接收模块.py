# -*-coding:utf-8 -*-
"""摄像头图像接收模块.py
Time    :   2025/06/11
Author  :   机械臂控制系统
Version :   1.0
Contact :   aweidw@163.com
License :   (C)Copyright 2024, robottime / robodyno

Summary

  机械臂端摄像头图像接收模块
  - 从通信中间文件读取摄像头端采集的图像数据
  - 将 Webots 原始 BGRA 字节数据转换为可处理的图像格式
  - 提供统一的图像读取接口供上层识别处理程序调用
"""

import os
import json
import time
import threading
from typing import Optional, Tuple, Dict, Any

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


class 摄像头图像接收接口:
    """机械臂端摄像头图像接收接口类

    负责从通信中间文件中读取摄像头端发布的图像数据与状态信息，
    并将 Webots 返回的 BGRA 原始字节数据转换为常用的图像格式。

    通信协议（与 camera_capture_controller 保持一致）：
        - 图像文件：通信中间文件/camera_image.bin（BGRA原始字节）
        - 状态文件：通信中间文件/摄像头状态.json（宽/高/通道数/时间戳）
    """

    def __init__(self, 通信目录: Optional[str] = None):
        """初始化摄像头图像接收接口

        Args:
            通信目录: 通信中间文件所在目录，默认自动推断为 controllers/通信中间文件
        """
        # 自动推断通信中间文件目录
        if 通信目录 is None:
            当前文件所在目录 = os.path.dirname(os.path.abspath(__file__))
            机器臂控制器目录 = os.path.dirname(当前文件所在目录)
            控制器根目录 = os.path.dirname(机器臂控制器目录)
            通信目录 = os.path.join(控制器根目录, "通信中间文件")

        self.通信中间文件目录 = 通信目录
        self.图像文件路径 = os.path.join(self.通信中间文件目录, "camera_image.bin")
        self.状态文件路径 = os.path.join(self.通信中间文件目录, "摄像头状态.json")

        # 记录上一次读取的时间戳，用于判断是否有新图像
        self._上一次时间戳 = None
        self._文件读取锁 = threading.Lock()
    # __init__() 结束

    def 检查通信目录是否存在(self) -> bool:
        """检查通信中间文件目录是否存在

        Returns:
            bool: 目录存在返回 True，否则返回 False
        """
        return os.path.exists(self.通信中间文件目录)
    # 检查通信目录是否存在() 结束

    def 检查摄像头状态文件是否存在(self) -> bool:
        """检查摄像头状态文件是否存在（判断摄像头端是否已启动）

        Returns:
            bool: 状态文件存在返回 True，否则返回 False
        """
        return os.path.exists(self.状态文件路径)
    # 检查摄像头状态文件是否存在() 结束

    def 读取摄像头状态(self) -> Optional[Dict[str, Any]]:
        """读取摄像头状态信息（JSON）

        Returns:
            dict: 状态信息字典，包含 width/height/channels/timestamp 等字段
            None: 读取失败时返回 None
        """
        if not os.path.exists(self.状态文件路径):
            return None
        try:
            with self._文件读取锁:
                with open(self.状态文件路径, "r", encoding="utf-8") as f:
                    return json.load(f)
        except Exception as e:
            print(f"[警告] 读取摄像头状态失败: {e}")
            return None
    # 读取摄像头状态() 结束

    def 读取原始图像字节(self) -> Optional[bytes]:
        """读取原始图像字节数据（BGRA 格式）

        Returns:
            bytes: 图像原始字节数据，失败返回 None
        """
        if not os.path.exists(self.图像文件路径):
            return None
        try:
            with self._文件读取锁:
                with open(self.图像文件路径, "rb") as f:
                    return f.read()
        except Exception as e:
            print(f"[警告] 读取图像文件失败: {e}")
            return None
    # 读取原始图像字节() 结束

    def 读取图像(self, 转换为BGR: bool = True) -> Optional[Tuple[Any, int, int]]:
        """读取当前最新的摄像头图像

        完整流程：读取状态 -> 读取原始字节 -> 重组为图像数组

        Args:
            转换为BGR: 是否将 BGRA 转换为 BGR（丢弃 Alpha 通道），默认 True。
                      False 时保持 BGRA 4通道

        Returns:
            tuple: (图像数组, 宽度, 高度) —— 图像数组为 numpy 数组。
                   读取失败返回 None
        """
        if not _HAS_NUMPY:
            print("[错误] 未安装 numpy，无法进行图像转换。请先安装 numpy。")
            return None

        # 1. 读取状态，获取宽/高/通道信息
        状态 = self.读取摄像头状态()
        if 状态 is None:
            return None

        宽度 = 状态.get("width", 0)
        高度 = 状态.get("height", 0)
        通道数 = 状态.get("channels", 4)

        if 宽度 <= 0 or 高度 <= 0:
            return None

        # 2. 读取原始字节数据
        原始字节 = self.读取原始图像字节()
        if 原始字节 is None or len(原始字节) == 0:
            return None

        # 3. 将 BGRA 字节重组为 numpy 数组
        期望字节数 = 宽度 * 高度 * 通道数
        if len(原始字节) < 期望字节数:
            print(f"[警告] 图像数据不完整: 期望 {期望字节数} 字节，实际 {len(原始字节)} 字节")
            return None

        try:
            # Webots getImage() 返回的是 BGRA 顺序的平面字节数据
            原始数组 = np.frombuffer(原始字节[:期望字节数], dtype=np.uint8)
            图像数组 = 原始数组.reshape((高度, 宽度, 通道数))

            if 转换为BGR and 通道数 == 4:
                # BGRA -> BGR（丢弃 Alpha 通道）
                图像数组 = 图像数组[:, :, 0:3]

            return (图像数组, 宽度, 高度)
        except Exception as e:
            print(f"[警告] 图像数据解析失败: {e}")
            return None
    # 读取图像() 结束

    def 是否有新图像(self) -> bool:
        """检查自上一次读取以来是否有新的图像

        Returns:
            bool: 有新图像返回 True，否则返回 False
        """
        状态 = self.读取摄像头状态()
        if 状态 is None:
            return False

        当前时间戳 = 状态.get("timestamp", None)
        if 当前时间戳 is None:
            return False

        if self._上一次时间戳 is None:
            return True
        return 当前时间戳 != self._上一次时间戳
    # 是否有新图像() 结束

    def 获取最新图像(self, 转换为BGR: bool = True, 仅当有新图像时: bool = False) -> Optional[Tuple[Any, int, int]]:
        """获取最新摄像头图像（推荐使用的接口）

        Args:
            转换为BGR: 是否转换为 BGR 3通道，默认 True
            仅当有新图像时: 只有存在新图像时才返回数据，否则返回 None，默认 False

        Returns:
            tuple: (图像数组, 宽度, 高度)，失败返回 None
        """
        if 仅当有新图像时 and not self.是否有新图像():
            return None

        结果 = self.读取图像(转换为BGR=转换为BGR)
        if 结果 is not None:
            状态 = self.读取摄像头状态()
            if 状态 is not None:
                self._上一次时间戳 = 状态.get("timestamp", None)
        return 结果
    # 获取最新图像() 结束

    def 获取通信信息摘要(self) -> str:
        """获取当前通信状态的人类可读摘要文本

        Returns:
            str: 状态摘要字符串
        """
        状态 = self.读取摄像头状态()
        if 状态 is None:
            return "通信状态: 未连接 / 状态文件不存在"
        文本 = [
            f"通信目录: {self.通信中间文件目录}",
            f"摄像头: {状态.get('camera_name', 'unknown')}",
            f"分辨率: {状态.get('width', 0)}x{状态.get('height', 0)}",
            f"通道数: {状态.get('channels', 0)}",
            f"采集帧数: {状态.get('frame_count', 0)}",
            f"最新时间戳: {状态.get('timestamp', 'unknown')}",
        ]
        return "\n".join(文本)
    # 获取通信信息摘要() 结束


def 创建摄像头接收接口() -> 摄像头图像接收接口:
    """创建并返回一个摄像头图像接收接口实例（便捷函数）

    Returns:
        摄像头图像接收接口: 已初始化的接收接口实例
    """
    return 摄像头图像接收接口()
# 创建摄像头接收接口() 结束


def main():
    """模块自测函数

    当直接运行此文件时，会启动一个简单的测试循环，
    持续读取并打印图像信息，验证通信是否正常。
    """
    接收接口 = 摄像头图像接收接口()

    print("=" * 60)
    print("摄像头图像接收模块 - 自测模式")
    print("=" * 60)
    print(接收接口.获取通信信息摘要())
    print("=" * 60)

    if not 接收接口.检查通信目录是否存在():
        print(f"[错误] 通信目录不存在: {接收接口.通信中间文件目录}")
        print("请确保 camera_capture_controller 已启动并正在运行")
        return

    print("\n开始循环读取图像（按 Ctrl+C 退出）...\n")

    读取次数 = 0
    try:
        while True:
            结果 = 接收接口.获取最新图像(转换为BGR=True, 仅当有新图像时=True)
            if 结果 is not None:
                图像数组, 宽度, 高度 = 结果
                读取次数 += 1
                print(f"[{读取次数}] 新图像: {宽度}x{高度}, shape={图像数组.shape}, dtype={图像数组.dtype}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\n自测结束。")
    # try-except 结束
# main() 结束


if __name__ == "__main__":
    main()
# main() 结束

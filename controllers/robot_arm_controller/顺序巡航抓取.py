# -*-coding:utf-8 -*-
"""顺序巡航抓取.py
Time    :   2026/05/24
Author  :   机械臂控制系统
Version :   1.0

Summary

  顺序巡航抓取模块
  负责按照预设位置依次巡航，并在最后位置触发自动抓取放置流程
  该模块与GUI解耦，提供独立的功能入口和按钮创建接口
"""

import threading
import time
import os
import sys
from tkinter import ttk, messagebox

from 物料抓取控制 import 物料抓取控制器

# 导入识别模块
try:
    当前文件目录 = os.path.dirname(os.path.abspath(__file__))
    图像处理目录 = os.path.join(当前文件目录, "机器臂端图像处理程序文件夹")
    if 图像处理目录 not in sys.path:
        sys.path.insert(0, 图像处理目录)
    from 识别模块 import 识别函数映射表
    _HAS_识别模块 = True
except Exception:
    _HAS_识别模块 = False
    识别函数映射表 = {}


class 顺序巡航抓取控制器:
    """顺序巡航抓取控制器

    该控制器按照预设的巡航坐标依次移动机械臂，并在最后一个位置完成巡航后
    自动对所有检测到的物料执行抓取放置操作。避免重复抓取相同物料。
    """

    默认等待时间 = 2.0
    巡航位置列表 = [
        {
            "name": "物料识别点1",
            "pose": {
                "x": 0.15,
                "y": -0.11,
                "z": 0.18,
                "rx": -3.1415926535,
                "ry": 0,
                "rz": 0,
            },
            "wait": 默认等待时间,
            "识别函数": 识别函数映射表.get("物料识别点1"),
        },
        {
            "name": "物料识别点2",
            "pose": {
                "x": 0.15,
                "y": 0.03,
                "z": 0.18,
                "rx": -3.1415926535,
                "ry": 0,
                "rz": 0,
            },
            "wait": 默认等待时间,
            "识别函数": 识别函数映射表.get("物料识别点2"),
        },
        {
            "name": "托盘识别点1",
            "pose": {
                "x": 0.18,
                "y": 0.17,
                "z": 0.22,
                "rx": -3.1415926535,
                "ry": 0,
                "rz": 1.5707963267,
            },
            "wait": 默认等待时间,
            "识别函数": 识别函数映射表.get("托盘识别点1"),
        },
        {
            "name": "托盘识别点2",
            "pose": {
                "x": 0.0,
                "y": 0.17,
                "z": 0.22,
                "rx": -3.1415926535,
                "ry": 0,
                "rz": 1.5707963267,
            },
            "wait": 默认等待时间,
            "识别函数": 识别函数映射表.get("托盘识别点2"),
        },
    ]

    def __init__(self, arm, 物料数据, update_status_callback, gui=None):
        """初始化顺序巡航抓取控制器

        Args:
            arm: 机械臂对象
            物料数据: 物料位置数据对象
            update_status_callback: 状态更新回调函数
            gui: ArmGUI 实例，用于获取摄像头图像接收接口和窗口更新接口
        """
        self.arm = arm
        self.物料数据 = 物料数据
        self.update_status_callback = update_status_callback
        self.gui = gui  # 用于获取窗口更新接口
        self.抓取控制器 = 物料抓取控制器(arm, 物料数据)
        self.is_running = False
        self._button = None

    def _update_status(self, text):
        """更新状态回调"""
        if callable(self.update_status_callback):
            self.update_status_callback(text)

    def _移动到位姿(self, pose, duration=2.0):
        """移动机械臂到指定位姿"""
        joint_angles = self.arm.inverse_kinematics(
            pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"]
        )
        self.arm.joint_space_interpolated_motion(joint_angles, duration=duration)

    def _获取唯一物料列表(self):
        """获取不重复的待抓取物料名称列表"""
        all_names = self.物料数据.获取所有物料名称()
        unique_names = []
        seen = set()
        for name in all_names:
            if name not in seen:
                seen.add(name)
                unique_names.append(name)
        return unique_names

    def 执行巡航位置(self):
        """按照预设位置依次巡航，每个识别点：移动到位 -> 等待 -> 执行识别函数

        公开方法，供外部调用执行识别流程
        """
        for index, 点 in enumerate(self.巡航位置列表, start=1):
            self._update_status(f"移动到{点['name']} ({index}/{len(self.巡航位置列表)})")
            self._移动到位姿(点["pose"], duration=2.0)

            # [阶段1] 等待（恢复原本的等待行为）
            self._update_status(f"已到位，等待{点['wait']}秒")
            time.sleep(点["wait"])

            # [阶段2] 调用识别函数
            self._update_status(f"正在识别{点['name']}...")
            识别函数 = 点.get("识别函数")
            if callable(识别函数):
                # 构建窗口更新回调（在主线程中执行）
                def 窗口更新回调(图像数组, 宽度, 高度, 标题):
                    if self.gui is not None:
                        self.gui.线程安全_更新识别窗口(图像数组, 宽度, 高度, 标题)
                # 调用识别函数
                识别函数(
                    arm=self.arm,
                    图像接收接口=self.gui._获取图像接收接口() if self.gui else None,
                    窗口更新回调=窗口更新回调,
                    状态回调=self._update_status
                )
            else:
                # 识别函数不存在时，跳过识别阶段
                self._update_status(f"{点['name']} 无识别函数，跳过")

    def _执行抓取放置流程(self):
        """对所有检测到的物料执行抓取并放置，避免重复执行"""
        物料列表 = self._获取唯一物料列表()
        if not 物料列表:
            self._update_status("未检测到物料，巡航结束")
            return

        for 物料名称 in 物料列表:
            self._update_status(f"抓取并放置: {物料名称}")
            try:
                成功 = self.抓取控制器.执行抓取放置序列(
                    物料名称,
                    延时时间=self.默认等待时间,
                )
                if not 成功:
                    self._update_status(f"{物料名称} 抓取放置失败，跳过")
                else:
                    self._update_status(f"{物料名称} 已完成")
            except Exception as e:
                self._update_status(f"{物料名称} 执行失败: {str(e)}")
            time.sleep(self.默认等待时间)

    def 执行顺序巡航抓取(self):
        """执行顺序巡航抓取主流程"""
        if self.is_running:
            return
        self.is_running = True
        try:
            self._update_status("开始顺序巡航")
            self.执行巡航位置()
            self._update_status("巡航完成，开始自动抓取放置")
            self._执行抓取放置流程()
            self._update_status("顺序巡航抓取已完成")
        except Exception as e:
            self._update_status(f"顺序巡航抓取失败: {str(e)}")
        finally:
            self.is_running = False
            if self._button is not None:
                self._button.after(0, lambda: self._button.config(state="normal"))

    def _on_start(self):
        """按钮回调，启动工作线程"""
        if self.is_running:
            messagebox.showwarning("提示", "顺序巡航抓取已在运行中")
            return

        if self._button is not None:
            self._button.config(state="disabled")

        worker = threading.Thread(target=self.执行顺序巡航抓取, daemon=True)
        worker.start()

    def 创建按钮区域(self, parent):
        """创建顺序巡航抓取按钮区域"""
        frame = ttk.LabelFrame(parent, text="顺序巡航抓取", padding="10")
        btn = ttk.Button(frame, text="开始巡航抓取", command=self._on_start, width=18)
        btn.pack(side="left", padx=5)
        self._button = btn
        return frame


def 创建巡航抓取按钮(parent, arm, 物料数据, update_status_callback, gui=None):
    """创建顺序巡航抓取按钮的便捷函数

    Args:
        parent: Tkinter 父容器
        arm: 机械臂对象
        物料数据: 物料位置数据对象
        update_status_callback: 状态更新回调函数
        gui: ArmGUI 实例，用于获取窗口更新接口
    """
    control = 顺序巡航抓取控制器(arm, 物料数据, update_status_callback, gui=gui)
    frame = control.创建按钮区域(parent)
    return frame

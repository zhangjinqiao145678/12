# -*-coding:utf-8 -*-
"""物料抓取控制.py
Time    :   2025/05/24
Author  :   机器臂控制系统
Version :   1.1

Summary

  物料自动抓取控制模块
  用于读取物料位置数据，控制机械臂移动到物料上方并开启吸盘
  V1.1: 添加物料放置功能
"""
import os
import tkinter as tk
from tkinter import ttk, messagebox
import threading

# 导入姿态变换工具
from 姿态变换工具 import 姿态变换工具

# 导入物料放置控制器
from 物料放置控制 import 物料放置控制器


class 物料抓取控制器:
    """物料抓取控制器类

    用于管理物料数据读取和自动抓取逻辑
    预留扩展接口以便根据物料信息动态计算姿态
    """

    默认上方高度 = -0.015
    默认姿态_rx = -3.14159
    默认姿态_ry = 0
    默认姿态_rz = -1.5708

    def __init__(self, arm, 物料数据):
        """初始化物料抓取控制器

        Args:
            arm: 机械臂对象
            物料数据: 物料位置数据对象
        """
        self.arm = arm
        self.物料数据 = 物料数据
        self.is_moving = False
        # 初始化物料放置控制器
        self.放置控制器 = 物料放置控制器()

    def 计算姿态(self, 物料名称, 使用高级姿态变换=False, 机器人位姿=None):
        """根据物料信息计算姿态

        Args:
            物料名称: 物料的名称
            使用高级姿态变换: 是否使用完整的姿态变换（法线相反）
            机器人位姿: 机器人在世界坐标系的位姿 (translation, rotation)

        Returns:
            tuple: (rx, ry, rz) 姿态角度
        """
        物料世界旋转 = self.物料数据.获取世界旋转(物料名称)
        if 物料世界旋转 is None:
            return self.__class__.默认姿态_rx, self.__class__.默认姿态_ry, self.__class__.默认姿态_rz

        if 使用高级姿态变换 and 机器人位姿 is not None:
            # 使用高级姿态变换，使末端执行器法线与物料法线相反
            姿态 = 姿态变换工具.计算末端执行器姿态(
                [物料世界旋转['rx'], 物料世界旋转['ry'], 
                 物料世界旋转['rz'], 物料世界旋转['angle']],
                机器人位姿
            )
            return 姿态['rx'], 姿态['ry'], 姿态['rz']
        else:
            # 使用简单姿态变换
            rz = self.计算Rz从旋转数据(物料世界旋转)
            return self.__class__.默认姿态_rx, self.__class__.默认姿态_ry, rz

    def 计算目标位姿(self, 物料名称, 上方高度=None, 使用高级姿态变换=False, 机器人位姿=None):
        """根据物料信息计算目标位姿

        Args:
            物料名称: 物料的名称
            上方高度: 物料上方的固定高度（米），默认使用默认值
            使用高级姿态变换: 是否使用完整的姿态变换（法线相反）
            机器人位姿: 机器人在世界坐标系的位姿 (translation, rotation)

        Returns:
            dict: 目标位姿字典，包含x, y, z, rx, ry, rz
        """
        if 上方高度 is None:
            上方高度 = self.__class__.默认上方高度

        物料机器人坐标 = self.物料数据.获取机器人坐标(物料名称)
        if 物料机器人坐标 is None:
            raise ValueError(f"未找到物料 {物料名称} 的机器人坐标系数据")

        rx, ry, rz = self.计算姿态(物料名称, 使用高级姿态变换, 机器人位姿)

        return {
            "x": 物料机器人坐标["x"],
            "y": 物料机器人坐标["y"],
            "z": 上方高度,
            "rx": rx,
            "ry": ry,
            "rz": rz
        }

    def 计算Rz从旋转数据(self, 旋转数据):
        """从旋转数据中计算rz角度

        Args:
            旋转数据: 包含rx, ry, rz, angle的字典

        Returns:
            float: rz角度（弧度）
        """
        rx = 旋转数据.get("rx", 0)
        ry = 旋转数据.get("ry", 0)
        rz = 旋转数据.get("rz", 1)
        angle = 旋转数据.get("angle", 0)

        k = abs(rz)
        if k > 0.9:
            return angle if rz > 0 else -angle

        k_x, k_y, k_z = rx, ry, rz
        kz_dot = k_z
        angle_z = angle * kz_dot
        return angle_z

    def 移动到物料上方(self, 物料名称):
        """移动机械臂到物料上方

        Args:
            物料名称: 物料的名称

        Returns:
            bool: 移动是否成功
        """
        if self.is_moving:
            return False

        self.is_moving = True
        try:
            目标位姿 = self.计算目标位姿(物料名称)
            joint_angles = self.arm.inverse_kinematics(
                目标位姿["x"], 目标位姿["y"], 目标位姿["z"],
                目标位姿["rx"], 目标位姿["ry"], 目标位姿["rz"]
            )
            self.arm.joint_space_interpolated_motion(joint_angles, duration=2)
            return True
        except Exception as e:
            raise e
        finally:
            self.is_moving = False

    def 开启吸盘(self):
        """开启真空吸盘（使用正确的Webots VacuumGripper API）"""
        try:
            # 调用arm的enable_end_effector方法
            self.arm.enable_end_effector()
        except Exception as e:
            # 如果开启失败，记录错误但继续执行
            pass

    def 关闭吸盘(self):
        """关闭真空吸盘"""
        try:
            # 调用arm的disable_end_effector方法
            self.arm.disable_end_effector()
        except Exception as e:
            pass

    def 检测吸盘接触(self):
        """检测吸盘是否接触到物体

        Returns:
            bool: 是否接触到物体
        """
        presence = self.arm.vacuum_gripper.getPresence()
        return presence > 0

    def 移动到指定高度(self, 物料名称, target_z):
        """移动机械臂到物料的指定高度

        Args:
            物料名称: 物料的名称
            target_z: 目标z高度

        Returns:
            bool: 移动是否成功
        """
        目标位姿 = self.计算目标位姿(物料名称, target_z)
        joint_angles = self.arm.inverse_kinematics(
            目标位姿["x"], 目标位姿["y"], 目标位姿["z"],
            目标位姿["rx"], 目标位姿["ry"], 目标位姿["rz"]
        )
        self.arm.joint_space_interpolated_motion(joint_angles, duration=0.5)
        return True

    def 执行抓取序列(self, 物料名称):
        """执行完整的抓取序列：开启吸盘→移动到Z=0.04米→逐步下移→检测接触→抓取

        Args:
            物料名称: 物料的名称

        Returns:
            bool: 抓取序列是否成功
        """
        if self.is_moving:
            return False
        
        self.is_moving = True
        try:
            # 步骤1: 先开启吸盘检测和真空吸力
            self.开启吸盘()
            
            # 步骤2: 移动到Z=0.04米处（绝对坐标）
            初始高度 = 0.04
            self.移动到指定高度(物料名称, 初始高度)
            
            # 步骤3: 逐步向下移动，检测接触
            当前高度 = 初始高度
            最低高度 = 0.01  # 最低Z=0.01米处（绝对坐标）
            步长 = 0.005  # 每次下移0.005米
            剩余高度 = 初始高度 - 最低高度
            最大尝试次数 = int(剩余高度 / 步长)
            
            # 确保至少执行一次
            if 最大尝试次数 < 1:
                最大尝试次数 = 1
            
            for i in range(最大尝试次数):
                # 下移一个步长
                当前高度 -= 步长
                if 当前高度 < 最低高度:
                    当前高度 = 最低高度
                
                # 移动到当前高度
                self.移动到指定高度(物料名称, 当前高度)
                
                # 检测是否接触到物体
                if self.检测吸盘接触():
                    return True  # 抓取成功
                
                # 达到最低高度仍未接触，抓取失败
                if 当前高度 <= 最低高度:
                    return False
            
            return False
        except Exception as e:
            raise e
        finally:
            self.is_moving = False

    def 执行抓取放置序列(self, 物料名称, 延时时间=2.0):
        """执行完整的抓取-放置序列

        Args:
            物料名称: 物料的名称
            延时时间: 到达位置后的等待时间（秒）

        Returns:
            bool: 序列是否成功
        """
        if self.is_moving:
            return False
        
        self.is_moving = True
        try:
            # 步骤1: 执行抓取（吸附成功后才继续）
            # 注意：执行抓取序列内部会管理自己的 is_moving 状态
            抓取成功 = self._执行抓取序列无状态检查(物料名称)
            if not 抓取成功:
                return False
            
            # 步骤2: 抓取成功后，先移动到安全高度（物料位置的xy，z=0.07）
            self.移动到指定高度(物料名称, 0.07)
            
            # 步骤3: 获取物料类型（从物料名称提取，如 "FivePointed1" -> "FivePointed"）
            物料类型 = self._提取物料类型(物料名称)
            
            # 步骤4: 根据物料类型执行放置到对应位置
            放置成功 = self.放置控制器.执行放置(self.arm, 物料类型, 延时时间)
            if not 放置成功:
                return False
            
            return True
        except Exception as e:
            raise e
        finally:
            self.is_moving = False

    def _执行抓取序列无状态检查(self, 物料名称):
        """执行完整的抓取序列（不检查is_moving状态，供内部调用）

        Args:
            物料名称: 物料的名称

        Returns:
            bool: 抓取序列是否成功
        """
        try:
            # 步骤1: 先开启吸盘检测和真空吸力
            self.开启吸盘()
            
            # 步骤2: 移动到Z=0.04米处（绝对坐标）
            初始高度 = 0.04
            self.移动到指定高度(物料名称, 初始高度)
            
            # 步骤3: 逐步向下移动，检测接触
            当前高度 = 初始高度
            最低高度 = 0.01  # 最低Z=0.01米处（绝对坐标）
            步长 = 0.005  # 每次下移0.005米
            剩余高度 = 初始高度 - 最低高度
            最大尝试次数 = int(剩余高度 / 步长)
            
            # 确保至少执行一次
            if 最大尝试次数 < 1:
                最大尝试次数 = 1
            
            for i in range(最大尝试次数):
                # 下移一个步长
                当前高度 -= 步长
                if 当前高度 < 最低高度:
                    当前高度 = 最低高度
                
                # 移动到当前高度
                self.移动到指定高度(物料名称, 当前高度)
                
                # 检测是否接触到物体
                if self.检测吸盘接触():
                    return True  # 抓取成功
                
                # 达到最低高度仍未接触，抓取失败
                if 当前高度 <= 最低高度:
                    return False
            
            return False
        except Exception as e:
            raise e

    def _提取物料类型(self, 物料名称):
        """从物料名称中提取物料类型
        
        支持多种物料名称格式:
        - FivePointed（无后缀）
        - FivePointed1（直接数字后缀）
        - FivePointed(1)（带括号的数字后缀）
        
        Args:
            物料名称: 物料的完整名称
        
        Returns:
            str: 物料类型（不含数字后缀）
        """
        # 处理带括号的格式，如 "FivePointed(1)"
        if '(' in 物料名称 and ')' in 物料名称:
            左括号位置 = 物料名称.find('(')
            return 物料名称[:左括号位置]
        
        # 处理直接数字后缀格式，如 "FivePointed1"
        for i in range(len(物料名称)-1, -1, -1):
            if 物料名称[i].isdigit():
                return 物料名称[:i]
        
        # 如果没有数字后缀，直接返回原名称
        return 物料名称


class 物料选择对话框:
    """物料选择对话框类

    弹出窗口让用户选择要抓取的物料
    """

    def __init__(self, 物料名称列表):
        """初始化物料选择对话框

        Args:
            物料名称列表: 可选的物料名称列表
        """
        self.物料名称列表 = 物料名称列表
        self.选中物料 = None
        self.root = None

    def 显示(self):
        """显示对话框并返回选择的物料名称

        Returns:
            str: 选中的物料名称，若取消返回None
        """
        self.root = tk.Toplevel()
        self.root.title("选择要抓取的物料")
        self.root.geometry("400x300")
        self.root.transient()
        self.root.grab_set()

        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.pack(fill="both", expand=True)

        ttk.Label(main_frame, text="请选择要抓取的物料：",
                  font=("Microsoft YaHei", 11)).pack(pady=(0, 10))

        list_frame = ttk.Frame(main_frame)
        list_frame.pack(fill="both", expand=True, pady=(0, 10))

        canvas = tk.Canvas(list_frame)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        self.content_frame = ttk.Frame(canvas)

        self.content_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self.content_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.selected_var = tk.StringVar(value=self.物料名称列表[0] if self.物料名称列表 else None)

        for 物料名称 in self.物料名称列表:
            rb = ttk.Radiobutton(
                self.content_frame,
                text=物料名称,
                variable=self.selected_var,
                value=物料名称
            )
            rb.pack(anchor="w", pady=2)

        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill="x")
        ttk.Button(btn_frame, text="确认", command=self._on_confirm, width=10).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="取消", command=self._on_cancel, width=10).pack(side="left", padx=5)

        self.root.protocol("WM_DELETE_WINDOW", self._on_cancel)
        self.root.wait_window()

        return self.选中物料

    def _on_confirm(self):
        """确认按钮回调"""
        self.选中物料 = self.selected_var.get()
        self.root.destroy()

    def _on_cancel(self):
        """取消按钮回调"""
        self.选中物料 = None
        self.root.destroy()


class 自动抓取按钮:
    """自动抓取按钮管理类

    在机械臂GUI中添加自动抓取相关按钮
    第一次点击时先执行识别流程，之后直接弹出物料选择窗口
    """

    def __init__(self, parent, arm, 物料数据, update_status_callback, gui=None):
        """初始化自动抓取按钮

        Args:
            parent: 父容器
            arm: 机械臂对象
            物料数据: 物料位置数据对象
            update_status_callback: 更新状态栏的回调函数
            gui: ArmGUI实例，用于获取摄像头图像接收接口和窗口更新接口
        """
        self.parent = parent
        self.arm = arm
        self.物料数据 = 物料数据
        self.update_status_callback = update_status_callback
        self.gui = gui
        self.抓取控制器 = 物料抓取控制器(arm, 物料数据)
        self.frame = None
        self._是否已执行过识别 = False
        from 顺序巡航抓取 import 顺序巡航抓取控制器
        self._识别控制器 = 顺序巡航抓取控制器(arm, 物料数据, update_status_callback, gui=gui)

    def 创建按钮区域(self):
        """创建按钮区域"""
        self.frame = ttk.LabelFrame(self.parent, text="自动抓取", padding="10")

        self._btn_auto_grab = ttk.Button(
            self.frame,
            text="自动抓取",
            command=self._on_auto_grab,
            width=15
        )
        self._btn_auto_grab.pack(side="left", padx=5)

        self._btn_grab_place = ttk.Button(
            self.frame,
            text="抓取并放置",
            command=self._on_grab_place,
            width=15
        )
        self._btn_grab_place.pack(side="left", padx=5)

        return self.frame

    def _禁用按钮(self):
        """禁用自动抓取和抓取并放置按钮"""
        if self._btn_auto_grab:
            self._btn_auto_grab.config(state="disabled")
        if self._btn_grab_place:
            self._btn_grab_place.config(state="disabled")

    def _启用按钮(self):
        """启用自动抓取和抓取并放置按钮"""
        if self._btn_auto_grab:
            self._btn_auto_grab.config(state="normal")
        if self._btn_grab_place:
            self._btn_grab_place.config(state="normal")

    def _on_auto_grab(self):
        """自动抓取按钮回调

        第一次点击时先执行识别流程，之后直接弹出物料选择窗口
        """
        if not self._是否已执行过识别:
            self._执行识别后弹出窗口(self._执行自动抓取)
        else:
            self._显示物料选择窗口(self._执行自动抓取)

    def _on_grab_place(self):
        """抓取并放置按钮回调

        第一次点击时先执行识别流程，之后直接弹出物料选择窗口
        """
        if not self._是否已执行过识别:
            self._执行识别后弹出窗口(self._执行抓取放置)
        else:
            self._显示物料选择窗口(self._执行抓取放置)

    def _执行识别后弹出窗口(self, 后续操作回调):
        """执行识别流程后弹出物料选择窗口

        Args:
            后续操作回调: 选择物料后要执行的操作（自动抓取或抓取放置）
        """
        self._禁用按钮()

        def 识别工作线程():
            try:
                self.update_status_callback("执行识别流程...")
                self._识别控制器.执行巡航位置()
                self._是否已执行过识别 = True
                self.update_status_callback("识别完成")
            except Exception as e:
                self.update_status_callback(f"识别失败: {str(e)}")
            finally:
                self.parent.after(0, lambda: self._启用按钮())
                self.parent.after(0, lambda: self._显示物料选择窗口(后续操作回调))

        worker = threading.Thread(target=识别工作线程, daemon=True)
        worker.start()

    def _显示物料选择窗口(self, 后续操作回调):
        """显示物料选择窗口

        Args:
            后续操作回调: 选择物料后要执行的操作
        """
        物料列表 = self.物料数据.获取所有物料名称()
        if not 物料列表:
            self.update_status_callback("未检测到物料")
            return

        选择对话框 = 物料选择对话框(物料列表)
        选中物料 = 选择对话框.显示()

        if 选中物料 is None:
            return

        后续操作回调(选中物料)

    def _执行自动抓取(self, 物料名称):
        """执行自动抓取操作

        Args:
            物料名称: 要抓取的物料名称
        """
        self.update_status_callback("抓取中...")
        try:
            成功 = self.抓取控制器.执行抓取序列(物料名称)
            if 成功:
                self.update_status_callback(f"抓取成功: {物料名称}")
            else:
                self.update_status_callback("抓取失败: 未接触到物料")
        except Exception as e:
            self.update_status_callback(f"抓取失败: {str(e)}")

    def _执行抓取放置(self, 物料名称):
        """执行抓取并放置操作

        Args:
            物料名称: 要抓取的物料名称
        """
        self.update_status_callback("抓取中...")
        try:
            成功 = self.抓取控制器.执行抓取放置序列(物料名称, 延时时间=2.0)
            if 成功:
                self.update_status_callback(f"抓取并放置成功: {物料名称}")
            else:
                self.update_status_callback("抓取或放置失败")
        except Exception as e:
            self.update_status_callback(f"抓取放置失败: {str(e)}")


def 创建自动抓取按钮(parent, arm, 物料数据, update_status_callback, gui=None):
    """创建自动抓取按钮的便捷函数

    Args:
        parent: 父容器
        arm: 机械臂对象
        物料数据: 物料位置数据对象
        update_status_callback: 更新状态栏的回调函数
        gui: ArmGUI实例，用于获取摄像头图像接收接口和窗口更新接口

    Returns:
        ttk.LabelFrame: 创建的按钮区域框架
    """
    按钮管理 = 自动抓取按钮(parent, arm, 物料数据, update_status_callback, gui=gui)
    return 按钮管理.创建按钮区域()
# 物料抓取控制.py 结束

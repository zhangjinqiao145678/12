# -*-coding:utf-8 -*-
"""rpbot.py
Time    :   2026/07/06
Author  :   Trae AI
Version :   1.0
Contact :   trae@trae.cn
License :   (C)Copyright 2026

Summary

  可自由加装的机械臂控制器
  支持在仿真世界中自由搭建和控制机械臂
  通过GUI窗口输入位姿控制机械臂运动
"""
from math import pi
from robodyno.components import Motor
from robodyno.interfaces import Webots
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
import os
import sys

webots = Webots()
webots.sleep(1)


class FlexibleArmController(SixDoFCollabRobot):
    """可自由加装的六自由度协作机械臂控制器类"""

    def __init__(self):
        """初始化机械臂各关节电机和末端执行器"""
        M1 = Motor(webots, 0x20)
        M2 = Motor(webots, 0x21)
        M3 = Motor(webots, 0x22)
        M4 = Motor(webots, 0x23)
        M5 = Motor(webots, 0x24)
        M6 = Motor(webots, 0x25)
        self.webots = webots
        self.vacuum_gripper = None
        
        try:
            self.vacuum_gripper = webots.robot.getDevice("0x26")
            print("检测到真空夹爪设备")
        except Exception as e:
            print(f"未检测到真空夹爪设备: {e}")
            print("可通过 endChildren 在机械臂末端挂载真空夹爪")
        
        tcp_length = 0.045
        super().__init__(M1, M2, M3, M4, M5, M6, 0.065, 0.150, 0.150, 0.08, 0.075, 0.045 + tcp_length)

    def move_to_pose(self, x, y, z, rx, ry, rz, duration=2):
        """移动到指定位姿

        Args:
            x, y, z: 目标位置坐标 (米)
            rx, ry, rz: 目标姿态角度 (弧度)
            duration: 运动时间 (秒)
        """
        print(f"移动到位姿: ({x:.3f}, {y:.3f}, {z:.3f}), 姿态: ({rx:.3f}, {ry:.3f}, {rz:.3f}), 耗时: {duration}s")
        joint_angles = self.inverse_kinematics(x, y, z, rx, ry, rz)
        self.joint_space_interpolated_motion(joint_angles, duration=duration)

    def move_home(self):
        """回到初始位置"""
        print("返回初始位置")
        self.go_home()

    def enable_end_effector(self):
        """使能末端执行器（真空夹爪）- 开启真空吸力"""
        if self.vacuum_gripper is None:
            print("警告: 未挂载真空夹爪")
            return
        
        self.vacuum_gripper.enablePresence(webots.time_step)
        if hasattr(self.vacuum_gripper, 'turnOn'):
            self.vacuum_gripper.turnOn()
            print("真空夹爪已开启")
        elif hasattr(self.vacuum_gripper, 'setTensileStrength'):
            self.vacuum_gripper.setTensileStrength(10.0)
            print("真空夹爪已开启")
        elif hasattr(self.vacuum_gripper, 'setVacuum'):
            self.vacuum_gripper.setVacuum(1.0)
            print("真空夹爪已开启")

    def disable_end_effector(self):
        """关闭末端执行器 - 关闭真空吸力"""
        if self.vacuum_gripper is None:
            print("警告: 未挂载真空夹爪")
            return
        
        self.vacuum_gripper.disablePresence()
        if hasattr(self.vacuum_gripper, 'turnOff'):
            self.vacuum_gripper.turnOff()
            print("真空夹爪已关闭")
        elif hasattr(self.vacuum_gripper, 'setTensileStrength'):
            self.vacuum_gripper.setTensileStrength(-1.0)
            print("真空夹爪已关闭")
        elif hasattr(self.vacuum_gripper, 'setVacuum'):
            self.vacuum_gripper.setVacuum(0.0)
            print("真空夹爪已关闭")

    def is_object_grasped(self):
        """检查是否抓取到物体

        Returns:
            bool: 是否抓取到物体
        """
        if self.vacuum_gripper is None:
            return False
        presence = self.vacuum_gripper.getPresence()
        return presence > 0

    def get_current_pose(self):
        """获取当前末端位姿

        Returns:
            tuple: (x, y, z, rx, ry, rz) 当前位姿
        """
        return self.forward_kinematics()


def print_help():
    """打印帮助信息"""
    print("\n=== FlexibleArm 控制器 ===")
    print("命令列表:")
    print("  home          - 返回初始位置")
    print("  move x y z    - 移动到指定位置")
    print("  pose x y z rx ry rz - 移动到指定位姿")
    print("  grip on/off   - 开启/关闭真空夹爪")
    print("  status        - 显示当前状态")
    print("  quit          - 退出控制器")
    print("  help          - 显示帮助信息")
    print("=========================\n")


def main():
    """主函数"""
    print("=== FlexibleArm 控制器启动 ===")
    print("正在初始化机械臂...")
    
    arm = FlexibleArmController()
    arm.init()
    arm.enable()
    
    print("机械臂初始化完成")
    print("电机ID: 0x20, 0x21, 0x22, 0x23, 0x24, 0x25")
    
    print_help()
    
    while webots.sleep(0.032) != -1:
        pass
    
    print("控制器退出")
    arm.disable()


if __name__ == "__main__":
    main()

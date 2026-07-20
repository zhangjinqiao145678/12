# -*-coding:utf-8 -*-
"""robot_arm_controller.py
Time    :   2025/03/24
Author  :   ryan.zhang
Version :   3.0
Contact :   aweidw@163.com
License :   (C)Copyright 2024, robottime / robodyno

Summary

  机械臂控制器
  通过GUI窗口输入位姿控制机械臂运动
  V3.0: 添加了物料自动抓取功能
"""
from math import pi
from robodyno.components import Motor
from robodyno.interfaces import Webots
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
from 机器臂控制窗口 import start_gui
import os
import sys

通信中间文件目录 = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '通信中间文件')
sys.path.insert(0, os.path.normpath(通信中间文件目录))
from 物料位置数据 import 物料位置数据


webots = Webots()
webots.sleep(1)


class MySixDoFArm(SixDoFCollabRobot):
    """六自由度协作机械臂类"""

    def __init__(self):
        """初始化机械臂各关节电机和末端执行器"""
        M1 = Motor(webots, 0x10)
        M2 = Motor(webots, 0x11)
        M3 = Motor(webots, 0x12)
        M4 = Motor(webots, 0x13)
        M5 = Motor(webots, 0x14)
        M6 = Motor(webots, 0x15)
        self.vacuum_gripper = webots.robot.getDevice("0x21")
        self.webots = webots
        tcp_length = 0.045
        super().__init__(M1, M2, M3, M4, M5, M6, 0.065, 0.150, 0.150, 0.08, 0.075, 0.045 + tcp_length)

    def move_to_pose(self, x, y, z, rx, ry, rz, duration=2):
        """移动到指定位姿

        Args:
            x, y, z: 目标位置坐标 (米)
            rx, ry, rz: 目标姿态角度 (弧度)
            duration: 运动时间 (秒)
        """
        joint_angles = self.inverse_kinematics(x, y, z, rx, ry, rz)
        self.joint_space_interpolated_motion(joint_angles, duration=duration)

    def enable_end_effector(self):
        """使能末端执行器（真空夹爪）- 开启真空吸力"""
        self.vacuum_gripper.enablePresence(webots.time_step)
        # 使用turnOn()方法开启吸盘（参考参考代码）
        if hasattr(self.vacuum_gripper, 'turnOn'):
            self.vacuum_gripper.turnOn()
        elif hasattr(self.vacuum_gripper, 'setTensileStrength'):
            self.vacuum_gripper.setTensileStrength(10.0)
        elif hasattr(self.vacuum_gripper, 'setVacuum'):
            self.vacuum_gripper.setVacuum(1.0)

    def disable_end_effector(self):
        """关闭末端执行器 - 关闭真空吸力"""
        self.vacuum_gripper.disablePresence()
        # 使用turnOff()方法关闭吸盘（参考参考代码）
        if hasattr(self.vacuum_gripper, 'turnOff'):
            self.vacuum_gripper.turnOff()
        elif hasattr(self.vacuum_gripper, 'setTensileStrength'):
            self.vacuum_gripper.setTensileStrength(-1.0)
        elif hasattr(self.vacuum_gripper, 'setVacuum'):
            self.vacuum_gripper.setVacuum(0.0)

    def is_end_effector_enabled(self):
        """检查末端执行器是否已启用

        Returns:
            bool: 是否启用
        """
        return self.vacuum_gripper.getPresence() != -1

    def is_object_grasped(self):
        """检查是否抓取到物体

        Returns:
            bool: 是否抓取到物体
        """
        presence = self.vacuum_gripper.getPresence()
        return presence > 0
# robot_arm_controller.py 结束


def main():
    """主函数"""
    arm = MySixDoFArm()
    arm.init()
    arm.enable()
    arm.enable_end_effector()

    中间文件路径 = os.path.join(通信中间文件目录, '物料位置.json')
    物料数据 = 物料位置数据()
    if os.path.exists(中间文件路径):
        物料数据.从文件导入(中间文件路径)
        print(f"已加载物料位置数据: {中间文件路径}")
        物料列表 = 物料数据.获取所有物料名称()
        print(f"检测到 {len(物料列表)} 个物料: {', '.join(物料列表)}")
    else:
        print(f"警告: 物料位置数据文件不存在: {中间文件路径}")
        print("将无法使用自动抓取功能")

    start_gui(arm, 物料数据=物料数据)
    while webots.sleep(0.032) != -1:
        pass
    arm.disable_end_effector()


if __name__ == "__main__":
    main()
# main() 结束

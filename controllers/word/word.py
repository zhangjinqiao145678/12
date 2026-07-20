# -*-coding:utf-8 -*-
"""word.py
Time    :   2025/05/24
Author  :   Webots World Controller
Version :   3.0

Summary

  世界控制器（后台运行版）
  用于检测仿真世界中物料的位置，将世界坐标转换为机器人坐标系下的位置
  并通过中间文件与机器臂控制器进行数据共享
  无GUI，纯后台运行
  V3.0: 添加了物料旋转信息的检测和存储功能
"""
import os
import sys
from controller import Supervisor
import numpy as np

控制器目录 = os.path.dirname(os.path.abspath(__file__))
通信中间文件目录 = os.path.join(控制器目录, '..', '通信中间文件')
sys.path.insert(0, os.path.normpath(通信中间文件目录))
from 物料位置数据 import 物料位置数据

# 导入物料排序工具
from 物料排序工具 import 物料排序工具


class 坐标转换工具:
    """坐标转换工具类

    提供世界坐标系到机器人坐标系的转换功能
    """

    @staticmethod
    def 构建齐次变换矩阵(translation, rotation):
        """根据Webots的translation和rotation构建齐次变换矩阵

        Args:
            translation: [x, y, z] 平移向量
            rotation: [rx, ry, rz, angle] 旋转轴和角度

        Returns:
            numpy.ndarray: 4x4齐次变换矩阵
        """
        tx, ty, tz = translation
        rx, ry, rz, angle = rotation

        k = np.array([rx, ry, rz])
        k = k / np.linalg.norm(k)

        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c

        nx, ny, nz = k
        m = np.array([
            [t*nx*nx + c,    t*nx*ny - s*nz, t*nx*nz + s*ny, tx],
            [t*nx*ny + s*nz, t*ny*ny + c,    t*ny*nz - s*nx, ty],
            [t*nx*nz - s*ny, t*ny*nz + s*nx, t*nz*nz + c,    tz],
            [0,              0,              0,              1]
        ])
        return m

    @staticmethod
    def 获取逆矩阵(T):
        """获取齐次变换矩阵的逆矩阵

        Args:
            T: 4x4齐次变换矩阵

        Returns:
            numpy.ndarray: 逆矩阵
        """
        R = T[:3, :3]
        p = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ p
        return T_inv

    @classmethod
    def 世界坐标转机器人坐标(cls, world_point, robot_translation, robot_rotation):
        """将世界坐标系下的点转换到机器人坐标系下

        Args:
            world_point: [x, y, z] 世界坐标系下的位置
            robot_translation: [x, y, z] 机器人在世界中的位置
            robot_rotation: [rx, ry, rz, angle] 机器人在世界中的旋转

        Returns:
            numpy.ndarray: [x, y, z] 机器人坐标系下的位置
        """
        T_world_to_robot = cls.获取逆矩阵(
            cls.构建齐次变换矩阵(robot_translation, robot_rotation)
        )
        point_h = np.array([world_point[0], world_point[1], world_point[2], 1])
        robot_point = T_world_to_robot @ point_h
        return robot_point[:3]


class 物料检测器:
    """物料检测器类

    用于检测Webots世界中的物料位置
    """

    物料类型列表 = [
        "Cruciform", "Cuboid", "FivePointed", "Cylindrical", "Cube",
        "Parallelogram", "Triangle", "Pentagonal", "Quincunx"
    ]

    def __init__(self, robot):
        """初始化物料检测器

        Args:
            robot: Webots Supervisor实例
        """
        self.robot = robot
        self.物料节点字典 = {}
        self.机械臂节点 = None

    def 检测所有物料(self):
        """检测世界中所有物料节点

        Returns:
            dict: 物料名称到节点对象的字典
        """
        self.物料节点字典.clear()
        root = self.robot.getRoot()
        children_field = root.getField("children")
        num_children = children_field.getCount()

        for i in range(num_children):
            node = children_field.getMFNode(i)
            node_type = node.getType()
            node_name = node.getTypeName()

            if node_name in self.物料类型列表:
                name_field = node.getField("name")
                if name_field:
                    name = name_field.getSFString()
                else:
                    name = node_name + "_" + str(i)
                self.物料节点字典[name] = node

        return self.物料节点字典

    def 获取机械臂节点(self):
        """获取机械臂节点

        Returns:
            Node: SixDofCollaborationRobot节点，若不存在返回None
        """
        root = self.robot.getRoot()
        children_field = root.getField("children")
        num_children = children_field.getCount()

        for i in range(num_children):
            node = children_field.getMFNode(i)
            if node.getTypeName() == "SixDofCollaborationRobot":
                self.机械臂节点 = node
                return self.机械臂节点
        return None

    def 获取物料世界坐标(self, 物料名称):
        """获取物料在世界坐标系中的位置

        Args:
            物料名称: 物料的名称

        Returns:
            list: [x, y, z] 世界坐标，若不存在返回None
        """
        if 物料名称 not in self.物料节点字典:
            return None
        节点 = self.物料节点字典[物料名称]
        translation_field = 节点.getField("translation")
        return translation_field.getSFVec3f()

    def 获取物料世界旋转(self, 物料名称):
        """获取物料在世界坐标系中的旋转（轴角表示）

        Args:
            物料名称: 物料的名称

        Returns:
            list: [rx, ry, rz, angle] 世界旋转，若不存在返回None
        """
        if 物料名称 not in self.物料节点字典:
            return None
        节点 = self.物料节点字典[物料名称]
        rotation_field = 节点.getField("rotation")
        return rotation_field.getSFRotation()

    def 获取机械臂位姿(self):
        """获取机械臂在世界坐标系中的位姿

        Returns:
            tuple: (translation[3], rotation[4])，若不存在返回(None, None)
        """
        if self.机械臂节点 is None:
            return None, None
        translation_field = self.机械臂节点.getField("translation")
        rotation_field = self.机械臂节点.getField("rotation")
        return translation_field.getSFVec3f(), rotation_field.getSFRotation()

    def 检测并转换所有物料(self):
        """检测所有物料并转换到机器人坐标系

        Returns:
            tuple: (物料位置数据对象, 机械臂位姿)
        """
        物料数据 = 物料位置数据()
        robot_trans, robot_rot = self.获取机械臂位姿()

        if robot_trans is None:
            return 物料数据, (None, None)

        for 物料名称, 节点 in self.物料节点字典.items():
            world_pos = self.获取物料世界坐标(物料名称)
            world_rot = self.获取物料世界旋转(物料名称)
            if world_pos is not None:
                物料数据.设置世界坐标(物料名称, *world_pos)
                if world_rot is not None:
                    物料数据.设置世界旋转(物料名称, *world_rot)
                robot_pos = 坐标转换工具.世界坐标转机器人坐标(
                    world_pos, robot_trans, robot_rot
                )
                物料数据.设置机器人坐标(
                    物料名称,
                    robot_pos[0], robot_pos[1], robot_pos[2],
                    -3.14159, 0, -1.5708
                )

        return 物料数据, (robot_trans, robot_rot)


def 获取中间文件路径():
    """获取中间文件的绝对路径

    Returns:
        str: 中间文件路径
    """
    controller_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(controller_dir, "..", "通信中间文件", "物料位置.json")


def main():
    """主函数

    世界控制器主函数
    用于检测仿真世界中物料的位置，将世界坐标转换为机器人坐标系下的位置
    并通过中间文件与GUI进行数据共享
    无GUI，纯后台运行
    """
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    中间文件路径 = 获取中间文件路径()

    物料检测 = 物料检测器(robot)
    物料检测.获取机械臂节点()
    物料检测.检测所有物料()
    物料数据, 机械臂位姿 = 物料检测.检测并转换所有物料()

    # 按Z轴坐标降序排序物料（高处的物料优先抓取）
    物料数据 = 物料排序工具.按Z轴降序排序(物料数据)

    # print("=" * 50)
    # print("世界控制器启动（后台运行）")
    # print("=" * 50)
    # print("检测到的物料位置信息（机器人坐标系，按Z轴降序排序）：")
    # print("-" * 50)
    # for 物料名称 in 物料数据.获取所有物料名称():
    #     pos = 物料数据.获取机器人坐标(物料名称)
    #     if pos:
    #         print(f"  {物料名称}: X={pos['x']:.4f}, Y={pos['y']:.4f}, Z={pos['z']:.4f}")
    # print("-" * 50)
    # print(f"共检测到 {len(物料数据.获取所有物料名称())} 个物料")
    # print("=" * 50)

    os.makedirs(os.path.dirname(中间文件路径), exist_ok=True)
    try:
        物料数据.导出到文件(中间文件路径)
        # print(f"已将物料位置数据写入: {中间文件路径}")
    except Exception as e:
        # print(f"写入物料位置数据失败: {e}")
        pass

    # print("世界控制器初始化完成，正在进入仿真循环...")
    # print("机器臂控制器窗口现在可以正常显示了")

    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
# word.py 结束

# -*-coding:utf-8 -*-
"""姿态变换工具.py
Time    :   2026/06/07
Author  :   姿态变换模块
Version :   1.0

Summary
  提供完整的姿态变换功能，包括：
  1. 轴角表示与旋转矩阵、四元数、欧拉角之间的转换
  2. 世界坐标系与机器人坐标系之间的姿态变换
  3. 根据物料姿态计算末端执行器法线相反的控制参数
"""
import numpy as np


class 姿态变换工具:
    """姿态变换工具类

    提供多种姿态表示方式之间的转换，以及根据物料姿态计算末端执行器姿态
    """

    @staticmethod
    def 轴角转旋转矩阵(rx, ry, rz, angle):
        """将轴角表示转换为旋转矩阵

        Args:
            rx, ry, rz: 旋转轴方向向量
            angle: 旋转角度（弧度）

        Returns:
            numpy.ndarray: 3x3旋转矩阵
        """
        k = np.array([rx, ry, rz])
        norm = np.linalg.norm(k)
        if norm < 1e-10:
            return np.eye(3)
        k = k / norm

        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        kx, ky, kz = k

        R = np.array([
            [t*kx*kx + c,    t*kx*ky - s*kz, t*kx*kz + s*ky],
            [t*kx*ky + s*kz, t*ky*ky + c,    t*ky*kz - s*kx],
            [t*kx*kz - s*ky, t*ky*kz + s*kx, t*kz*kz + c]
        ])
        return R

    @staticmethod
    def 旋转矩阵转轴角(R):
        """将旋转矩阵转换为轴角表示

        Args:
            R: 3x3旋转矩阵

        Returns:
            tuple: (rx, ry, rz, angle) 旋转轴和角度（弧度）
        """
        trace = np.trace(R)
        angle = np.arccos((trace - 1) / 2)

        if angle < 1e-10:
            return 0, 0, 1, 0

        rx = R[2, 1] - R[1, 2]
        ry = R[0, 2] - R[2, 0]
        rz = R[1, 0] - R[0, 1]
        norm = np.sqrt(rx*rx + ry*ry + rz*rz)

        if norm < 1e-10:
            return 0, 0, 1, 0

        rx /= norm
        ry /= norm
        rz /= norm

        return rx, ry, rz, angle

    @staticmethod
    def 旋转矩阵转四元数(R):
        """将旋转矩阵转换为四元数

        Args:
            R: 3x3旋转矩阵

        Returns:
            numpy.ndarray: [w, x, y, z] 四元数
        """
        q = np.zeros(4)
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1)
            q[0] = 0.25 / s
            q[1] = (R[2, 1] - R[1, 2]) * s
            q[2] = (R[0, 2] - R[2, 0]) * s
            q[3] = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = 0.25 * s
            q[2] = (R[0, 1] + R[1, 0]) / s
            q[3] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
            q[0] = (R[0, 2] - R[2, 0]) / s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = 0.25 * s
            q[3] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
            q[0] = (R[1, 0] - R[0, 1]) / s
            q[1] = (R[0, 2] + R[2, 0]) / s
            q[2] = (R[1, 2] + R[2, 1]) / s
            q[3] = 0.25 * s

        return q

    @staticmethod
    def 四元数转旋转矩阵(q):
        """将四元数转换为旋转矩阵

        Args:
            q: [w, x, y, z] 四元数

        Returns:
            numpy.ndarray: 3x3旋转矩阵
        """
        w, x, y, z = q
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        return R

    @staticmethod
    def 旋转矩阵转欧拉角(R):
        """将旋转矩阵转换为ZYX欧拉角(rx, ry, rz)

        Args:
            R: 3x3旋转矩阵

        Returns:
            tuple: (rx, ry, rz) 欧拉角（弧度）
        """
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            rx = np.arctan2(R[2, 1], R[2, 2])
            ry = np.arctan2(-R[2, 0], sy)
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            rx = np.arctan2(-R[1, 2], R[1, 1])
            ry = np.arctan2(-R[2, 0], sy)
            rz = 0

        return rx, ry, rz

    @staticmethod
    def 欧拉角转旋转矩阵(rx, ry, rz):
        """将ZYX欧拉角转换为旋转矩阵

        Args:
            rx, ry, rz: 欧拉角（弧度）

        Returns:
            numpy.ndarray: 3x3旋转矩阵
        """
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        R_y = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        R_z = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        R = R_z @ R_y @ R_x
        return R

    @classmethod
    def 构建齐次变换矩阵(cls, translation, rotation):
        """根据平移和旋转（轴角）构建齐次变换矩阵

        Args:
            translation: [x, y, z] 平移向量
            rotation: [rx, ry, rz, angle] 旋转轴和角度

        Returns:
            numpy.ndarray: 4x4齐次变换矩阵
        """
        tx, ty, tz = translation
        rx, ry, rz, angle = rotation
        R = cls.轴角转旋转矩阵(rx, ry, rz, angle)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        return T

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
    def 计算末端执行器姿态(cls, 物料旋转轴角, 机器人位姿):
        """根据物料姿态计算末端执行器姿态，使法线与物料法线相反

        Args:
            物料旋转轴角: [rx, ry, rz, angle] 物料在世界坐标系中的旋转（轴角）
            机器人位姿: tuple (translation[3], rotation[4]) 机器人在世界坐标系中的位姿

        Returns:
            dict: 机器人坐标系下的姿态 {'rx': rx, 'ry': ry, 'rz': rz}，单位弧度
        """
        # 1. 物料旋转矩阵（世界坐标系）
        R_material_world = cls.轴角转旋转矩阵(*物料旋转轴角)

        # 2. 物料的法线（自身坐标系+Z轴），在世界坐标系下的方向
        material_normal_world = R_material_world[:, 2]

        # 3. 末端执行器的法线需要与物料法线相反
        end_effector_normal_world = -material_normal_world

        # 4. 构建末端执行器的旋转矩阵（世界坐标系）
        # 我们需要找到旋转矩阵 R_end，使得 R_end[:, 2] = end_effector_normal_world
        # 同时保持Y轴尽可能接近原始方向

        # 目标Z轴
        z_axis = end_effector_normal_world

        # 选择初始Y轴（尽可能接近垂直向上）
        y_axis_candidate = np.array([0, 1, 0])
        if np.abs(np.dot(z_axis, y_axis_candidate)) > 0.9:
            y_axis_candidate = np.array([1, 0, 0])

        # Gram-Schmidt正交化
        x_axis = np.cross(y_axis_candidate, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        R_end_world = np.column_stack([x_axis, y_axis, z_axis])

        # 5. 获取机器人在世界坐标系下的变换矩阵
        robot_trans, robot_rot = 机器人位姿
        T_robot_world = cls.构建齐次变换矩阵(robot_trans, robot_rot)
        T_world_robot = cls.获取逆矩阵(T_robot_world)

        # 6. 将末端执行器的旋转矩阵转换到机器人坐标系
        R_robot_world = T_robot_world[:3, :3]
        R_end_robot = R_robot_world.T @ R_end_world

        # 7. 将旋转矩阵转换为欧拉角
        rx, ry, rz = cls.旋转矩阵转欧拉角(R_end_robot)

        return {'rx': rx, 'ry': ry, 'rz': rz}

    @classmethod
    def 计算简单抓取姿态(cls, 物料旋转轴角):
        """简化版：仅根据物料旋转计算合适的rz角度，保持rx=-π, ry=0

        适用于物料主要在XY平面旋转的情况

        Args:
            物料旋转轴角: [rx, ry, rz, angle] 物料在世界坐标系中的旋转（轴角）

        Returns:
            dict: 机器人坐标系下的姿态 {'rx': rx, 'ry': ry, 'rz': rz}，单位弧度
        """
        rx_m, ry_m, rz_m, angle_m = 物料旋转轴角

        # 检查是否主要绕Z轴旋转
        if abs(rz_m) > 0.9:
            # 绕Z轴旋转，直接使用该角度
            rz = angle_m if rz_m > 0 else -angle_m
        else:
            # 否则计算投影到Z轴的旋转分量
            rz = angle_m * rz_m

        # 保持标准抓取姿态：rx=-π, ry=0
        return {
            'rx': -3.141592653589793,
            'ry': 0,
            'rz': rz - 1.5708
        }


# 姿态变换工具.py 结束

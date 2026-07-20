# -*-coding:utf-8 -*-
"""物料位置数据.py
Time    :   2025/05/24
Author  :   通信中间件
Version :   2.0

Summary

  物料位置数据结构定义
  定义物料在世界坐标系和机器人坐标系下的位置和旋转数据结构
  V2.0: 添加了物料旋转信息的存储功能
"""
import json
import os


class 物料位置数据:
    """物料位置数据类

    用于存储和管理物料在世界坐标系和机器人坐标系下的位置信息
    """

    def __init__(self):
        """初始化物料位置数据"""
        self.世界坐标系 = {}
        self.机器人坐标系 = {}

    def 设置世界坐标(self, 物料名称, x, y, z):
        """设置物料在世界坐标系中的位置

        Args:
            物料名称: 物料的名称
            x, y, z: 世界坐标系下的位置
        """
        if 物料名称 not in self.世界坐标系:
            self.世界坐标系[物料名称] = {}
        self.世界坐标系[物料名称]["x"] = x
        self.世界坐标系[物料名称]["y"] = y
        self.世界坐标系[物料名称]["z"] = z

    def 设置世界旋转(self, 物料名称, rx, ry, rz, angle):
        """设置物料在世界坐标系中的旋转（轴角表示）

        Args:
            物料名称: 物料的名称
            rx, ry, rz: 旋转轴方向（归一化向量）
            angle: 旋转角度（弧度）
        """
        if 物料名称 not in self.世界坐标系:
            self.世界坐标系[物料名称] = {}
        self.世界坐标系[物料名称]["rotation_rx"] = rx
        self.世界坐标系[物料名称]["rotation_ry"] = ry
        self.世界坐标系[物料名称]["rotation_rz"] = rz
        self.世界坐标系[物料名称]["rotation_angle"] = angle

    def 获取世界坐标(self, 物料名称):
        """获取物料在世界坐标系中的位置

        Args:
            物料名称: 物料的名称

        Returns:
            dict: 包含x, y, z的字典，若不存在返回None
        """
        return self.世界坐标系.get(物料名称)

    def 获取世界旋转(self, 物料名称):
        """获取物料在世界坐标系中的旋转信息

        Args:
            物料名称: 物料的名称

        Returns:
            dict: 包含rx, ry, rz, angle的字典，若不存在返回None
        """
        物料数据 = self.世界坐标系.get(物料名称)
        if 物料数据 is None:
            return None
        return {
            "rx": 物料数据.get("rotation_rx"),
            "ry": 物料数据.get("rotation_ry"),
            "rz": 物料数据.get("rotation_rz"),
            "angle": 物料数据.get("rotation_angle")
        }

    def 设置机器人坐标(self, 物料名称, x, y, z, rx, ry, rz):
        """设置物料在机器人坐标系中的位置和姿态

        Args:
            物料名称: 物料的名称
            x, y, z: 机器人坐标系下的位置
            rx, ry, rz: 机器人坐标系下的姿态（弧度）
        """
        self.机器人坐标系[物料名称] = {
            "x": x, "y": y, "z": z,
            "rx": rx, "ry": ry, "rz": rz
        }

    def 获取世界坐标(self, 物料名称):
        """获取物料在世界坐标系中的位置

        Args:
            物料名称: 物料的名称

        Returns:
            dict: 包含x, y, z的字典，若不存在返回None
        """
        return self.世界坐标系.get(物料名称)

    def 获取机器人坐标(self, 物料名称):
        """获取物料在机器人坐标系中的位置和姿态

        Args:
            物料名称: 物料的名称

        Returns:
            dict: 包含x, y, z, rx, ry, rz的字典，若不存在返回None
        """
        return self.机器人坐标系.get(物料名称)

    def 获取所有物料名称(self):
        """获取所有已记录的物料名称列表"""
        return list(self.机器人坐标系.keys())

    def 导出到文件(self, 文件路径):
        """导出数据到JSON文件

        Args:
            文件路径: 保存文件的路径
        """
        数据 = {
            "世界坐标系": self.世界坐标系,
            "机器人坐标系": self.机器人坐标系
        }
        with open(文件路径, 'w', encoding='utf-8') as f:
            json.dump(数据, f, indent=4, ensure_ascii=False)

    def 从文件导入(self, 文件路径):
        """从JSON文件导入数据

        Args:
            文件路径: 数据文件的路径

        Returns:
            bool: 导入是否成功
        """
        if not os.path.exists(文件路径):
            return False
        try:
            with open(文件路径, 'r', encoding='utf-8') as f:
                数据 = json.load(f)
                self.世界坐标系 = 数据.get("世界坐标系", {})
                self.机器人坐标系 = 数据.get("机器人坐标系", {})
            return True
        except Exception:
            return False

    def 清除数据(self):
        """清除所有数据"""
        self.世界坐标系.clear()
        self.机器人坐标系.clear()
# 物料位置数据.py 结束

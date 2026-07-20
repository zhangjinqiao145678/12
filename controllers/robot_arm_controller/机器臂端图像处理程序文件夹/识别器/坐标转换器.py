# -*-coding:utf-8 -*-
"""坐标转换器.py

手眼标定坐标转换模块
用于将视觉识别得到的像素坐标转换为机械臂坐标系下的位置坐标

设计目标：
1. 支持多元线性回归手眼标定（仿射变换）
2. 支持多识别点参数切换
3. 读取视觉识别结果并转换保存
4. 输出格式与通信中间文件/物料位置.json保持一致

转换公式（多元线性回归）：
robot_x = a1 * pixel_x + a2 * pixel_y + b1
robot_y = a3 * pixel_x + a4 * pixel_y + b2

基于实际数据计算的标定参数
"""

import json
import os
import time
import numpy as np
from typing import Dict, Any, Optional, Tuple


class 手眼标定坐标转换器:
    """手眼标定坐标转换器类

    将像素坐标转换为机械臂坐标系下的位置坐标
    支持多元线性回归标定方式，支持多识别点参数切换
    """

    默认标定参数 = {
        'a1': 0.0003315650966074353,
        'a2': -5.850287183493084e-07,
        'b1': -0.09415932178249778,
        'a3': -1.2617274186381701e-06,
        'a4': -0.0003348552056318504,
        'b2': 0.3309616043450774
    }

    def __init__(self, 标定参数: Optional[Dict[str, float]] = None):
        """初始化手眼标定坐标转换器

        Args:
            标定参数: 手眼标定参数字典，包含a1, a2, b1, a3, a4, b2（可选）
                     如果不提供，使用基于实际数据计算的默认标定参数
        """
        if 标定参数 is None:
            self.标定参数 = self.默认标定参数.copy()
        else:
            self.标定参数 = {**self.默认标定参数, **标定参数}
        
        # 加载识别点参数配置
        self.识别点参数配置 = self._加载识别点参数配置()
        # 当前识别点名称
        self.当前识别点名称 = None
        
        print(f"[坐标转换器] 初始化完成，标定参数: {self.标定参数}")

    def _加载识别点参数配置(self) -> Dict[str, Dict[str, float]]:
        """加载识别点参数配置文件

        Returns:
            Dict[str, Dict[str, float]]: 识别点参数配置字典
        """
        配置文件路径 = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            '识别点参数配置.json'
        )
        
        if os.path.exists(配置文件路径):
            try:
                with open(配置文件路径, 'r', encoding='utf-8') as f:
                    配置 = json.load(f)
                print(f"[坐标转换器] 已加载识别点参数配置: {配置文件路径}")
                return 配置.get('识别点配置', {})
            except Exception as e:
                print(f"[坐标转换器] 加载识别点参数配置失败: {e}")
        
        return {}

    def 切换识别点参数(self, 识别点名称: str) -> bool:
        """切换到指定识别点的标定参数

        Args:
            识别点名称: 识别点名称，如"物料识别点1"、"物料识别点2"等

        Returns:
            bool: 是否成功切换（如果识别点没有配置参数，则使用默认参数）
        """
        self.当前识别点名称 = 识别点名称
        
        # 查找该识别点的配置参数
        识别点配置 = self.识别点参数配置.get(识别点名称, {})
        
        if 识别点配置 and len(识别点配置) >= 6:
            # 使用识别点特定参数
            self.标定参数 = {
                'a1': 识别点配置['a1'],
                'a2': 识别点配置['a2'],
                'b1': 识别点配置['b1'],
                'a3': 识别点配置['a3'],
                'a4': 识别点配置['a4'],
                'b2': 识别点配置['b2']
            }
            print(f"[坐标转换器] 已切换到识别点 '{识别点名称}' 的参数")
            return True
        else:
            # 使用默认参数
            self.标定参数 = self.默认标定参数.copy()
            print(f"[坐标转换器] 识别点 '{识别点名称}' 无配置参数，使用默认参数")
            return False

    def 设置标定参数(self, a1: float, a2: float, b1: float, a3: float, a4: float, b2: float):
        """设置手眼标定参数（多元线性回归）

        Args:
            a1: pixel_x对robot_x的系数
            a2: pixel_y对robot_x的系数
            b1: robot_x偏移量
            a3: pixel_x对robot_y的系数
            a4: pixel_y对robot_y的系数
            b2: robot_y偏移量
        """
        self.标定参数['a1'] = a1
        self.标定参数['a2'] = a2
        self.标定参数['b1'] = b1
        self.标定参数['a3'] = a3
        self.标定参数['a4'] = a4
        self.标定参数['b2'] = b2
        print(f"[坐标转换器] 标定参数已更新: a1={a1}, a2={a2}, b1={b1}, a3={a3}, a4={a4}, b2={b2}")

    def 保存标定参数(self, 文件路径: str) -> bool:
        """保存标定参数到文件

        Args:
            文件路径: 保存标定参数的文件路径

        Returns:
            bool: 是否保存成功
        """
        try:
            目录 = os.path.dirname(文件路径)
            if 目录 and not os.path.exists(目录):
                os.makedirs(目录, exist_ok=True)
            
            with open(文件路径, 'w', encoding='utf-8') as f:
                json.dump(self.标定参数, f, indent=4, ensure_ascii=False)
            
            print(f"[坐标转换器] 标定参数已保存到: {文件路径}")
            return True
        except Exception as e:
            print(f"[坐标转换器] 保存标定参数失败: {e}")
            return False

    def 加载标定参数(self, 文件路径: str) -> bool:
        """从文件加载标定参数

        Args:
            文件路径: 标定参数文件路径

        Returns:
            bool: 是否加载成功
        """
        try:
            if not os.path.exists(文件路径):
                print(f"[坐标转换器] 警告: 标定参数文件不存在: {文件路径}")
                return False
            
            with open(文件路径, 'r', encoding='utf-8') as f:
                参数 = json.load(f)
            
            self.标定参数.update(参数)
            print(f"[坐标转换器] 标定参数已从 {文件路径} 加载")
            return True
        except Exception as e:
            print(f"[坐标转换器] 加载标定参数失败: {e}")
            return False

    def 像素坐标转机械臂坐标(self, pixel_x: float, pixel_y: float) -> Tuple[float, float]:
        """将像素坐标转换为机械臂坐标

        使用多元线性回归公式：
        robot_x = a1 * pixel_x + a2 * pixel_y + b1
        robot_y = a3 * pixel_x + a4 * pixel_y + b2

        Args:
            pixel_x: 像素坐标X
            pixel_y: 像素坐标Y

        Returns:
            Tuple[float, float]: 机械臂坐标 (x, y)
        """
        a1 = self.标定参数['a1']
        a2 = self.标定参数['a2']
        b1 = self.标定参数['b1']
        a3 = self.标定参数['a3']
        a4 = self.标定参数['a4']
        b2 = self.标定参数['b2']

        robot_x = a1 * pixel_x + a2 * pixel_y + b1
        robot_y = a3 * pixel_x + a4 * pixel_y + b2

        print(f"[坐标转换器] 像素坐标 ({pixel_x}, {pixel_y}) -> 机械臂坐标 ({robot_x:.6f}, {robot_y:.6f})")
        return (robot_x, robot_y)

    def 批量转换像素坐标(self, 像素坐标列表: list) -> list:
        """批量转换像素坐标列表

        Args:
            像素坐标列表: 包含(pixel_x, pixel_y)的列表

        Returns:
            list: 转换后的机械臂坐标列表
        """
        return [self.像素坐标转机械臂坐标(x, y) for x, y in 像素坐标列表]

    def 转换视觉识别结果(self, 识别结果: Dict[str, Any], 识别点名称: Optional[str] = None, is_tray: bool = False) -> Dict[str, Any]:
        """转换视觉识别结果中的坐标（支持物料和托盘缺口）

        Args:
            识别结果: 视觉识别结果字典，格式参考物料位置.json或托盘缺口位置.json
            识别点名称: 当前识别点名称（可选），用于选择对应的标定参数
            is_tray: 是否为托盘缺口识别（True表示托盘识别，False表示物料识别）

        Returns:
            Dict[str, Any]: 包含转换后坐标的字典，格式与通信中间文件/物料位置.json一致
        """
        # 如果指定了识别点名称，切换到对应的参数
        if 识别点名称:
            self.切换识别点参数(识别点名称)
        
        结果 = {
            '世界坐标系': {},
            '机器人坐标系': {}
        }

        # 根据识别类型选择不同的数据字段
        if is_tray:
            # 托盘缺口识别
            数据详情 = 识别结果.get('缺口详情', {})
            类型标签 = '缺口'
        else:
            # 物料识别
            数据详情 = 识别结果.get('物料详情', {})
            类型标签 = '物料'

        for 名称, 信息 in 数据详情.items():
            pixel_x = 信息.get('像素坐标_x', 0)
            pixel_y = 信息.get('像素坐标_y', 0)
            旋转角度 = 信息.get('旋转角度', 0)
            对象类型 = 信息.get('物料类型') if not is_tray else 信息.get('缺口类型', 名称)

            # 转换像素坐标到机械臂坐标
            hand_x, hand_y = self.像素坐标转机械臂坐标(pixel_x, pixel_y)

            # 计算旋转参数（将角度转换为弧度，构建旋转轴）
            rotation_angle_rad = np.deg2rad(旋转角度)
            rotation_rz = np.sin(rotation_angle_rad / 2.0)
            rotation_angle = 2.0 * np.arcsin(rotation_rz)

            # 构建世界坐标系数据（使用float()转换numpy类型为Python原生类型）
            结果['世界坐标系'][名称] = {
                'x': float(hand_x),
                'y': float(hand_y),
                'z': 0.015,  # 默认Z高度，根据实际情况调整
                'rotation_rx': 0.0,
                'rotation_ry': 0.0,
                'rotation_rz': 1.0,
                'rotation_angle': float(rotation_angle)
            }

            # 构建机器人坐标系数据（使用实际的旋转角度）
            # 将缺口旋转角度转换为机械臂的rz角度
            # 缺口角度是相对于垂直方向的旋转，需要转换为机械臂末端角度
            缺口角度_rad = np.deg2rad(旋转角度)
            机械臂_rz = -1.5708 + 缺口角度_rad  # 基准角度加上缺口旋转角度

            结果['机器人坐标系'][名称] = {
                'x': float(hand_x),
                'y': float(hand_y),
                'z': float(0.016193868669948278),  # 默认Z高度
                'rx': -3.14159,
                'ry': 0,
                'rz': float(机械臂_rz)
            }

        print(f"[坐标转换器] 已转换 {len(数据详情)} 个{类型标签}的坐标")
        return 结果

    def 读取视觉识别结果(self, 文件路径: str) -> Optional[Dict[str, Any]]:
        """读取视觉识别结果文件

        Args:
            文件路径: 视觉识别结果JSON文件路径

        Returns:
            Optional[Dict[str, Any]]: 识别结果字典，失败返回None
        """
        try:
            if not os.path.exists(文件路径):
                print(f"[坐标转换器] 错误: 视觉识别结果文件不存在: {文件路径}")
                return None
            
            with open(文件路径, 'r', encoding='utf-8') as f:
                识别结果 = json.load(f)
            
            print(f"[坐标转换器] 已读取视觉识别结果: {文件路径}")
            return 识别结果
        except Exception as e:
            print(f"[坐标转换器] 读取视觉识别结果失败: {e}")
            return None

    def 保存转换结果(self, 转换结果: Dict[str, Any], 文件路径: str, 识别点名称: Optional[str] = None, is_tray: bool = False) -> bool:
        """保存转换结果到文件

        相同识别点的数据覆盖，不同识别点的数据保留

        Args:
            转换结果: 转换后的坐标数据
            文件路径: 保存路径
            识别点名称: 识别点名称（可选），用于分组存储
            is_tray: 是否为托盘缺口识别（True表示托盘识别，False表示物料识别）

        Returns:
            bool: 是否保存成功
        """
        try:
            目录 = os.path.dirname(文件路径)
            if 目录 and not os.path.exists(目录):
                os.makedirs(目录, exist_ok=True)
            
            # 读取现有数据（如果存在）
            现有数据 = {}
            if os.path.exists(文件路径):
                try:
                    with open(文件路径, 'r', encoding='utf-8') as f:
                        现有数据 = json.load(f)
                except:
                    现有数据 = {}
            
            # 添加识别类型标识
            转换结果['识别类型'] = '托盘缺口' if is_tray else '物料'
            
            # 按识别点名称分组存储
            if 识别点名称:
                现有数据[识别点名称] = 转换结果
                现有数据[识别点名称]['时间戳'] = int(time.time())
            else:
                # 没有识别点名称时，使用默认键
                现有数据['默认'] = 转换结果
                现有数据['默认']['时间戳'] = int(time.time())
            
            with open(文件路径, 'w', encoding='utf-8') as f:
                json.dump(现有数据, f, indent=4, ensure_ascii=False)
            
            类型标签 = '托盘缺口' if is_tray else '物料'
            print(f"[坐标转换器] {类型标签}转换结果已保存到: {文件路径} (识别点: {识别点名称})")
            return True
        except Exception as e:
            print(f"[坐标转换器] 保存转换结果失败: {e}")
            return False

    def 执行完整转换流程(self, 输入文件路径: str, 输出文件路径: str, 识别点名称: Optional[str] = None, is_tray: bool = False) -> bool:
        """执行完整的转换流程

        Args:
            输入文件路径: 视觉识别结果文件路径（物料位置.json）
            输出文件路径: 转换结果保存路径
            识别点名称: 当前识别点名称（可选）
            is_tray: 是否为托盘缺口识别

        Returns:
            bool: 是否成功
        """
        print(f"[坐标转换器] 开始执行转换流程...")
        
        # 步骤1: 读取视觉识别结果
        识别结果 = self.读取视觉识别结果(输入文件路径)
        if 识别结果 is None:
            return False
        
        # 步骤2: 转换坐标（支持识别点参数切换）
        转换结果 = self.转换视觉识别结果(识别结果, 识别点名称, is_tray)
        
        # 步骤3: 保存转换结果（带识别点名称分组）
        成功 = self.保存转换结果(转换结果, 输出文件路径, 识别点名称, is_tray)
        
        if 成功:
            print(f"[坐标转换器] 转换流程执行完成")
        else:
            print(f"[坐标转换器] 转换流程执行失败")
        
        return 成功


# 坐标转换器.py 结束
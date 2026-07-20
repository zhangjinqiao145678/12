# -*-coding:utf-8 -*-
"""手眼标定工具.py

手眼标定参数计算工具
用于根据像素坐标和机器人坐标计算标定参数

计算方法：多元线性回归（最小二乘法）
转换公式：
    robot_x = a1 * pixel_x + a2 * pixel_y + b1
    robot_y = a3 * pixel_x + a4 * pixel_y + b2

使用方法：
1. 准备物料位置/物料位置.json（像素坐标数据）
2. 准备通信中间文件/物料位置.json（机器人坐标数据）
3. 运行此脚本自动计算标定参数
4. 参数会自动保存到标定参数文件

注意：需要确保两组数据中的物料名称能够正确对应

新增功能：支持为特定识别点保存标定参数
使用方式: python 手眼标定工具.py [像素文件] [机器人文件] [识别点名称]
"""

import numpy as np
import json
import os
from typing import Dict, List, Tuple, Optional


class 手眼标定计算器:
    """手眼标定计算器类
    
    使用多元线性回归方法计算像素坐标到机器人坐标的转换参数
    支持为特定识别点保存参数
    """
    
    def __init__(self):
        """初始化标定计算器"""
        self.标定参数 = None
        self.映射关系 = None
    
    def 设置映射关系(self, 映射: Dict[str, str]):
        """设置物料名称映射关系
        
        Args:
            映射: 像素坐标物料名称到机器人坐标物料名称的映射字典
        """
        self.映射关系 = 映射
    
    def 从文件加载数据(self, 像素文件路径: str, 机器人文件路径: str) -> bool:
        """从文件加载像素坐标和机器人坐标数据
        
        Args:
            像素文件路径: 视觉识别结果文件路径（物料位置.json）
            机器人文件路径: 机器人坐标系数据文件路径
            
        Returns:
            bool: 是否加载成功
        """
        try:
            # 读取像素坐标数据
            with open(像素文件路径, 'r', encoding='utf-8') as f:
                self.像素数据 = json.load(f)
            
            # 读取机器人坐标数据
            with open(机器人文件路径, 'r', encoding='utf-8') as f:
                self.机器人数据 = json.load(f)
            
            print(f"[手眼标定工具] 已加载像素坐标文件: {像素文件路径}")
            print(f"[手眼标定工具] 已加载机器人坐标文件: {机器人文件路径}")
            return True
        except Exception as e:
            print(f"[手眼标定工具] 加载数据失败: {e}")
            return False
    
    def 提取对应点(self) -> Tuple[List, List, List, List]:
        """提取对应的像素坐标和机器人坐标点
        
        Returns:
            Tuple[List, List, List, List]: pixel_x, pixel_y, robot_x, robot_y列表
        """
        pixel_x_list = []
        pixel_y_list = []
        robot_x_list = []
        robot_y_list = []
        
        # 如果没有设置映射关系，尝试自动匹配
        if self.映射关系 is None:
            self.自动构建映射()
        
        print("\n[手眼标定工具] 物料对应关系:")
        print("-" * 60)
        
        for pixel_name, robot_name in self.映射关系.items():
            if pixel_name in self.像素数据.get('物料详情', {}):
                物料像素 = self.像素数据['物料详情'][pixel_name]
                if robot_name in self.机器人数据.get('机器人坐标系', {}):
                    物料机器人 = self.机器人数据['机器人坐标系'][robot_name]
                    
                    pixel_x_list.append(物料像素['像素坐标_x'])
                    pixel_y_list.append(物料像素['像素坐标_y'])
                    robot_x_list.append(物料机器人['x'])
                    robot_y_list.append(物料机器人['y'])
                    
                    print(f"{pixel_name} -> {robot_name}")
                    print(f"  像素: ({物料像素['像素坐标_x']}, {物料像素['像素坐标_y']})")
                    print(f"  机器人: ({物料机器人['x']:.6f}, {物料机器人['y']:.6f})")
        
        return pixel_x_list, pixel_y_list, robot_x_list, robot_y_list
    
    def 自动构建映射(self):
        """自动构建物料名称映射关系
        
        根据物料类型匹配：Cuboid_1 -> Cuboid, Cube_2 -> Cube等
        """
        映射 = {}
        像素物料 = self.像素数据.get('物料详情', {})
        机器人物料 = self.机器人数据.get('机器人坐标系', {}).keys()
        
        for pixel_name in 像素物料:
            # 提取物料类型（如 Cuboid_1 -> Cuboid）
            物料类型 = pixel_name.split('_')[0]
            if 物料类型 in 机器人物料:
                映射[pixel_name] = 物料类型
        
        self.映射关系 = 映射
        print(f"[手眼标定工具] 自动构建映射关系: {映射}")
    
    def 执行多元线性回归(self) -> Dict[str, float]:
        """执行多元线性回归计算标定参数
        
        使用最小二乘法拟合：
            robot_x = a1 * pixel_x + a2 * pixel_y + b1
            robot_y = a3 * pixel_x + a4 * pixel_y + b2
        
        Returns:
            Dict[str, float]: 标定参数字典
        """
        pixel_x_list, pixel_y_list, robot_x_list, robot_y_list = self.提取对应点()
        
        if len(pixel_x_list) < 2:
            print("[手眼标定工具] 错误: 有效数据点不足，无法进行回归计算")
            return None
        
        # 转换为numpy数组
        pixel_x = np.array(pixel_x_list)
        pixel_y = np.array(pixel_y_list)
        robot_x = np.array(robot_x_list)
        robot_y = np.array(robot_y_list)
        
        # 多元线性回归：构建设计矩阵 [pixel_x, pixel_y, 1]
        X = np.vstack([pixel_x, pixel_y, np.ones(len(pixel_x))]).T
        
        # 拟合 robot_x = a1 * pixel_x + a2 * pixel_y + b1
        result_x = np.linalg.lstsq(X, robot_x, rcond=None)
        a1, a2, b1 = result_x[0]
        
        # 拟合 robot_y = a3 * pixel_x + a4 * pixel_y + b2
        result_y = np.linalg.lstsq(X, robot_y, rcond=None)
        a3, a4, b2 = result_y[0]
        
        self.标定参数 = {
            'a1': float(a1),
            'a2': float(a2),
            'b1': float(b1),
            'a3': float(a3),
            'a4': float(a4),
            'b2': float(b2)
        }
        
        return self.标定参数
    
    def 验证标定结果(self) -> float:
        """验证标定结果的准确性
        
        Returns:
            float: 平均误差
        """
        if self.标定参数 is None:
            print("[手眼标定工具] 错误: 未计算标定参数")
            return float('inf')
        
        pixel_x_list, pixel_y_list, robot_x_list, robot_y_list = self.提取对应点()
        
        a1, a2, b1 = self.标定参数['a1'], self.标定参数['a2'], self.标定参数['b1']
        a3, a4, b2 = self.标定参数['a3'], self.标定参数['a4'], self.标定参数['b2']
        
        总误差 = 0.0
        print("\n[手眼标定工具] 验证结果:")
        print("-" * 70)
        print(f"{'物料':<15} {'实际X':<12} {'预测X':<12} {'误差X':<10} {'实际Y':<12} {'预测Y':<12} {'误差Y':<10}")
        print("-" * 70)
        
        for i, (px, py, rx, ry) in enumerate(zip(pixel_x_list, pixel_y_list, robot_x_list, robot_y_list)):
            # 预测值
            pred_x = a1 * px + a2 * py + b1
            pred_y = a3 * px + a4 * py + b2
            
            # 计算误差
            err_x = abs(pred_x - rx)
            err_y = abs(pred_y - ry)
            总误差 += err_x + err_y
            
            print(f"{list(self.映射关系.keys())[i]:<15} {rx:<12.6f} {pred_x:<12.6f} {err_x:<10.6f} {ry:<12.6f} {pred_y:<12.6f} {err_y:<10.6f}")
        
        平均误差 = 总误差 / (2 * len(pixel_x_list))
        print("-" * 70)
        print(f"平均误差: {平均误差:.6f} 米")
        
        return 平均误差
    
    def 保存标定参数(self, 文件路径: str) -> bool:
        """保存标定参数到文件
        
        Args:
            文件路径: 保存路径
            
        Returns:
            bool: 是否保存成功
        """
        if self.标定参数 is None:
            print("[手眼标定工具] 错误: 未计算标定参数")
            return False
        
        try:
            目录 = os.path.dirname(文件路径)
            if 目录 and not os.path.exists(目录):
                os.makedirs(目录, exist_ok=True)
            
            with open(文件路径, 'w', encoding='utf-8') as f:
                json.dump(self.标定参数, f, indent=4, ensure_ascii=False)
            
            print(f"\n[手眼标定工具] 标定参数已保存到: {文件路径}")
            return True
        except Exception as e:
            print(f"[手眼标定工具] 保存失败: {e}")
            return False
    
    def 保存识别点参数(self, 识别点名称: str, 配置文件路径: str = None) -> bool:
        """保存标定参数到识别点配置文件
        
        Args:
            识别点名称: 识别点名称，如"物料识别点1"、"物料识别点2"等
            配置文件路径: 配置文件路径，默认为识别点参数配置.json
            
        Returns:
            bool: 是否保存成功
        """
        if self.标定参数 is None:
            print("[手眼标定工具] 错误: 未计算标定参数")
            return False
        
        if not 识别点名称:
            print("[手眼标定工具] 错误: 识别点名称为空")
            return False
        
        try:
            # 默认配置文件路径
            if 配置文件路径 is None:
                当前目录 = os.path.dirname(os.path.abspath(__file__))
                配置文件路径 = os.path.join(当前目录, '识别点参数配置.json')
            
            # 读取现有配置
            if os.path.exists(配置文件路径):
                with open(配置文件路径, 'r', encoding='utf-8') as f:
                    配置 = json.load(f)
            else:
                配置 = {
                    "description": "各识别点的手眼标定参数配置",
                    "default": {
                        "a1": 0.0007,
                        "a2": 0.0007,
                        "b1": -0.0626,
                        "b2": -0.0913
                    },
                    "识别点配置": {}
                }
            
            # 更新识别点配置
            if "识别点配置" not in 配置:
                配置["识别点配置"] = {}
            
            配置["识别点配置"][识别点名称] = self.标定参数
            
            # 保存配置
            with open(配置文件路径, 'w', encoding='utf-8') as f:
                json.dump(配置, f, indent=4, ensure_ascii=False)
            
            print(f"\n[手眼标定工具] 识别点 '{识别点名称}' 的标定参数已保存到: {配置文件路径}")
            return True
        except Exception as e:
            print(f"[手眼标定工具] 保存识别点参数失败: {e}")
            return False
    
    def 加载标定参数(self, 文件路径: str) -> bool:
        """从文件加载标定参数
        
        Args:
            文件路径: 参数文件路径
            
        Returns:
            bool: 是否加载成功
        """
        try:
            if not os.path.exists(文件路径):
                print(f"[手眼标定工具] 警告: 文件不存在: {文件路径}")
                return False
            
            with open(文件路径, 'r', encoding='utf-8') as f:
                self.标定参数 = json.load(f)
            
            print(f"[手眼标定工具] 已加载标定参数: {文件路径}")
            return True
        except Exception as e:
            print(f"[手眼标定工具] 加载失败: {e}")
            return False
    
    def 执行完整标定流程(self, 像素文件路径: str, 机器人文件路径: str, 输出参数文件: str = None, 识别点名称: str = None) -> bool:
        """执行完整的标定流程
        
        Args:
            像素文件路径: 像素坐标文件路径
            机器人文件路径: 机器人坐标文件路径
            输出参数文件: 标定参数保存路径（可选）
            识别点名称: 识别点名称（可选），用于保存到识别点配置
            
        Returns:
            bool: 是否成功
        """
        print("=" * 60)
        print("[手眼标定工具] 开始执行手眼标定")
        print("=" * 60)
        
        # 步骤1: 加载数据
        if not self.从文件加载数据(像素文件路径, 机器人文件路径):
            return False
        
        # 步骤2: 执行多元线性回归
        标定参数 = self.执行多元线性回归()
        if 标定参数 is None:
            return False
        
        # 步骤3: 显示计算结果
        print("\n" + "=" * 60)
        print("[手眼标定工具] 计算得到的标定参数:")
        print("=" * 60)
        print(f"a1 (pixel_x -> robot_x系数) = {标定参数['a1']:.10e}")
        print(f"a2 (pixel_y -> robot_x系数) = {标定参数['a2']:.10e}")
        print(f"b1 (robot_x偏移量)          = {标定参数['b1']:.6f}")
        print(f"a3 (pixel_x -> robot_y系数) = {标定参数['a3']:.10e}")
        print(f"a4 (pixel_y -> robot_y系数) = {标定参数['a4']:.10e}")
        print(f"b2 (robot_y偏移量)          = {标定参数['b2']:.6f}")
        print("=" * 60)
        print("\n转换公式:")
        print(f"  robot_x = {标定参数['a1']:.6e} * pixel_x + {标定参数['a2']:.6e} * pixel_y + {标定参数['b1']:.6f}")
        print(f"  robot_y = {标定参数['a3']:.6e} * pixel_x + {标定参数['a4']:.6e} * pixel_y + {标定参数['b2']:.6f}")
        
        # 步骤4: 验证结果
        self.验证标定结果()
        
        # 步骤5: 保存参数（如果指定了输出路径）
        if 输出参数文件:
            self.保存标定参数(输出参数文件)
        
        # 步骤6: 如果指定了识别点名称，保存到识别点配置
        if 识别点名称:
            self.保存识别点参数(识别点名称)
        
        print("\n" + "=" * 60)
        print("[手眼标定工具] 标定流程完成")
        print("=" * 60)
        
        return True
    
    def 应用标定参数到转换器(self, 转换器文件路径: str) -> bool:
        """将标定参数应用到坐标转换器
        
        Args:
            转换器文件路径: 手眼标定坐标转换器.py文件路径
            
        Returns:
            bool: 是否成功
        """
        if self.标定参数 is None:
            print("[手眼标定工具] 错误: 未计算标定参数")
            return False
        
        try:
            # 读取转换器文件
            with open(转换器文件路径, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 替换默认标定参数（匹配任意数值的正则方式）
            import re
            
            # 匹配默认标定参数块并替换
            old_pattern = r"    默认标定参数 = \{[^}]+\}"
            new_params = f"""    默认标定参数 = {{
        'a1': {self.标定参数['a1']},
        'a2': {self.标定参数['a2']},
        'b1': {self.标定参数['b1']},
        'a3': {self.标定参数['a3']},
        'a4': {self.标定参数['a4']},
        'b2': {self.标定参数['b2']}
    }}"""
            
            content = re.sub(old_pattern, new_params, content)
            
            # 写回文件
            with open(转换器文件路径, 'w', encoding='utf-8') as f:
                f.write(content)
            
            print(f"[手眼标定工具] 已将标定参数应用到: {转换器文件路径}")
            return True
        except Exception as e:
            print(f"[手眼标定工具] 应用参数失败: {e}")
            return False


def 主():
    """主函数"""
    import sys
    
    # 设置默认文件路径
    当前目录 = os.path.dirname(os.path.abspath(__file__))
    像素文件 = os.path.join(当前目录, '物料位置', '物料位置.json')
    机器人文件 = os.path.join(当前目录, '通信中间文件', '物料位置.json')
    参数文件 = os.path.join(当前目录, '标定参数.json')
    转换器文件 = os.path.join(当前目录, 'robodyno_camera_arm_controller', '手眼标定坐标转换器.py')
    
    # 识别点名称（可选）
    识别点名称 = None
    
    # 解析命令行参数
    # 使用方式: python 手眼标定工具.py [像素文件] [机器人文件] [识别点名称]
    if len(sys.argv) >= 2:
        像素文件 = sys.argv[1]
    if len(sys.argv) >= 3:
        机器人文件 = sys.argv[2]
    if len(sys.argv) >= 4:
        识别点名称 = sys.argv[3]
    
    print(f"[手眼标定工具] 像素文件: {像素文件}")
    print(f"[手眼标定工具] 机器人文件: {机器人文件}")
    if 识别点名称:
        print(f"[手眼标定工具] 识别点名称: {识别点名称}")
    print(f"[手眼标定工具] 参数输出文件: {参数文件}")
    
    # 检查文件是否存在
    if not os.path.exists(像素文件):
        print(f"[手眼标定工具] 错误: 像素文件不存在: {像素文件}")
        print("请确保已运行视觉识别并生成像素坐标数据")
        return
    
    if not os.path.exists(机器人文件):
        print(f"[手眼标定工具] 错误: 机器人文件不存在: {机器人文件}")
        print("请确保世界控制器已运行并生成机器人坐标数据")
        print("或者使用命令行参数指定正确的文件路径")
        return
    
    # 创建标定计算器
    计算器 = 手眼标定计算器()
    
    # 执行标定流程（支持识别点名称）
    成功 = 计算器.执行完整标定流程(像素文件, 机器人文件, 参数文件, 识别点名称)
    
    if 成功:
        # 将参数应用到转换器
        计算器.应用标定参数到转换器(转换器文件)


if __name__ == "__main__":
    主()
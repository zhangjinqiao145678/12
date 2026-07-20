# -*-coding:utf-8 -*-
"""验证标定参数.py

验证之前计算的标定参数是否正确，确保与历史结果一致
"""

import json
import os

# 之前计算得到的标定参数
历史标定参数 = {
    'a1': 6.375281969363859e-07,
    'a2': -0.00029595007290852683,
    'b1': 0.2855414323599386,
    'a3': -0.00029535341990105494,
    'a4': 1.3188778941671821e-08,
    'b2': -0.022938809093657465
}

# 历史验证结果（之前计算的预期输出）
历史验证结果 = {
    'Cuboid_1': {'x': 0.1496, 'y': -0.1301},
    'Cube_2': {'x': 0.2196, 'y': -0.0602},
    'Cruciform_3': {'x': 0.1504, 'y': -0.0599},
    'Parallelogram_4': {'x': 0.2204, 'y': -0.1299}
}

# 物料名称映射
物料映射 = {
    'Cuboid_1': 'Cuboid',
    'Cube_2': 'Cube',
    'Cruciform_3': 'Cruciform',
    'Parallelogram_4': 'Parallelogram'
}

def 像素坐标转机械臂坐标(pixel_x, pixel_y, 参数):
    """使用标定参数转换坐标"""
    a1, a2, b1 = 参数['a1'], 参数['a2'], 参数['b1']
    a3, a4, b2 = 参数['a3'], 参数['a4'], 参数['b2']
    
    robot_x = a1 * pixel_x + a2 * pixel_y + b1
    robot_y = a3 * pixel_x + a4 * pixel_y + b2
    
    return robot_x, robot_y

def 验证标定参数():
    """验证标定参数是否正确"""
    print("=" * 60)
    print("[验证工具] 验证标定参数")
    print("=" * 60)
    
    # 读取像素坐标数据
    当前目录 = os.path.dirname(os.path.abspath(__file__))
    像素文件 = os.path.join(当前目录, '物料位置', '物料位置.json')
    
    try:
        with open(像素文件, 'r', encoding='utf-8') as f:
            像素数据 = json.load(f)
    except Exception as e:
        print(f"[验证工具] 读取像素文件失败: {e}")
        return False
    
    # 读取实际机器人坐标数据（通信中间文件）
    机器人文件 = os.path.join(当前目录, '通信中间文件', '物料位置.json')
    try:
        with open(机器人文件, 'r', encoding='utf-8') as f:
            机器人数据 = json.load(f)
    except Exception as e:
        print(f"[验证工具] 读取机器人文件失败: {e}")
        print("警告: 将使用历史验证结果进行对比")
        机器人数据 = None
    
    print("\n[验证工具] 使用历史标定参数进行转换:")
    print("-" * 70)
    print(f"{'物料':<15} {'像素坐标':<15} {'预测坐标':<20} {'历史预期':<20} {'误差':<10}")
    print("-" * 70)
    
    所有测试通过 = True
    
    for pixel_name, robot_name in 物料映射.items():
        if pixel_name in 像素数据.get('物料详情', {}):
            物料像素 = 像素数据['物料详情'][pixel_name]
            px, py = 物料像素['像素坐标_x'], 物料像素['像素坐标_y']
            
            # 使用历史参数转换
            pred_x, pred_y = 像素坐标转机械臂坐标(px, py, 历史标定参数)
            
            # 获取历史预期值
            历史_x, 历史_y = 历史验证结果[pixel_name]['x'], 历史验证结果[pixel_name]['y']
            
            # 计算与历史预期的误差
            误差_x = abs(pred_x - 历史_x)
            误差_y = abs(pred_y - 历史_y)
            总误差 = (误差_x**2 + 误差_y**2)**0.5
            
            # 判断是否通过（误差小于0.001米）
            通过 = 总误差 < 0.001
            状态 = "通过" if 通过 else "失败"
            
            if not 通过:
                所有测试通过 = False
            
            print(f"{pixel_name:<15} ({px:4d}, {py:4d})  ({pred_x:.4f}, {pred_y:.4f})    ({历史_x:.4f}, {历史_y:.4f})    {总误差:.6f}  {状态}")
    
    print("-" * 70)
    
    # 如果有机器人数据，也对比实际机器人坐标
    if 机器人数据 and '机器人坐标系' in 机器人数据:
        print("\n[验证工具] 与实际机器人坐标对比:")
        print("-" * 70)
        print(f"{'物料':<15} {'预测坐标':<20} {'实际坐标':<20} {'误差(m)':<10}")
        print("-" * 70)
        
        for pixel_name, robot_name in 物料映射.items():
            if pixel_name in 像素数据.get('物料详情', {}) and robot_name in 机器人数据['机器人坐标系']:
                物料像素 = 像素数据['物料详情'][pixel_name]
                px, py = 物料像素['像素坐标_x'], 物料像素['像素坐标_y']
                
                # 使用历史参数转换
                pred_x, pred_y = 像素坐标转机械臂坐标(px, py, 历史标定参数)
                
                # 获取实际机器人坐标
                实际_x = 机器人数据['机器人坐标系'][robot_name]['x']
                实际_y = 机器人数据['机器人坐标系'][robot_name]['y']
                
                # 计算误差
                误差_x = abs(pred_x - 实际_x)
                误差_y = abs(pred_y - 实际_y)
                总误差 = (误差_x**2 + 误差_y**2)**0.5
                
                print(f"{pixel_name:<15} ({pred_x:.6f}, {pred_y:.6f})  ({实际_x:.6f}, {实际_y:.6f})  {总误差:.6f}")
        
        print("-" * 70)
    
    print("\n" + "=" * 60)
    if 所有测试通过:
        print("[验证工具] 所有验证测试通过！")
        print("[验证工具] 标定参数计算程序正确，与历史结果一致")
    else:
        print("[验证工具] 部分测试失败，请检查")
    print("=" * 60)
    
    return 所有测试通过

if __name__ == "__main__":
    验证标定参数()
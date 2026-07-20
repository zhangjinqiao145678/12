# -*-coding:utf-8 -*-
"""图像接收使用示例.py
Time    :   2025/06/11
Author  :   机械臂控制系统
Version :   1.0
Contact :   aweidw@163.com
License :   (C)Copyright 2024, robottime / robodyno

Summary

  摄像头图像接收模块的使用示例
  - 演示如何调用 摄像头图像接收模块 获取摄像头数据
  - 演示如何将获取到的图像用于识别、显示等后续处理

使用方法：
  1. 启动 Webots 仿真，确保 camera_capture_controller 和 robot_arm_controller 都在运行
  2. 运行本文件即可看到图像接收和基本处理的效果
"""

import sys
import os

# 确保能正确导入同级模块
当前文件目录 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, 当前文件目录)

from 摄像头图像接收模块 import 摄像头图像接收接口, 创建摄像头接收接口

try:
    import numpy as np
except ImportError:
    print("[错误] 请先安装 numpy: pip install numpy")
    sys.exit(1)

# 可选：如果需要显示图像，安装 opencv-python
try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


def 示例_基本读取():
    """示例1：最简单的图像读取

    演示如何从中间文件获取最新的一帧图像并检查其基本属性。
    """
    print("\n" + "=" * 60)
    print("【示例1】基本图像读取")
    print("=" * 60)

    接收接口 = 创建摄像头接收接口()
    结果 = 接收接口.获取最新图像()

    if 结果 is None:
        print("  ↳ 未能获取图像（可能摄像头端未启动）")
        return

    图像数组, 宽度, 高度 = 结果
    print(f"  ↳ 成功获取图像: {宽度}x{高度}")
    print(f"  ↳ 数组形状: {图像数组.shape}")
    print(f"  ↳ 数据类型: {图像数组.dtype}")
    print(f"  ↳ 像素范围: {图像数组.min()} ~ {图像数组.max()}")
# 示例_基本读取() 结束


def 示例_循环读取新图像(最大帧数: int = 10):
    """示例2：循环读取新图像

    演示如何只在有新图像时才返回数据。

    Args:
        最大帧数: 最多读取多少帧后停止
    """
    print("\n" + "=" * 60)
    print(f"【示例2】循环读取新图像（最多 {最大帧数} 帧）")
    print("=" * 60)

    接收接口 = 创建摄像头接收接口()
    已读取帧数 = 0

    import time
    while 已读取帧数 < 最大帧数:
        结果 = 接收接口.获取最新图像(转换为BGR=True, 仅当有新图像时=True)
        if 结果 is not None:
            图像数组, 宽度, 高度 = 结果
            已读取帧数 += 1
            print(f"  [{已读取帧数}/{最大帧数}] 新帧: {宽度}x{高度}")
        time.sleep(0.05)

    print(f"  ↳ 共读取 {已读取帧数} 帧新图像")
# 示例_循环读取新图像() 结束


def 示例_简单图像处理():
    """示例3：简单的图像预处理

    演示获取图像后，如何做一些常见的预处理：
      - 灰度化
      - 二值化（阈值分割）
      - 这些都是后续物料识别的前置步骤
    """
    print("\n" + "=" * 60)
    print("【示例3】简单图像预处理")
    print("=" * 60)

    接收接口 = 创建摄像头接收接口()
    结果 = 接收接口.获取最新图像()
    if 结果 is None:
        print("  ↳ 未能获取图像，跳过此示例")
        return

    图像数组, 宽度, 高度 = 结果
    print(f"  ↳ 原始图像: {宽度}x{高度}, {图像数组.shape[2]} 通道")

    # 使用 numpy 进行简单图像处理（不依赖 opencv 也可以工作）
    # 灰度化：BGR -> 灰度（加权平均法）
    if 图像数组.shape[2] >= 3:
        B = 图像数组[:, :, 0].astype(np.float32)
        G = 图像数组[:, :, 1].astype(np.float32)
        R = 图像数组[:, :, 2].astype(np.float32)
        灰度图 = (0.299 * R + 0.587 * G + 0.114 * B).astype(np.uint8)
        print(f"  ↳ 灰度化完成: {灰度图.shape}")

        # 简单阈值分割：亮度大于 127 的像素设为 255，其他为 0
        二值图 = (灰度图 > 127).astype(np.uint8) * 255
        print(f"  ↳ 二值化完成: 白色像素数 = {int((二值图 == 255).sum())}")
    print("  ↳ 图像预处理完成，可以用于后续物料识别")
# 示例_简单图像处理() 结束


def 示例_显示图像窗口():
    """示例4：使用 OpenCV 显示图像窗口

    如果安装了 opencv-python，可以用这个示例实时查看摄像头画面。
    """
    print("\n" + "=" * 60)
    print("【示例4】OpenCV 图像窗口显示")
    print("=" * 60)

    if not _HAS_CV2:
        print("  ↳ 未安装 opencv-python，跳过此示例")
        print("  ↳ 如需安装：pip install opencv-python")
        return

    接收接口 = 创建摄像头接收接口()
    print("  ↳ 正在打开图像窗口（在窗口内按 'q' 键退出）...")

    import time
    try:
        while True:
            结果 = 接收接口.获取最新图像(转换为BGR=True, 仅当有新图像时=True)
            if 结果 is not None:
                图像数组, 宽度, 高度 = 结果
                # 显示图像
                cv2.imshow("机械臂端摄像头画面", 图像数组)
                # 按 q 键退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n  ↳ 用户中断")
    finally:
        cv2.destroyAllWindows()
    # try-finally 结束
# 示例_显示图像窗口() 结束


def 示例_打印通信摘要():
    """示例5：打印当前通信连接摘要"""
    print("\n" + "=" * 60)
    print("【示例5】通信连接摘要")
    print("=" * 60)

    接收接口 = 创建摄像头接收接口()
    print(接收接口.获取通信信息摘要())
# 示例_打印通信摘要() 结束


def main():
    """主函数：依次运行所有示例

    运行顺序：通信摘要 -> 基本读取 -> 循环读取 -> 图像预处理 ->（可选）窗口显示
    """
    print("\n★ 摄像头图像接收模块 - 使用示例 ★")
    print("★ 请确保 camera_capture_controller 已在 Webots 中运行 ★")

    示例_打印通信摘要()
    示例_基本读取()
    示例_循环读取新图像(最大帧数=5)
    示例_简单图像处理()

    # 如果您想测试实时显示，取消下面这行的注释：
    # 示例_显示图像窗口()

    print("\n" + "=" * 60)
    print("所有示例运行结束。")
    print("=" * 60)
    print("\n提示：")
    print("  在您的实际程序中，只需这样使用：")
    print("  ───────────────────────────────")
    print("  from 摄像头图像接收模块 import 摄像头图像接收接口")
    print("  接收接口 = 摄像头图像接收接口()")
    print("  结果 = 接收接口.获取最新图像()")
    print("  if 结果 is not None:")
    print("      图像数组, 宽度, 高度 = 结果")
    print("      # 在这里进行您的识别处理...")
    print("  ───────────────────────────────\n")
# main() 结束


if __name__ == "__main__":
    main()
# main() 结束

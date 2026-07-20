# -*-coding:utf-8 -*-
"""机器臂控制窗口.py
Time    :   2025/05/24
Author  :   机器臂控制系统
Version :   1.0
Contact :   aweidw@163.com
License :   (C)Copyright 2024, robottime / robodyno

Summary

  机械臂GUI控制窗口模块
  提供位姿输入界面，通过多线程与主控制器共享机械臂对象
"""
import tkinter as tk
from tkinter import ttk, messagebox
from math import pi
import threading
import os
import sys

try:
    from 物料抓取控制 import 创建自动抓取按钮
except ImportError:
    创建自动抓取按钮 = None

try:
    from 顺序巡航抓取 import 创建巡航抓取按钮
except ImportError:
    创建巡航抓取按钮 = None

# 尝试导入摄像头图像接收模块（如果存在）
try:
    当前文件目录 = os.path.dirname(os.path.abspath(__file__))
    图像处理目录 = os.path.join(当前文件目录, "机器臂端图像处理程序文件夹")
    if 图像处理目录 not in sys.path:
        sys.path.insert(0, 图像处理目录)
    from 摄像头图像接收模块 import 摄像头图像接收接口
    _HAS_CAMERA_MODULE = True
except Exception:
    _HAS_CAMERA_MODULE = False
    摄像头图像接收接口 = None

# 尝试导入 PIL（用于在 Tkinter 中显示图像）
try:
    from PIL import Image as PILImage, ImageTk
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


class ArmPoseValidator:
    """位姿参数验证器

    用于验证用户输入的位姿参数是否在有效范围内
    """
    X_MIN, X_MAX = -0.3, 0.4
    Y_MIN, Y_MAX = -0.3, 0.3
    Z_MIN, Z_MAX = 0.0, 0.4
    R_MIN, R_MAX = -pi, pi

    @classmethod
    def validate_x(cls, value):
        """验证X坐标"""
        return cls._validate("X", value, cls.X_MIN, cls.X_MAX)

    @classmethod
    def validate_y(cls, value):
        """验证Y坐标"""
        return cls._validate("Y", value, cls.Y_MIN, cls.Y_MAX)

    @classmethod
    def validate_z(cls, value):
        """验证Z坐标"""
        return cls._validate("Z", value, cls.Z_MIN, cls.Z_MAX)

    @classmethod
    def validate_rx(cls, value):
        """验证RX角度"""
        return cls._validate("RX", value, cls.R_MIN, cls.R_MAX)

    @classmethod
    def validate_ry(cls, value):
        """验证RY角度"""
        return cls._validate("RY", value, cls.R_MIN, cls.R_MAX)

    @classmethod
    def validate_rz(cls, value):
        """验证RZ角度"""
        return cls._validate("RZ", value, cls.R_MIN, cls.R_MAX)

    @classmethod
    def _validate(cls, name, value, vmin, vmax):
        """通用验证方法

        Args:
            name: 参数名称
            value: 待验证的值
            vmin: 最小值
            vmax: 最大值

        Returns:
            tuple: (是否有效, 错误消息)
        """
        try:
            float_val = float(value)
            if vmin <= float_val <= vmax:
                return True, ""
            return False, f"{name}值必须在[{vmin:.2f}, {vmax:.2f}]范围内"
        except ValueError:
            return False, f"{name}必须是有效数字"

    @classmethod
    def validate_pose(cls, x, y, z, rx, ry, rz):
        """验证完整位姿

        Args:
            x, y, z: 位置坐标
            rx, ry, rz: 姿态角度

        Returns:
            tuple: (是否全部有效, 错误消息列表)
        """
        errors = []
        for validate_func, val, name in [
            (cls.validate_x, x, "X"),
            (cls.validate_y, y, "Y"),
            (cls.validate_z, z, "Z"),
            (cls.validate_rx, rx, "RX"),
            (cls.validate_ry, ry, "RY"),
            (cls.validate_rz, rz, "RZ"),
        ]:
            valid, msg = validate_func(val)
            if not valid:
                errors.append(msg)
        return len(errors) == 0, errors


class ArmGUI:
    """机械臂控制GUI窗口类

    提供Tkinter界面，用于输入末端执行器位姿并控制机械臂运动
    """

    DEFAULT_POSE = {"x": 0.110, "y": 0.2075, "z": 0.02, "rx": -pi, "ry": 0, "rz": 0}

    def __init__(self, arm, shared_event, 物料数据=None):
        """初始化GUI窗口

        Args:
            arm: 机械臂对象（由主控制器传入）
            shared_event: 线程间共享的事件标志
            物料数据: 物料位置数据对象，默认为None则不启用自动抓取功能
        """
        self.arm = arm
        self.shared_event = shared_event
        self.物料数据 = 物料数据
        self.root = None
        self.entries = {}
        self.status_label = None
        self.is_moving = False
        self.吸盘启动指示灯 = None
        self.抓取状态指示灯 = None
        self.指示灯更新任务_id = None

        # ============================================================
        # 共享识别窗口相关成员变量（供顺序巡航识别使用）
        # ============================================================
        self._识别窗口 = None          # 共享的 Toplevel 窗口引用
        self._识别窗口画布 = None      # Canvas 控件，显示图像
        self._识别窗口标签 = None      # Label 控件，显示识别点名称/状态
        self._识别窗口图像 = None      # PhotoImage 引用，防止被回收
        self._图像接收接口 = None      # 摄像头图像接收接口实例（延迟初始化）

        # 尝试导入摄像头图像接收接口
        if _HAS_CAMERA_MODULE and 摄像头图像接收接口 is not None:
            self._图像接收接口 = 摄像头图像接收接口()

    def _create_entry(self, parent, label_text, row, default_value):
        """创建标签-输入框组合

        Args:
            parent: 父容器
            label_text: 标签文本
            row: 行号
            default_value: 默认值

        Returns:
            ttk.Entry: 创建的输入框控件
        """
        label = ttk.Label(parent, text=label_text, width=8)
        label.grid(row=row, column=0, padx=5, pady=3, sticky="e")
        entry = ttk.Entry(parent, width=12)
        entry.grid(row=row, column=1, padx=5, pady=3)
        entry.insert(0, str(default_value))
        return entry

    def _validate_and_get_pose(self):
        """验证并获取当前输入的位姿

        Returns:
            tuple: (是否成功, 位姿字典或错误消息)
        """
        try:
            pose = {
                "x": float(self.entries["x"].get()),
                "y": float(self.entries["y"].get()),
                "z": float(self.entries["z"].get()),
                "rx": float(self.entries["rx"].get()),
                "ry": float(self.entries["ry"].get()),
                "rz": float(self.entries["rz"].get()),
            }
            valid, errors = ArmPoseValidator.validate_pose(**pose)
            if not valid:
                return False, "\n".join(errors)
            return True, pose
        except ValueError as e:
            return False, f"输入格式错误: {str(e)}"

    def _on_move(self):
        """移动按钮回调"""
        if self.is_moving:
            messagebox.showwarning("警告", "机械臂正在运动中，请等待完成")
            return
        valid, result = self._validate_and_get_pose()
        if not valid:
            messagebox.showerror("输入错误", result)
            return
        self.is_moving = True
        self._update_status("运动中...")
        try:
            target_pose = result
            joint_angles = self.arm.inverse_kinematics(
                target_pose["x"], target_pose["y"], target_pose["z"],
                target_pose["rx"], target_pose["ry"], target_pose["rz"]
            )
            self.arm.joint_space_interpolated_motion(joint_angles, duration=2)
            self._update_status("运动完成")
        except Exception as e:
            messagebox.showerror("运动错误", str(e))
            self._update_status("运动失败")
        finally:
            self.is_moving = False

    def _on_home(self):
        """回归原点按钮回调"""
        if self.is_moving:
            messagebox.showwarning("警告", "机械臂正在运动中，请等待完成")
            return
        self.is_moving = True
        self._update_status("回归原点中...")
        try:
            self.arm.home(2)
            self._update_status("已回归原点")
        except Exception as e:
            messagebox.showerror("运动错误", str(e))
            self._update_status("回归原点失败")
        finally:
            self.is_moving = False

    def _on_enable_suction(self):
        """开启吸盘按钮回调"""
        try:
            self.arm.enable_end_effector()
            self._update_status("真空吸盘已开启")
        except Exception as e:
            messagebox.showerror("错误", f"开启吸盘失败: {str(e)}")

    def _on_disable_suction(self):
        """关闭吸盘按钮回调"""
        try:
            self.arm.disable_end_effector()
            self._update_status("真空吸盘已关闭")
        except Exception as e:
            messagebox.showerror("错误", f"关闭吸盘失败: {str(e)}")

    def _update_status(self, text):
        """更新状态显示

        Args:
            text: 状态文本
        """
        if self.status_label:
            self.status_label.config(text=f"状态: {text}")

    def _safe_update_status(self, text):
        """线程安全地更新状态文本"""
        if self.root is not None:
            self.root.after(0, lambda: self._update_status(text))
        else:
            self._update_status(text)

    def _绘制指示灯(self, canvas, is_on):
        """绘制指示灯

        Args:
            canvas: Canvas控件
            is_on: 是否点亮
        """
        canvas.delete("all")
        color = "#00FF00" if is_on else "#808080"
        canvas.create_oval(2, 2, 18, 18, fill=color, outline="black")

    def _更新指示灯状态(self):
        """更新指示灯状态"""
        if self.root is None:
            return

        try:
            吸盘启动 = self.arm.is_end_effector_enabled()
            抓取状态 = self.arm.is_object_grasped()

            self._绘制指示灯(self.吸盘启动指示灯, 吸盘启动)
            self._绘制指示灯(self.抓取状态指示灯, 抓取状态)
        except Exception:
            pass

        self.指示灯更新任务_id = self.root.after(200, self._更新指示灯状态)

    def _启动指示灯更新(self):
        """启动指示灯定期更新"""
        self._更新指示灯状态()

    def _停止指示灯更新(self):
        """停止指示灯定期更新"""
        if self.指示灯更新任务_id is not None:
            self.root.after_cancel(self.指示灯更新任务_id)
            self.指示灯更新任务_id = None

    # ============================================================
    # 摄像头画面显示功能
    # ============================================================
    def _初始化摄像头接口(self):
        """初始化摄像头图像接收接口

        仅在首次点击"显示摄像头"时调用，避免重复创建实例。
        """
        if not _HAS_CAMERA_MODULE:
            return None
        if getattr(self, "_摄像头接口", None) is None:
            self._摄像头接口 = 摄像头图像接收接口()
        return self._摄像头接口

    def _on_show_camera(self):
        """显示摄像头按键回调

        点击后弹出一个独立窗口，显示当前接收到的摄像头画面。
        画面以 1 赫兹频率更新（每秒刷新一次）。
        """
        摄像头接口 = self._初始化摄像头接口()
        if 摄像头接口 is None:
            messagebox.showerror("错误", "摄像头图像接收模块未找到")
            return

        if not _HAS_PIL:
            messagebox.showerror("错误", "未安装 PIL/Pillow。请先运行: pip install Pillow")
            return

        if not _HAS_NUMPY:
            messagebox.showerror("错误", "未安装 numpy。请先运行: pip install numpy")
            return

        # 如果已经有摄像头窗口，就不重复创建
        if getattr(self, "_摄像头窗口", None) is not None and self._摄像头窗口.winfo_exists():
            self._摄像头窗口.lift()
            self._摄像头窗口.focus_force()
            return

        # 创建独立的摄像头显示窗口
        self._摄像头窗口 = tk.Toplevel(self.root)
        self._摄像头窗口.title("摄像头画面")
        self._摄像头窗口.resizable(False, False)

        # 信息标签
        self._摄像头信息标签 = ttk.Label(self._摄像头窗口, text="等待图像...", foreground="blue")
        self._摄像头信息标签.pack(padx=10, pady=(8, 2))

        # 画面显示区（用 Canvas，初始显示 640x480 灰色占位图）
        self._摄像头画布 = tk.Canvas(self._摄像头窗口, width=640, height=480, bg="#202020")
        self._摄像头画布.pack(padx=10, pady=8)

        # 保存 PhotoImage 引用，防止被 Python 自动回收
        self._当前摄像头图像 = None
        self._图像图像对象 = None

        # 立即刷新一次，然后注册 1 秒后的刷新
        self._刷新摄像头画面()

        # 窗口关闭时清理
        self._摄像头窗口.protocol("WM_DELETE_WINDOW", self._on_camera_window_close)

    def _on_camera_window_close(self):
        """摄像头窗口关闭回调

        取消定时刷新任务并销毁窗口。
        """
        if getattr(self, "_摄像头刷新任务_id", None) is not None:
            try:
                self._摄像头窗口.after_cancel(self._摄像头刷新任务_id)
            except Exception:
                pass
            self._摄像头刷新任务_id = None
        if getattr(self, "_摄像头窗口", None) is not None:
            self._摄像头窗口.destroy()
            self._摄像头窗口 = None

    def _刷新摄像头画面(self):
        """刷新摄像头画面

        从中间文件读取最新图像，并在窗口中显示。
        画面以 1 赫兹频率更新，完成后注册下一次刷新。
        """
        try:
            if getattr(self, "_摄像头窗口", None) is None or not self._摄像头窗口.winfo_exists():
                return

            摄像头接口 = getattr(self, "_摄像头接口", None)
            if 摄像头接口 is None:
                return

            # 读取最新图像
            结果 = 摄像头接口.读取图像(转换为BGR=True)
            if 结果 is None:
                self._摄像头信息标签.config(text="未获取到图像，请确保摄像头端正在运行", foreground="red")
                self._摄像头刷新任务_id = self._摄像头窗口.after(1000, self._刷新摄像头画面)
                return

            图像数组, 宽度, 高度 = 结果
            时间戳 = ""
            状态 = 摄像头接口.读取摄像头状态()
            if 状态 is not None:
                时间戳 = str(状态.get("timestamp", ""))

            # numpy 数组 (H, W, 3) BGR -> PIL Image
            try:
                # 将 BGR 转为 RGB
                RGB数组 = 图像数组[:, :, ::-1]
                pil_image = PILImage.fromarray(RGB数组.astype(np.uint8))
                # 如果尺寸过大，按比例缩小到最大 640 宽
                最大宽度 = 640
                if pil_image.width > 最大宽度:
                    缩放比例 = 最大宽度 / pil_image.width
                    新高度 = int(pil_image.height * 缩放比例)
                    pil_image = pil_image.resize((最大宽度, 新高度), PILImage.BILINEAR)
                # 更新画布尺寸以匹配图像
                if (pil_image.width, pil_image.height) != (self._摄像头画布.winfo_width(), self._摄像头画布.winfo_height()):
                    self._摄像头画布.config(width=pil_image.width, height=pil_image.height)
                self._图像图像对象 = ImageTk.PhotoImage(pil_image)
                self._摄像头画布.delete("all")
                self._摄像头画布.create_image(0, 0, anchor="nw", image=self._图像图像对象)
                self._摄像头信息标签.config(
                    text=f"{pil_image.width}×{pil_image.height}  |  时间: {时间戳}",
                    foreground="green"
                )
            except Exception as e:
                self._摄像头信息标签.config(text=f"图像转换错误: {e}", foreground="red")

            # 注册下一次 1 秒后的刷新
            self._摄像头刷新任务_id = self._摄像头窗口.after(1000, self._刷新摄像头画面)

        except Exception as e:
            # 任何意外错误都记录但不崩溃
            try:
                self._摄像头信息标签.config(text=f"刷新错误: {e}", foreground="red")
                self._摄像头刷新任务_id = self._摄像头窗口.after(1000, self._刷新摄像头画面)
            except Exception:
                pass
    # ============================================================
    # 摄像头画面显示功能结束
    # ============================================================

    # ============================================================
    # 共享识别窗口相关方法（供顺序巡航抓取识别环节调用）
    # ============================================================

    def _获取图像接收接口(self):
        """获取摄像头图像接收接口实例

        Returns:
            摄像头图像接收接口实例，如果未安装模块则返回 None
        """
        if getattr(self, "_图像接收接口", None) is None:
            if _HAS_CAMERA_MODULE and 摄像头图像接收接口 is not None:
                self._图像接收接口 = 摄像头图像接收接口()
        return self._图像接收接口

    def _获取或创建识别窗口(self):
        """获取或创建共享识别窗口

        如果窗口不存在则创建，如果已存在则直接返回。
        该方法在主线程中调用。

        Returns:
            tk.Toplevel: 识别窗口引用
        """
        if getattr(self, "_识别窗口", None) is not None and self._识别窗口.winfo_exists():
            return self._识别窗口

        # 创建新的识别窗口
        self._识别窗口 = tk.Toplevel(self.root)
        self._识别窗口.title("识别结果")
        self._识别窗口.resizable(False, False)

        # 标题/状态标签
        self._识别窗口标签 = ttk.Label(
            self._识别窗口,
            text="等待识别...",
            font=("Microsoft YaHei", 11),
            foreground="blue"
        )
        self._识别窗口标签.pack(padx=10, pady=(10, 5))

        # 图像显示画布
        self._识别窗口画布 = tk.Canvas(
            self._识别窗口, width=640, height=480, bg="#202020"
        )
        self._识别窗口画布.pack(padx=10, pady=(0, 10))

        # 保持窗口引用，防止 PhotoImage 被回收
        self._识别窗口图像 = None

        return self._识别窗口
    # _获取或创建识别窗口() 结束

    def _更新识别窗口内容(self, 图像数组, 宽度, 高度, 标题文本):
        """在主线程中更新识别窗口的图像和标题

        此方法应在主线程中调用，用于将 numpy 图像数组显示到 Canvas 上。

        Args:
            图像数组: numpy 数组 (H, W, 3)，BGR 格式
            宽度: 图像宽度
            高度: 图像高度
            标题文本: 要显示在标签上的文本
        """
        if not _HAS_PIL or not _HAS_NUMPY:
            return

        try:
            窗口 = self._获取或创建识别窗口()
            if not 窗口.winfo_exists():
                return

            # 更新标签文本
            if getattr(self, "_识别窗口标签", None) is not None:
                self._识别窗口标签.config(text=标题文本, foreground="blue")

            # BGR -> RGB，然后转 PIL Image
            RGB数组 = 图像数组[:, :, ::-1]
            pil_image = PILImage.fromarray(RGB数组.astype(np.uint8))

            # 缩放到最大宽度 640
            最大宽度 = 640
            if pil_image.width > 最大宽度:
                缩放比例 = 最大宽度 / pil_image.width
                新高度 = int(pil_image.height * 缩放比例)
                pil_image = pil_image.resize((最大宽度, 新高度), PILImage.BILINEAR)

            # 转换为 PhotoImage
            self._识别窗口图像 = ImageTk.PhotoImage(pil_image)

            # 更新 Canvas
            画布 = getattr(self, "_识别窗口画布", None)
            if 画布 is not None and 画布.winfo_exists():
                画布.delete("all")
                画布.config(width=pil_image.width, height=pil_image.height)
                画布.create_image(0, 0, anchor="nw", image=self._识别窗口图像)

        except Exception as e:
            print(f"[警告] 更新识别窗口失败: {e}")
    # _更新识别窗口内容() 结束

    def _关闭识别窗口(self):
        """关闭共享识别窗口（如果存在）"""
        窗口 = getattr(self, "_识别窗口", None)
        if 窗口 is not None and 窗口.winfo_exists():
            try:
                窗口.destroy()
            except Exception:
                pass
        self._识别窗口 = None
        self._识别窗口画布 = None
        self._识别窗口标签 = None
        self._识别窗口图像 = None
    # _关闭识别窗口() 结束

    def 线程安全_更新识别窗口(self, 图像数组, 宽度, 高度, 标题文本):
        """线程安全地更新识别窗口（供工作线程调用）

        工作线程调用此方法后，会通过 root.after(0, ...) 切换到主线程执行实际更新。

        Args:
            图像数组: numpy 数组 (H, W, 3)，BGR 格式
            宽度: 图像宽度
            高度: 图像高度
            标题文本: 要显示在窗口标题栏的文本
        """
        if self.root is None:
            return
        # 包装成 lambda 避免 late binding 问题
        图像数组_副本 = 图像数组.copy() if 图像数组 is not None else None
        包装 = lambda: self._更新识别窗口内容(图像数组_副本, 宽度, 高度, 标题文本)
        self.root.after(0, 包装)
    # 线程安全_更新识别窗口() 结束

    def 线程安全_关闭识别窗口(self):
        """线程安全地关闭识别窗口（供工作线程调用）"""
        if self.root is None:
            return
        self.root.after(0, self._关闭识别窗口)
    # 线程安全_关闭识别窗口() 结束

    def 线程安全_更新状态(self, 文本):
        """线程安全地更新状态文本（供工作线程调用）"""
        if self.root is None:
            return
        self.root.after(0, lambda: self._update_status(文本))
    # 线程安全_更新状态() 结束

    # ============================================================
    # 共享识别窗口方法结束
    # ============================================================

    def _on_closing(self):
        """窗口关闭回调"""
        # 如果摄像头窗口还开着，先关闭它
        if getattr(self, "_摄像头窗口", None) is not None:
            try:
                if self._摄像头窗口.winfo_exists():
                    if getattr(self, "_摄像头刷新任务_id", None) is not None:
                        self._摄像头窗口.after_cancel(self._摄像头刷新任务_id)
                    self._摄像头窗口.destroy()
            except Exception:
                pass
            self._摄像头窗口 = None
        if self.is_moving:
            if not messagebox.askyesno("确认", "机械臂正在运动中，确定要关闭吗？"):
                return
        self._停止指示灯更新()
        self.root.quit()

    def create_widgets(self):
        """创建窗口部件"""
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.pack(fill="both", expand=True)

        title_label = ttk.Label(
            main_frame, text="机械臂位姿控制", font=("Microsoft YaHei", 14, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 15))

        pose_frame = ttk.LabelFrame(main_frame, text="位置 (单位: 米)", padding="10")
        pose_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        self.entries["x"] = self._create_entry(pose_frame, "X:", 0, self.DEFAULT_POSE["x"])
        self.entries["y"] = self._create_entry(pose_frame, "Y:", 1, self.DEFAULT_POSE["y"])
        self.entries["z"] = self._create_entry(pose_frame, "Z:", 2, self.DEFAULT_POSE["z"])

        attitude_frame = ttk.LabelFrame(main_frame, text="姿态 (单位: 弧度)", padding="10")
        attitude_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        self.entries["rx"] = self._create_entry(attitude_frame, "RX:", 0, self.DEFAULT_POSE["rx"])
        self.entries["ry"] = self._create_entry(attitude_frame, "RY:", 1, self.DEFAULT_POSE["ry"])
        self.entries["rz"] = self._create_entry(attitude_frame, "RZ:", 2, self.DEFAULT_POSE["rz"])

        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=15)

        btn_move = ttk.Button(
            button_frame, text="移动到位姿", command=self._on_move, width=12
        )
        btn_move.pack(side="left", padx=5)

        btn_home = ttk.Button(
            button_frame, text="回归原点", command=self._on_home, width=12
        )
        btn_home.pack(side="left", padx=5)

        self.status_label = ttk.Label(main_frame, text="状态: 就绪", foreground="green")
        self.status_label.grid(row=4, column=0, columnspan=2, pady=(0, 5))

        吸盘状态_frame = ttk.LabelFrame(main_frame, text="真空吸盘状态", padding="10")
        吸盘状态_frame.grid(row=5, column=0, columnspan=2, padx=5, pady=(5, 5), sticky="ew")

        吸盘启动_frame = ttk.Frame(吸盘状态_frame)
        吸盘启动_frame.pack(side="left", padx=20)
        ttk.Label(吸盘启动_frame, text="吸盘启动:").pack(side="left", padx=5)
        self.吸盘启动指示灯 = tk.Canvas(吸盘启动_frame, width=20, height=20)
        self.吸盘启动指示灯.pack(side="left")
        self._绘制指示灯(self.吸盘启动指示灯, False)

        抓取状态_frame = ttk.Frame(吸盘状态_frame)
        抓取状态_frame.pack(side="left", padx=20)
        ttk.Label(抓取状态_frame, text="抓取状态:").pack(side="left", padx=5)
        self.抓取状态指示灯 = tk.Canvas(抓取状态_frame, width=20, height=20)
        self.抓取状态指示灯.pack(side="left")
        self._绘制指示灯(self.抓取状态指示灯, False)

        # 添加吸盘手动控制按钮 + 摄像头显示按钮（在同一行，不增加窗口高度）
        吸盘控制_frame = ttk.LabelFrame(main_frame, text="吸盘手动控制 / 摄像头", padding="10")
        吸盘控制_frame.grid(row=6, column=0, columnspan=2, padx=5, pady=(5, 5), sticky="ew")

        btn_enable_suction = ttk.Button(
            吸盘控制_frame,
            text="开启吸盘",
            command=self._on_enable_suction,
            width=10
        )
        btn_enable_suction.pack(side="left", padx=5)

        btn_disable_suction = ttk.Button(
            吸盘控制_frame,
            text="关闭吸盘",
            command=self._on_disable_suction,
            width=10
        )
        btn_disable_suction.pack(side="left", padx=5)

        # 摄像头显示按钮（不新增行，放在同一区域右侧）
        btn_show_camera = ttk.Button(
            吸盘控制_frame,
            text="显示摄像头",
            command=self._on_show_camera,
            width=12
        )
        btn_show_camera.pack(side="left", padx=5)

        if 创建自动抓取按钮 is not None and self.物料数据 is not None:
            自动抓取区域 = 创建自动抓取按钮(
                main_frame,
                self.arm,
                self.物料数据,
                self._safe_update_status,
                gui=self
            )
            自动抓取区域.grid(row=7, column=0, columnspan=2, pady=(10, 5))

        if 创建巡航抓取按钮 is not None and self.物料数据 is not None:
            巡航抓取区域 = 创建巡航抓取按钮(
                main_frame,
                self.arm,
                self.物料数据,
                self._safe_update_status,
                gui=self  # 传入 gui 实例，供识别函数调用窗口更新接口
            )
            巡航抓取区域.grid(row=8, column=0, columnspan=2, pady=(5, 10))

        self._启动指示灯更新()

    def run(self):
        """运行GUI主循环"""
        self.root = tk.Tk()
        self.root.title("机械臂位姿控制")
        self.root.resizable(False, False)
        self.create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.mainloop()


def gui_thread(arm, shared_event, 物料数据=None):
    """GUI线程函数

    Args:
        arm: 机械臂对象
        shared_event: 线程间共享的事件标志
        物料数据: 物料位置数据对象
    """
    gui = ArmGUI(arm, shared_event, 物料数据)
    gui.run()


def start_gui(arm, shared_event=None, 物料数据=None):
    """启动GUI线程的入口函数

    Args:
        arm: 机械臂对象
        shared_event: 线程间共享的事件标志，默认为None则创建新事件
        物料数据: 物料位置数据对象，默认为None则不启用自动抓取功能

    Returns:
        threading.Thread: GUI线程对象
    """
    if shared_event is None:
        shared_event = threading.Event()
    gui_t = threading.Thread(target=gui_thread, args=(arm, shared_event, 物料数据), daemon=True)
    gui_t.start()
    return gui_t
# 机器臂控制窗口.py 结束

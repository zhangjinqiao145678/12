# -*-coding:utf-8 -*-
"""物料识别器.py

物料识别模块 - 基于轮廓检测的物料识别算法
支持9种物料类型的识别
在图像上标记识别结果
"""

import cv2
import numpy as np
from typing import Optional, List, Dict, Any
from PIL import Image, ImageDraw, ImageFont


# 物料类型配置表
物料类型配置表 = {
    "Cuboid": {
        "对称重数": 2,
        "模板文件": "cuboid_template.png",
        "旋转增量": 180,
        "颜色": (0, 0, 255),
        "颜色范围": [(0, 150, 80), (10, 255, 255)]
    },
    "Cube": {
        "对称重数": 2,
        "模板文件": "cube_template.png",
        "旋转增量": 180,
        "颜色": (0, 165, 255),
        "颜色范围": [(11, 100, 100), (25, 255, 255)]
    },
    "Cruciform": {
        "对称重数": 4,
        "模板文件": "cruciform_template.png",
        "旋转增量": 90,
        "颜色": (255, 0, 255),
        "颜色范围": [(125, 100, 100), (145, 255, 255)]
    },
    "FivePointed": {
        "对称重数": 5,
        "模板文件": "fivepointed_template.png",
        "旋转增量": 72,
        "颜色": (255, 0, 155),
        "颜色范围": [(160, 100, 100), (175, 255, 255)]
    },
    "Triangle": {
        "对称重数": 3,
        "模板文件": "triangle_template.png",
        "旋转增量": 120,
        "颜色": (255, 100, 180),
        "颜色范围": [(0, 40, 150), (10, 150, 255)]
    },
    "Pentagonal": {
        "对称重数": 5,
        "模板文件": "pentagonal_template.png",
        "旋转增量": 72,
        "颜色": (0, 255, 0),
        "颜色范围": [(40, 100, 100), (80, 255, 255)]
    },
    "Parallelogram": {
        "对称重数": 2,
        "模板文件": "parallelogram_template.png",
        "旋转增量": 180,
        "颜色": (255, 0, 0),
        "颜色范围": [(100, 100, 100), (120, 255, 255)]
    },
    "Quincunx": {
        "对称重数": 4,
        "模板文件": "quincunx_template.png",
        "旋转增量": 90,
        "颜色": (255, 255, 0),
        "颜色范围": [(80, 100, 100), (95, 255, 255)]
    },
    "Cylindrical": {
        "对称重数": 1,
        "模板文件": "cylindrical_template.png",
        "旋转增量": 360,
        "颜色": (0, 255, 255),
        "颜色范围": [(25, 100, 100), (35, 255, 255)]
    }
}

# 物料中文名称映射表
物料中文名称表 = {
    "Cuboid": "长方形",
    "Cube": "正方形",
    "Cruciform": "十字形",
    "FivePointed": "五角星",
    "Triangle": "三角形",
    "Pentagonal": "五边形",
    "Parallelogram": "平行四边形",
    "Quincunx": "梅花形",
    "Cylindrical": "圆柱体"
}


def _绘制中文文字(图像, 文本, 位置, 颜色, 字体大小=20):
    """在图像上绘制中文文字（使用PIL，因为OpenCV不支持中文）

    Args:
        图像: BGR格式的numpy数组图像
        文本: 要绘制的中文文本
        位置: (x, y) 文字左上角坐标
        颜色: (B, G, R) 颜色元组
        字体大小: 字体大小，默认20

    Returns:
        np.ndarray: 绘制文字后的BGR图像
    """
    x, y = 位置
    b, g, r = 颜色

    img_pil = Image.fromarray(cv2.cvtColor(图像, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)

    try:
        font = ImageFont.truetype("simhei.ttf", 字体大小, encoding="utf-8")
    except Exception:
        try:
            font = ImageFont.truetype("SimHei.ttf", 字体大小, encoding="utf-8")
        except Exception:
            try:
                font = ImageFont.truetype("C:/Windows/Fonts/simhei.ttf", 字体大小, encoding="utf-8")
            except Exception:
                font = ImageFont.load_default()

    draw.text((x, y), 文本, font=font, fill=(r, g, b))

    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)


class 物料轮廓识别器:
    """基于轮廓检测的物料识别器

    使用颜色过滤 + 轮廓检测 + 形状特征匹配来识别物料
    在图像上绘制识别结果标记
    """

    def __init__(self, 物料类型: str = "Cube"):
        """初始化物料轮廓识别器

        Args:
            物料类型: 物料类型名称（如 "Cube", "Cuboid", "Cruciform" 等）
        """
        self.物料类型 = 物料类型
        self.颜色范围 = 物料类型配置表.get(物料类型, {}).get("颜色范围", [(0, 0, 0), (255, 255, 255)])
        self.最小面积 = 500
        self.对称重数 = 物料类型配置表.get(物料类型, {}).get("对称重数", 1)

    def _颜色过滤(self, 图像: np.ndarray) -> np.ndarray:
        """对图像进行颜色过滤，生成颜色掩码

        Args:
            图像: BGR格式图像

        Returns:
            np.ndarray: 二值掩码图像
        """
        hsv = cv2.cvtColor(图像, cv2.COLOR_BGR2HSV)
        lower = np.array(self.颜色范围[0], dtype=np.uint8)
        upper = np.array(self.颜色范围[1], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def _计算旋转角度(self, 轮廓: np.ndarray) -> float:
        """计算轮廓的旋转角度

        Args:
            轮廓: OpenCV轮廓

        Returns:
            float: 旋转角度（度）
        """
        if self.物料类型 == "Cylindrical":
            return 0.0

        if self.物料类型 in ["Cruciform", "Quincunx"]:
            return self._PCA角度(轮廓)

        if self.物料类型 in ["Cuboid", "Cube", "Parallelogram", "Triangle", "Pentagonal", "FivePointed"]:
            return self._轮廓矩角度(轮廓)

        return self._轮廓矩角度(轮廓)

    def _PCA角度(self, 轮廓: np.ndarray) -> float:
        """使用PCA主成分分析计算角度

        Args:
            轮廓: OpenCV轮廓

        Returns:
            float: 旋转角度（度）
        """
        点集 = 轮廓.reshape(-1, 2).astype(np.float32)
        if len(点集) < 2:
            return self._minAreaRect角度(轮廓)

        均值 = np.mean(点集, axis=0)
        去中心化点集 = 点集 - 均值
        协方差矩阵 = np.dot(去中心化点集.T, 去中心化点集) / (len(去中心化点集) - 1)
        特征值, 特征向量 = np.linalg.eig(协方差矩阵)
        最大特征值索引 = np.argmax(特征值)
        主方向向量 = 特征向量[:, 最大特征值索引]
        角度 = np.degrees(np.arctan2(主方向向量[1], 主方向向量[0]))

        角度 = 角度 % 360
        if 角度 >= 180:
            角度 = 角度 - 360
        if abs(角度) > 90:
            角度 = 角度 - 180 if 角度 > 0 else 角度 + 180
        if 角度 < 0:
            角度 = 角度 + 180

        return 角度

    def _轮廓矩角度(self, 轮廓: np.ndarray) -> float:
        """使用轮廓二阶矩计算角度

        Args:
            轮廓: OpenCV轮廓

        Returns:
            float: 旋转角度（度）
        """
        M = cv2.moments(轮廓)
        if M['m00'] == 0:
            return 0.0

        mu20 = M['m20'] / M['m00'] - (M['m10'] / M['m00']) ** 2
        mu11 = M['m11'] / M['m00'] - (M['m10'] / M['m00']) * (M['m01'] / M['m00'])
        mu02 = M['m02'] / M['m00'] - (M['m01'] / M['m00']) ** 2

        角度 = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
        角度 = np.degrees(角度)

        角度 = 角度 % 180
        if 角度 < 0:
            角度 = 角度 + 180

        if abs(角度 - 180) < 1.0:
            角度 = 0.0

        return 角度

    def _minAreaRect角度(self, 轮廓: np.ndarray) -> float:
        """使用minAreaRect计算角度

        Args:
            轮廓: OpenCV轮廓

        Returns:
            float: 旋转角度（度）
        """
        rect = cv2.minAreaRect(轮廓)
        角度 = rect[2]
        if 角度 < -45:
            角度 = 90 + 角度
        return 角度 % 360

    def 识别(self, 图像: np.ndarray) -> List[Dict[str, Any]]:
        """识别图像中的物料

        Args:
            图像: BGR格式图像

        Returns:
            List[Dict]: 识别结果列表，每个结果包含：
                - '中心点': (x, y) 像素坐标
                - '角度': 旋转角度（度）
                - '物料类型': 物料类型名称
                - '面积': 轮廓面积
                - '外接矩形': (宽度, 高度, 角度)
                - '置信度': float 置信度
        """
        mask = self._颜色过滤(图像)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        识别结果 = []
        for contour in contours:
            面积 = cv2.contourArea(contour)
            if 面积 < self.最小面积:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            中心点 = (x + w // 2, y + h // 2)

            角度 = self._计算旋转角度(contour)

            识别结果.append({
                '中心点': 中心点,
                '角度': 角度,
                '物料类型': self.物料类型,
                '面积': 面积,
                '外接矩形': (w, h, 角度),
                '置信度': 1.0
            })

        return 识别结果

    def 在图像上标记(self, 图像: np.ndarray, 识别结果: List[Dict[str, Any]] = None, 
                     颜色: tuple = None) -> np.ndarray:
        """在图像上标记识别结果

        Args:
            图像: BGR格式图像（会被修改）
            识别结果: 识别结果列表，如果为None则先执行识别
            颜色: 标记颜色，默认使用物料类型配置的颜色

        Returns:
            np.ndarray: 标记后的图像
        """
        if 识别结果 is None:
            识别结果 = self.识别(图像)

        if 颜色 is None:
            颜色 = 物料类型配置表.get(self.物料类型, {}).get("颜色", (0, 255, 0))

        标记图像 = 图像.copy()

        for 结果 in 识别结果:
            x, y = 结果['中心点']

            cv2.circle(标记图像, (x, y), 8, 颜色, -1)
            cv2.drawMarker(标记图像, (x, y), 颜色, cv2.MARKER_CROSS, 25, 2)

            角度 = 结果.get('角度', 0.0)
            物料类型 = 结果.get('物料类型', self.物料类型)
            文本 = f"{物料类型} {角度:.1f}°"
            cv2.putText(标记图像, 文本, (x + 15, y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 颜色, 1)

        return 标记图像


class 多物料轮廓识别器:
    """多物料轮廓识别器

    支持同时识别多种物料类型
    """

    def __init__(self):
        """初始化多物料轮廓识别器"""
        self.物料类型列表 = list(物料类型配置表.keys())

    def 识别所有物料(
        self,
        图像: np.ndarray,
        阈值: float = 0.7,
        仅识别类型: Optional[List[str]] = None
    ) -> Dict[str, List[Dict[str, Any]]]:
        """识别图像中的所有物料

        Args:
            图像: BGR格式图像
            阈值: 匹配置信度阈值（当前不使用，仅为兼容性保留）
            仅识别类型: 可选，指定只识别特定类型的物料列表

        Returns:
            Dict[str, List[Dict]]: 识别结果字典，键为物料类型，值为识别结果列表
        """
        所有结果 = {}

        目标类型 = 仅识别类型 if 仅识别类型 is not None else self.物料类型列表

        for 物料类型 in 目标类型:
            if 物料类型 not in self.物料类型列表:
                continue
            识别器 = 物料轮廓识别器(物料类型)
            结果 = 识别器.识别(图像)
            if 结果:
                所有结果[物料类型] = 结果

        return 所有结果

    def 在图像上标记所有物料(
        self,
        图像: np.ndarray,
        识别结果: Dict[str, List[Dict[str, Any]]] = None,
        显示文字: bool = True
    ) -> np.ndarray:
        """在图像上标记所有识别结果

        Args:
            图像: BGR格式图像
            识别结果: 识别结果字典，如果为None则先执行识别
            显示文字: 是否显示物料类型和角度文字（使用中文名称）

        Returns:
            np.ndarray: 标记后的图像
        """
        标记图像 = 图像.copy()

        if 识别结果 is None:
            识别结果 = self.识别所有物料(图像)

        物料数量 = 0

        for 物料类型, 结果列表 in 识别结果.items():
            颜色 = 物料类型配置表.get(物料类型, {}).get("颜色", (0, 255, 0))
            中文名称 = 物料中文名称表.get(物料类型, 物料类型)

            for 结果 in 结果列表:
                x, y = 结果['中心点']
                物料数量 += 1

                cv2.circle(标记图像, (x, y), 8, 颜色, -1)
                cv2.drawMarker(标记图像, (x, y), 颜色, cv2.MARKER_CROSS, 25, 2)

                if 显示文字:
                    文本 = f"{物料数量}:{中文名称}"
                    标记图像 = _绘制中文文字(标记图像, 文本, (x + 15, y - 30), 颜色, 字体大小=18)

        if 显示文字 and 物料数量 > 0:
            标题文本 = f"共识别到 {物料数量} 个物料"
            标记图像 = _绘制中文文字(标记图像, 标题文本, (10, 10), (0, 255, 0), 字体大小=24)

        return 标记图像


# 物料识别器.py 结束

# -*-coding:utf-8 -*-
"""托盘识别器.py

托盘缺口识别模块 - 支持托盘旋转识别
在图像上标记识别结果
"""

import cv2
import numpy as np
import math
from typing import Optional, List, Dict, Tuple, Any


# 托盘标准布局（3x3网格）
托盘标准布局 = {
    (0, 0): ("Quincunx", 0.0),      # 梅花
    (1, 0): ("Parallelogram", 0.0),  # 平行四边形
    (2, 0): ("Cylindrical", 0.0),    # 圆柱
    (0, 1): ("FivePointed", 0.0),    # 五角星
    (1, 1): ("Pentagonal", 0.0),     # 五边形
    (2, 1): ("Triangle", 0.0),       # 三角形
    (0, 2): ("Cruciform", 0.0),      # 十字形
    (1, 2): ("Cuboid", 0.0),         # 长方体
    (2, 2): ("Cube", 0.0)            # 正方体
}

# 形状到网格位置的映射（无旋转）
形状到位置 = {
    'Quincunx': (0, 0),      # 梅花
    'Parallelogram': (1, 0),  # 平行四边形
    'Cylindrical': (2, 0),    # 圆柱
    'FivePointed': (0, 1),    # 五角星
    'Pentagonal': (1, 1),     # 五边形
    'Triangle': (2, 1),       # 三角形
    'Cruciform': (0, 2),      # 十字形
    'Cuboid': (1, 2),         # 长方体
    'Cube': (2, 2)            # 正方体
}

# 位置到形状的映射（无旋转）
位置到形状 = {v: k for k, v in 形状到位置.items()}


class 托盘缺口识别器:
    """托盘缺口识别器（支持旋转）

    使用颜色过滤 + 轮廓检测 + 形状特征匹配来识别托盘缺口
    在图像上绘制识别结果标记
    """

    def __init__(self):
        """初始化托盘缺口识别器"""
        self.最小面积 = 400
        self.最大面积 = 35000
        self.输出日志 = False

    def _黑色过滤(self, 图像: np.ndarray) -> np.ndarray:
        """对图像进行黑色区域过滤，生成掩码

        Args:
            图像: BGR格式图像

        Returns:
            np.ndarray: 二值掩码图像
        """
        hsv = cv2.cvtColor(图像, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 60], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def _计算轮廓特征(self, 轮廓: np.ndarray) -> Dict[str, Any]:
        """计算轮廓的形状特征

        Args:
            轮廓: OpenCV轮廓

        Returns:
            Dict: 包含面积、圆形度、矩形度、角点数、凸包缺陷数、角度、中心点等特征
        """
        面积 = cv2.contourArea(轮廓)
        周长 = cv2.arcLength(轮廓, True)

        圆形度 = 4 * np.pi * 面积 / (周长 * 周长) if 周长 > 0 else 0

        rect = cv2.minAreaRect(轮廓)
        矩形面积 = rect[1][0] * rect[1][1] if rect[1][0] > 0 and rect[1][1] > 0 else 1
        矩形度 = 面积 / 矩形面积

        approx = cv2.approxPolyDP(轮廓, 0.04 * cv2.arcLength(轮廓, True), True)
        角点数 = len(approx)

        hull = cv2.convexHull(轮廓, returnPoints=False)
        defects = cv2.convexityDefects(轮廓, hull)
        凸包缺陷数 = len(defects) if defects is not None else 0

        角度 = rect[2]
        if 角度 < -45:
            角度 = 90 + 角度

        M = cv2.moments(轮廓)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            x, y, w, h = cv2.boundingRect(轮廓)
            cx = x + w // 2
            cy = y + h // 2

        return {
            '面积': 面积,
            '圆形度': 圆形度,
            '矩形度': 矩形度,
            '角点数': 角点数,
            '凸包缺陷数': 凸包缺陷数,
            '角度': 角度,
            '中心点': (cx, cy),
            '轮廓': 轮廓
        }

    def _识别圆形(self, 特征: Dict) -> bool:
        """判断是否为圆形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为圆形
        """
        has_high_roundness = 特征['圆形度'] > 0.85
        has_low_rectangularity = 特征['矩形度'] < 0.9
        has_moderate_corners = 6 <= 特征['角点数'] <= 12
        has_large_area = 特征['面积'] > 3000
        return has_high_roundness and has_low_rectangularity and has_moderate_corners and has_large_area

    def _识别十字形(self, 特征: Dict) -> bool:
        """判断是否为十字形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为十字形
        """
        has_many_defects = 特征['凸包缺陷数'] >= 4
        has_low_rectangularity = 特征['矩形度'] < 0.65
        has_many_corners = 特征['角点数'] >= 6
        return has_many_defects and has_low_rectangularity and has_many_corners

    def _识别五角星(self, 特征: Dict) -> bool:
        """判断是否为五角星

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为五角星
        """
        return 4 <= 特征['凸包缺陷数'] <= 9 and 5 <= 特征['角点数'] <= 12 and 0.45 < 特征['圆形度'] < 0.85

    def _识别三角形(self, 特征: Dict) -> bool:
        """判断是否为三角形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为三角形
        """
        return 3 <= 特征['角点数'] <= 5 and 特征['圆形度'] < 0.75

    def _识别正方形(self, 特征: Dict) -> bool:
        """判断是否为正方形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为正方形
        """
        return 特征['矩形度'] > 0.85 and 特征['角点数'] >= 4

    def _识别长方体(self, 特征: Dict) -> bool:
        """判断是否为长方体

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为长方体
        """
        rect = cv2.minAreaRect(特征['轮廓'])
        width, height = rect[1]
        宽高比 = max(width, height) / min(width, height) if min(width, height) > 0 else 1.0
        return 宽高比 > 1.3 and 特征['矩形度'] > 0.7

    def _识别五边形(self, 特征: Dict) -> bool:
        """判断是否为五边形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为五边形
        """
        return 5 <= 特征['角点数'] <= 7 and 0.7 < 特征['圆形度'] < 0.98

    def _识别梅花形(self, 特征: Dict) -> bool:
        """判断是否为梅花形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为梅花形
        """
        has_many_defects = 特征['凸包缺陷数'] >= 4
        has_many_corners = 特征['角点数'] >= 6
        has_low_rectangularity = 特征['矩形度'] < 0.7
        has_roundness = 特征['圆形度'] > 0.5
        return has_many_defects and has_many_corners and has_low_rectangularity and has_roundness

    def _识别平行四边形(self, 特征: Dict) -> bool:
        """判断是否为平行四边形

        Args:
            特征: 轮廓特征字典

        Returns:
            bool: 是否为平行四边形
        """
        rect = cv2.minAreaRect(特征['轮廓'])
        width, height = rect[1]
        宽高比 = max(width, height) / min(width, height) if min(width, height) > 0 else 1.0
        return 特征['角点数'] == 4 and 0.6 < 特征['矩形度'] < 0.95 and 宽高比 < 1.4

    def _识别形状(self, 特征: Dict) -> Tuple[str, float]:
        """识别形状类型（按独特性排序）

        Args:
            特征: 轮廓特征字典

        Returns:
            Tuple[str, float]: (形状类型名称, 置信度)
        """
        if self._识别圆形(特征):
            return ("Cylindrical", 0.95)
        if self._识别梅花形(特征):
            return ("Quincunx", 0.9)
        if self._识别十字形(特征):
            return ("Cruciform", 0.92)
        if self._识别五角星(特征):
            return ("FivePointed", 0.88)
        if self._识别五边形(特征):
            return ("Pentagonal", 0.9)
        if self._识别三角形(特征):
            return ("Triangle", 0.88)
        if self._识别平行四边形(特征):
            return ("Parallelogram", 0.85)
        if self._识别长方体(特征):
            return ("Cuboid", 0.88)
        if self._识别正方形(特征):
            return ("Cube", 0.88)
        return ("Unknown", 0.0)

    def _确定旋转角度(self, 候选特征: List[Dict]) -> int:
        """根据参考点确定托盘旋转角度

        Args:
            候选特征: 候选轮廓特征列表

        Returns:
            int: 旋转角度（0, 90, 180, 270）
        """
        圆形候选 = []
        十字候选 = []

        for 特征 in 候选特征:
            形状类型, 置信度 = self._识别形状(特征)
            if 形状类型 == "Cylindrical":
                圆形候选.append((置信度, 特征))
            elif 形状类型 == "Cruciform":
                十字候选.append((置信度, 特征))

        圆形特征 = None
        十字形特征 = None

        if 圆形候选:
            圆形候选.sort(key=lambda x: x[0], reverse=True)
            圆形特征 = 圆形候选[0][1]
        if 十字候选:
            十字候选.sort(key=lambda x: x[0], reverse=True)
            十字形特征 = 十字候选[0][1]

        if 圆形特征 is None or 十字形特征 is None:
            if self.输出日志:
                print(f"[托盘识别器] 无法找到参考点，使用几何方法推断")
            return self._通过几何特征推断旋转(候选特征)

        return self._通过参考点计算旋转(圆形特征, 十字形特征)

    def _通过参考点计算旋转(self, 圆形特征: Dict, 十字形特征: Dict) -> int:
        """通过圆形和十字形的相对位置计算旋转角度

        Args:
            圆形特征: 圆形轮廓特征
            十字形特征: 十字形轮廓特征

        Returns:
            int: 旋转角度（0, 90, 180, 270）
        """
        圆形中心 = 圆形特征['中心点']
        十字中心 = 十字形特征['中心点']

        dx = 十字中心[0] - 圆形中心[0]
        dy = 十字中心[1] - 圆形中心[1]

        tolerance = 30

        if abs(dx) < tolerance and abs(dy) < tolerance:
            return 0

        if dx > tolerance and dy > tolerance:
            return 0
        elif dx > tolerance and dy < -tolerance:
            return 90
        elif dx < -tolerance and dy < -tolerance:
            return 180
        elif dx < -tolerance and dy > tolerance:
            return 270

        if abs(dx) > tolerance or abs(dy) > tolerance:
            angle = math.degrees(math.atan2(dy, dx))
            if angle < 0:
                angle += 360

            if 315 <= angle < 360 or 0 <= angle < 45:
                return 0
            elif 45 <= angle < 135:
                return 90    # 修正：原来返回270，应该返回90
            elif 135 <= angle < 225:
                return 180
            elif 225 <= angle < 315:
                return 270   # 修正：原来返回90，应该返回270

        return 0

    def _通过几何特征推断旋转(self, 候选特征: List[Dict]) -> int:
        """当参考点识别失败时，通过几何特征推断旋转角度

        Args:
            候选特征: 候选轮廓特征列表

        Returns:
            int: 旋转角度（0, 90, 180, 270）
        """
        if len(候选特征) < 3:
            return 0

        points = [f['中心点'] for f in 候选特征]
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        width = max_x - min_x
        height = max_y - min_y

        if width < 50 or height < 50:
            return 0

        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2

        最小面积特征 = min(候选特征, key=lambda f: f['面积'])
        min_x_pos = 最小面积特征['中心点'][0]
        min_y_pos = 最小面积特征['中心点'][1]

        if min_x_pos < center_x and min_y_pos < center_y:
            return 0
        elif min_x_pos >= center_x and min_y_pos < center_y:
            return 270
        elif min_x_pos < center_x and min_y_pos >= center_y:
            return 90
        else:
            return 180

    def _获取旋转后的位置形状映射(self, 旋转角度: int) -> Dict[Tuple[int, int], str]:
        """获取旋转后的网格位置到形状的映射

        Args:
            旋转角度: 旋转角度（0, 90, 180, 270）

        Returns:
            Dict: 网格位置到形状的映射
        """
        if 旋转角度 == 0:
            return 位置到形状.copy()

        if 旋转角度 == 90:
            return {(pos[1], 2 - pos[0]): 形状 for 形状, pos in 形状到位置.items()}

        if 旋转角度 == 180:
            return {(2 - pos[0], 2 - pos[1]): 形状 for 形状, pos in 形状到位置.items()}

        if 旋转角度 == 270:
            return {(2 - pos[1], pos[0]): 形状 for 形状, pos in 形状到位置.items()}

        return 位置到形状.copy()

    def _聚类到网格(self, 候选特征: List[Dict]) -> List[Dict]:
        """使用K-means聚类将候选特征分配到3x3网格

        Args:
            候选特征: 候选轮廓特征列表

        Returns:
            List[Dict]: 聚类后的特征列表
        """
        if len(候选特征) < 4:
            return []

        points = np.array([[f['中心点'][0], f['中心点'][1]] for f in 候选特征], dtype=np.float32)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        flags = cv2.KMEANS_RANDOM_CENTERS
        compactness, labels, centers = cv2.kmeans(points, 9, None, criteria, 10, flags)

        聚类结果 = {}
        for i, 特征 in enumerate(候选特征):
            聚类标签 = labels[i][0]
            if 聚类标签 not in 聚类结果:
                聚类结果[聚类标签] = []
            聚类结果[聚类标签].append(特征)

        最终特征 = []
        for 标签 in sorted(聚类结果.keys()):
            特征列表 = 聚类结果[标签]
            最大特征 = max(特征列表, key=lambda f: f['面积'])
            最终特征.append(最大特征)

        return 最终特征

    def _映射到网格位置(self, 特征列表: List[Dict]) -> Dict[Tuple[int, int], Dict]:
        """将特征列表映射到3x3网格位置

        Args:
            特征列表: 特征列表

        Returns:
            Dict: 网格位置到特征的映射
        """
        if len(特征列表) < 4:
            return {}

        xs = [f['中心点'][0] for f in 特征列表]
        ys = [f['中心点'][1] for f in 特征列表]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        width = max_x - min_x
        height = max_y - min_y

        if width < 50 or height < 50:
            return {}

        cell_width = width / 2.0
        cell_height = height / 2.0

        网格映射 = {}

        for 特征 in 特征列表:
            cx, cy = 特征['中心点']

            col = int(round((cx - min_x) / cell_width))
            row = int(round((cy - min_y) / cell_height))

            col = max(0, min(2, col))
            row = max(0, min(2, row))

            if (col, row) in 网格映射:
                if 特征['面积'] > 网格映射[(col, row)]['面积']:
                    网格映射[(col, row)] = 特征
            else:
                网格映射[(col, row)] = 特征

        return 网格映射

    def 识别(self, 图像: np.ndarray) -> List[Dict[str, Any]]:
        """识别图像中的托盘缺口（支持托盘旋转）

        Args:
            图像: BGR格式图像

        Returns:
            List[Dict]: 识别结果列表
        """
        # [预处理] 截取图像中间8/10区域，去掉左右各1/10以减少干扰
        高度, 宽度 = 图像.shape[:2]
        截取宽度 = 宽度 // 10  # 每侧截取1/10
        截取图像 = 图像[:, 截取宽度:宽度 - 截取宽度]

        mask = self._黑色过滤(截取图像)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        候选特征 = []

        for contour in contours:
            面积 = cv2.contourArea(contour)
            if 面积 < self.最小面积 or 面积 > self.最大面积:
                continue

            特征 = self._计算轮廓特征(contour)
            候选特征.append(特征)

        if len(候选特征) < 4:
            return []

        聚类特征 = self._聚类到网格(候选特征)
        旋转角度 = self._确定旋转角度(聚类特征)
        位置到旋转后形状 = self._获取旋转后的位置形状映射(旋转角度)
        网格映射 = self._映射到网格位置(聚类特征)

        最终结果 = []

        for (col, row), 特征 in 网格映射.items():
            期望形状 = 位置到旋转后形状.get((col, row), "Unknown")
            实际形状, 置信度 = self._识别形状(特征)

            最终形状 = 期望形状 if 期望形状 != "Unknown" else 实际形状

            # 将截取后图像的中心点坐标转换回原始图像坐标
            原始中心点 = (特征['中心点'][0] + 截取宽度, 特征['中心点'][1])

            最终结果.append({
                '中心点': 原始中心点,
                '角度': 特征['角度'],
                '形状类型': 最终形状,
                '网格位置': (col, row),
                '置信度': 置信度,
                '推算来源': '布局推算' if 期望形状 != "Unknown" else '实际识别',
                '旋转角度': 旋转角度,
                '实际识别形状': 实际形状
            })

        最终结果.sort(key=lambda r: (r['网格位置'][1], r['网格位置'][0]))

        return 最终结果

    def 在图像上标记(self, 图像: np.ndarray, 识别结果: List[Dict[str, Any]] = None,
                    显示文字: bool = True) -> np.ndarray:
        """在图像上标记托盘缺口识别结果

        Args:
            图像: BGR格式图像（会被修改）
            识别结果: 识别结果列表，如果为None则先执行识别
            显示文字: 是否显示缺口类型和网格位置文字

        Returns:
            np.ndarray: 标记后的图像
        """
        if 识别结果 is None:
            识别结果 = self.识别(图像)

        标记图像 = 图像.copy()
        高度, 宽度 = 标记图像.shape[:2]

        # 绘制识别范围边界线（左右各1/10区域不参与识别）
        边界宽度 = 宽度 // 10
        左边界 = 边界宽度
        右边界 = 宽度 - 边界宽度

        # 绘制左边界线（红色）
        cv2.line(标记图像, (左边界, 0), (左边界, 高度), (0, 0, 255), 2, cv2.LINE_AA)
        # 绘制右边界线（红色）
        cv2.line(标记图像, (右边界, 0), (右边界, 高度), (0, 0, 255), 2, cv2.LINE_AA)

        缺口颜色映射 = {
            "Cylindrical": (0, 255, 255),
            "Triangle": (255, 100, 180),
            "Cube": (0, 165, 255),
            "Parallelogram": (255, 0, 0),
            "Pentagonal": (0, 255, 0),
            "Cuboid": (0, 0, 255),
            "Quincunx": (255, 255, 0),
            "FivePointed": (255, 0, 155),
            "Cruciform": (255, 0, 255)
        }

        缺口数量 = 0

        for 结果 in 识别结果:
            x, y = 结果['中心点']
            颜色 = 缺口颜色映射.get(结果['形状类型'], (255, 255, 255))
            缺口数量 += 1

            # 只显示标记点（圆圈+十字），不显示类型名称
            cv2.circle(标记图像, (x, y), 10, 颜色, -1)
            cv2.drawMarker(标记图像, (x, y), 颜色, cv2.MARKER_CROSS, 25, 2)

        return 标记图像


class 多缺口识别器:
    """多缺口识别器

    支持同时识别多个托盘缺口
    """

    def __init__(self):
        """初始化多缺口识别器"""
        self.识别器 = 托盘缺口识别器()

    def 识别所有缺口(
        self,
        图像: np.ndarray,
        阈值: float = 0.5,
        仅识别类型: Optional[List[str]] = None
    ) -> Dict[str, List[Dict[str, Any]]]:
        """识别图像中的所有托盘缺口

        Args:
            图像: BGR格式图像
            阈值: 置信度阈值
            仅识别类型: 可选，指定只识别特定类型

        Returns:
            Dict[str, List[Dict]]: 按缺口类型分组的识别结果
        """
        所有结果 = {}
        结果 = self.识别器.识别(图像)

        for 单个结果 in 结果:
            缺口类型 = 单个结果['形状类型']

            if 仅识别类型 is not None and 缺口类型 not in 仅识别类型:
                continue

            if 单个结果['置信度'] < 阈值:
                continue

            if 缺口类型 not in 所有结果:
                所有结果[缺口类型] = []
            所有结果[缺口类型].append(单个结果)

        return 所有结果


# 托盘识别器.py 结束

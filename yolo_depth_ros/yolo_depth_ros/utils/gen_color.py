
import cv2
import numpy as np
from typing import Tuple


def generate_color(index: int) -> Tuple[int, int, int]:
    """
    index를 바탕으로 HSV 색을 생성한 뒤 BGR로 변환해 반환합니다.
    Hue 범위(0~179)를 일정 간격(index*30)으로 증가시키고,
    Saturation/Value는 고정된 값으로 지정합니다.
    """
    hue = (index * 30) % 180
    sat = 200
    val = 200
    color_hsv = np.uint8([[[hue, sat, val]]])
    color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(color_bgr[0]), int(color_bgr[1]), int(color_bgr[2]))
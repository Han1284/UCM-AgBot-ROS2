#!/usr/bin/env python3
"""Generate the 4 m x 4 m metric calibration-floor texture."""

from pathlib import Path

import cv2
import numpy as np


SIZE = 2000
CENTER = SIZE // 2
PIXELS_PER_METRE = 500
MINOR_STEP = 50   # 0.1 m
MAJOR_STEP = 250  # 0.5 m


def put_label(image, text, position, color=(45, 45, 45), scale=0.65):
    cv2.putText(
        image,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


def main():
    image = np.full((SIZE, SIZE, 3), 238, dtype=np.uint8)

    for pixel in range(0, SIZE + 1, MINOR_STEP):
        cv2.line(image, (pixel, 0), (pixel, SIZE - 1), (205, 205, 205), 1)
        cv2.line(image, (0, pixel), (SIZE - 1, pixel), (205, 205, 205), 1)

    for pixel in range(0, SIZE + 1, MAJOR_STEP):
        cv2.line(image, (pixel, 0), (pixel, SIZE - 1), (125, 125, 125), 3)
        cv2.line(image, (0, pixel), (SIZE - 1, pixel), (125, 125, 125), 3)

    x_color = (35, 45, 210)
    y_color = (45, 150, 45)
    cv2.line(image, (0, CENTER), (SIZE - 1, CENTER), x_color, 7)
    cv2.line(image, (CENTER, SIZE - 1), (CENTER, 0), y_color, 7)
    cv2.arrowedLine(image, (SIZE - 150, CENTER), (SIZE - 15, CENTER), x_color, 12, tipLength=0.35)
    cv2.arrowedLine(image, (CENTER, 150), (CENTER, 15), y_color, 12, tipLength=0.35)

    for index in range(-4, 5):
        value = index * 0.5
        x_pixel = CENTER + index * MAJOR_STEP
        y_pixel = CENTER - index * MAJOR_STEP
        if index != 0:
            x_label = max(8, min(SIZE - 62, x_pixel - 28))
            y_label = max(30, min(SIZE - 12, y_pixel + 8))
            put_label(image, f"{value:+.1f}", (x_label, CENTER + 42))
            put_label(image, f"{value:+.1f}", (CENTER + 12, y_label))

    cv2.circle(image, (CENTER, CENTER), 18, (20, 20, 20), 4)
    put_label(image, "ORIGIN (0,0)", (CENTER + 25, CENTER - 24), scale=0.8)
    put_label(image, "+X", (SIZE - 95, CENTER - 24), color=x_color, scale=0.9)
    put_label(image, "+Y", (CENTER + 22, 55), color=y_color, scale=0.9)
    put_label(image, "0.1 m GRID / 0.5 m MAJOR", (35, 55), scale=0.85)
    cv2.rectangle(image, (2, 2), (SIZE - 3, SIZE - 3), (30, 30, 30), 6)

    output = Path(__file__).parent / "materials" / "textures" / "calibration_floor.png"
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"Failed to write {output}")
    print(output)


if __name__ == "__main__":
    main()

import argparse
import glob
import json
import math
import os
import random
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERA_TOPIC = '/perception_test_camera/color/image_raw'
SET_POSE_SERVICE = '/world/leaf_bench/set_pose'
PLANT_POSITION = (0.85, 0.0, 0.0)
HIDDEN_POSITION = (0.85, 0.0, -10.0)
TARGET_POSITION = np.array((0.85, 0.0, 0.30), dtype=np.float64)

# BGR values produced by the four emissive label meshes.
LABEL_COLORS = np.array(
    (
        (255, 0, 255),
        (255, 255, 0),
        (0, 255, 255),
        (255, 0, 0),
    ),
    dtype=np.int16,
)


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def set_entity_pose(name, position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    x, y, z = position
    qx, qy, qz, qw = quaternion
    request = (
        f'name: "{name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )
    command = [
        'ign',
        'service',
        '-s',
        SET_POSE_SERVICE,
        '--reqtype',
        'ignition.msgs.Pose',
        '--reptype',
        'ignition.msgs.Boolean',
        '--timeout',
        '3000',
        '--req',
        request,
    ]
    last_output = ''
    for _ in range(5):
        result = subprocess.run(
            command, check=False, capture_output=True, text=True
        )
        last_output = (result.stdout + result.stderr).strip()
        if result.returncode == 0 and 'true' in result.stdout.lower():
            return
        time.sleep(0.3)
    raise RuntimeError(
        f'Gazebo rejected pose update for {name}: {last_output}'
    )


class FrameReceiver(Node):
    def __init__(self):
        super().__init__('sim_leaf_dataset_collector')
        self.bridge = CvBridge()
        self.frame = None
        self.sequence = 0
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.sequence += 1

    def next_frame(self, previous_sequence, timeout=15.0, skip=1):
        deadline = time.monotonic() + timeout
        target_sequence = previous_sequence + skip + 1
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.sequence >= target_sequence:
                return self.frame.copy(), self.sequence
        raise TimeoutError(f'No new camera frame received from {CAMERA_TOPIC}')


def sample_camera_pose(rng):
    azimuth = rng.uniform(-math.pi, math.pi)
    elevation = math.radians(rng.uniform(52.0, 88.0))
    distance = rng.uniform(0.72, 0.98)

    position = TARGET_POSITION + distance * np.array(
        (
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            math.sin(elevation),
        )
    )
    forward = TARGET_POSITION - position
    yaw = math.atan2(forward[1], forward[0])
    pitch = math.atan2(
        -forward[2], math.hypot(forward[0], forward[1])
    )
    roll = math.radians(rng.uniform(-8.0, 8.0))
    quaternion = quaternion_from_rpy(roll, pitch, yaw)
    return position, quaternion, (roll, pitch, yaw)


def extract_instance_polygons(normal_image, label_image):
    difference = np.max(
        np.abs(label_image.astype(np.int16) - normal_image.astype(np.int16)),
        axis=2,
    )
    pixels = label_image.astype(np.int16)
    height, width = label_image.shape[:2]
    polygons = []
    masks = []

    for color in LABEL_COLORS:
        distance = np.linalg.norm(pixels - color, axis=2)
        mask = np.logical_and(distance < 70.0, difference > 30)
        mask = (mask.astype(np.uint8) * 255)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 120.0:
            continue

        epsilon = 0.002 * cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if len(polygon) < 3:
            continue
        normalized = []
        for x, y in polygon:
            normalized.extend((x / width, y / height))
        polygons.append(normalized)
        masks.append(mask)

    return polygons, masks


def write_sample(output_dir, split, index, image, polygons):
    image_dir = os.path.join(output_dir, 'images', split)
    label_dir = os.path.join(output_dir, 'labels', split)
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(label_dir, exist_ok=True)

    stem = f'leaf_{index:04d}'
    cv2.imwrite(os.path.join(image_dir, stem + '.png'), image)
    with open(os.path.join(label_dir, stem + '.txt'), 'w') as label_file:
        for polygon in polygons:
            coordinates = ' '.join(f'{value:.6f}' for value in polygon)
            label_file.write(f'0 {coordinates}\n')


def write_dataset_yaml(output_dir):
    content = (
        f'path: {os.path.abspath(output_dir)}\n'
        'train: images/train\n'
        'val: images/val\n'
        'names:\n'
        '  0: good leaf\n'
    )
    with open(os.path.join(output_dir, 'data.yaml'), 'w') as yaml_file:
        yaml_file.write(content)


def write_manifest(output_dir, manifest):
    path = os.path.join(output_dir, 'manifest.json')
    temporary_path = path + '.tmp'
    with open(temporary_path, 'w') as manifest_file:
        json.dump(manifest, manifest_file, indent=2)
    os.replace(temporary_path, path)


def completed_sample_indices(output_dir, required_instances=4):
    pattern = os.path.join(output_dir, 'images', '*', 'leaf_*.png')
    completed = set()
    for path in glob.glob(pattern):
        stem = os.path.splitext(os.path.basename(path))[0]
        split = os.path.basename(os.path.dirname(path))
        label_path = os.path.join(
            output_dir, 'labels', split, stem + '.txt'
        )
        if not os.path.isfile(label_path):
            continue
        with open(label_path) as label_file:
            instance_count = sum(1 for line in label_file if line.strip())
        if instance_count == required_instances:
            completed.add(int(stem.split('_')[-1]))
    return completed


def handle_termination(signum, _frame):
    raise SystemExit(128 + signum)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Collect geometry-labelled Gazebo leaf segmentation data.'
    )
    parser.add_argument('--output', required=True)
    parser.add_argument('--samples', type=int, default=120)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--skip-frames',
        type=int,
        default=3,
        help='Additional frames discarded after each Gazebo pose update.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rng = random.Random(args.seed)
    os.makedirs(args.output, exist_ok=True)
    write_dataset_yaml(args.output)
    completed = completed_sample_indices(args.output)
    manifest_path = os.path.join(args.output, 'manifest.json')
    if os.path.isfile(manifest_path):
        with open(manifest_path) as manifest_file:
            manifest = json.load(manifest_file)
    else:
        manifest = [
            {'index': index, 'resumed_without_pose_metadata': True}
            for index in sorted(completed)
        ]
    if completed:
        print(f'Resuming with {len(completed)} completed samples', flush=True)

    rclpy.init()
    receiver = FrameReceiver()
    sequence = receiver.sequence
    signal.signal(signal.SIGTERM, handle_termination)

    try:
        _, sequence = receiver.next_frame(sequence, skip=0)
        for index in range(args.samples):
            position, quaternion, rpy = sample_camera_pose(rng)
            if index in completed:
                continue
            set_entity_pose(
                'perception_test_camera', position, quaternion
            )
            set_entity_pose('plant_in_front_of_arm', PLANT_POSITION)
            set_entity_pose('perception_label_plant', HIDDEN_POSITION)
            normal, sequence = receiver.next_frame(
                sequence, skip=args.skip_frames
            )

            set_entity_pose('plant_in_front_of_arm', HIDDEN_POSITION)
            set_entity_pose('perception_label_plant', PLANT_POSITION)
            polygons = []
            for _ in range(5):
                label, sequence = receiver.next_frame(
                    sequence, skip=args.skip_frames
                )
                polygons, _ = extract_instance_polygons(normal, label)
                if len(polygons) == 4:
                    break
            if len(polygons) != 4:
                print(
                    f'[{index + 1:03d}/{args.samples:03d}] rejected: '
                    f'expected 4 leaves, found {len(polygons)}',
                    flush=True,
                )
                continue
            split = 'val' if index % 5 == 0 else 'train'
            write_sample(args.output, split, index, normal, polygons)
            manifest.append(
                {
                    'index': index,
                    'split': split,
                    'instances': len(polygons),
                    'camera_position': position.tolist(),
                    'camera_rpy': list(rpy),
                }
            )
            write_manifest(args.output, manifest)
            print(
                f'[{index + 1:03d}/{args.samples:03d}] '
                f'{split}: {len(polygons)} visible leaves',
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        try:
            set_entity_pose('plant_in_front_of_arm', PLANT_POSITION)
            set_entity_pose('perception_label_plant', HIDDEN_POSITION)
        finally:
            write_manifest(args.output, manifest)
            receiver.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

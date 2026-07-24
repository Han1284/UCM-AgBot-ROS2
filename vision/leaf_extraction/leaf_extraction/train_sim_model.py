import argparse
import os

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


def default_base_model():
    return os.path.join(
        get_package_share_directory('leaf_extraction'),
        'segmentation_model',
        'citrus.pt',
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Conservatively fine-tune leaf segmentation on Gazebo data.'
        )
    )
    parser.add_argument('--data', required=True)
    parser.add_argument('--project', required=True)
    parser.add_argument('--model', default=default_base_model())
    parser.add_argument('--name', default='leaf_sim_fine_tune')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--seed', type=int, default=20260724)
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.model)
    result = model.train(
        data=os.path.abspath(args.data),
        epochs=args.epochs,
        patience=15,
        imgsz=640,
        batch=8,
        device=0,
        workers=4,
        project=os.path.abspath(args.project),
        name=args.name,
        seed=args.seed,
        deterministic=True,
        plots=True,
        optimizer='AdamW',
        lr0=0.0001,
        lrf=0.1,
        warmup_epochs=1.0,
        weight_decay=0.0005,
        freeze=10,
        mosaic=0.0,
        close_mosaic=0,
        degrees=5.0,
        translate=0.05,
        scale=0.15,
        fliplr=0.5,
        flipud=0.5,
        hsv_h=0.01,
        hsv_s=0.15,
        hsv_v=0.15,
        erasing=0.0,
    )
    print(f'Best weights: {result.save_dir}/weights/best.pt')


if __name__ == '__main__':
    main()

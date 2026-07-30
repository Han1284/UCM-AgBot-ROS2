#!/usr/bin/env python3

"""Clear stale MTC RViz tasks and remember the currently displayed task."""

from pathlib import Path

from moveit_task_constructor_msgs.msg import TaskDescription
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy


class MtcRvizTaskCleaner(Node):
    """Use MTC's empty-description reset protocol for known task IDs."""

    def __init__(self):
        super().__init__('mtc_rviz_task_cleaner')
        self.state_path = Path('/tmp/leaf_mtc_rviz_task_ids.txt')
        self.known_task_ids = self._load_task_ids()
        self.initial_clear_complete = False
        qos = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            TaskDescription, '/task_description', qos)
        self.subscription = self.create_subscription(
            TaskDescription,
            '/task_description',
            self._description_callback,
            qos,
        )
        self.clear_timer = self.create_timer(
            0.75, self._clear_previous_tasks)

    def _load_task_ids(self):
        try:
            return {
                line.strip()
                for line in self.state_path.read_text(
                    encoding='utf-8').splitlines()
                if line.strip()
            }
        except FileNotFoundError:
            return set()
        except OSError as exc:
            self.get_logger().warning(
                f'Cannot read remembered RViz task IDs: {exc}')
            return set()

    def _save_task_ids(self):
        try:
            text = ''.join(
                f'{task_id}\n' for task_id in sorted(self.known_task_ids))
            self.state_path.write_text(text, encoding='utf-8')
        except OSError as exc:
            self.get_logger().warning(
                f'Cannot remember RViz task IDs: {exc}')

    def _description_callback(self, message):
        if not message.task_id:
            return
        if message.stages:
            self.known_task_ids.add(message.task_id)
        elif self.initial_clear_complete:
            self.known_task_ids.discard(message.task_id)
        self._save_task_ids()

    def _clear_previous_tasks(self):
        self.clear_timer.cancel()
        task_ids = tuple(self.known_task_ids)
        for task_id in task_ids:
            reset = TaskDescription()
            reset.task_id = task_id
            self.publisher.publish(reset)
        self.known_task_ids.clear()
        self._save_task_ids()
        self.initial_clear_complete = True
        self.get_logger().info(
            f'Cleared {len(task_ids)} remembered or active MTC RViz task(s)')


def main(args=None):
    rclpy.init(args=args)
    node = MtcRvizTaskCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

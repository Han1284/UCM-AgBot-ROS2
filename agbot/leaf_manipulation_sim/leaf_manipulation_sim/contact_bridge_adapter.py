#!/usr/bin/env python3
"""Keep the Gazebo Classic contact-topic API after moving to Gazebo Fortress."""

import rclpy
from gazebo_msgs.msg import ContactState, ContactsState
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts


class ContactBridgeAdapter(Node):
    def __init__(self):
        super().__init__('contact_bridge_adapter')
        self._legacy_publishers = {}
        self._contact_subscriptions = []
        self._latest = {leaf_id: None for leaf_id in range(1, 5)}
        self._latest_time_ns = {leaf_id: 0 for leaf_id in range(1, 5)}
        for leaf_id in range(1, 5):
            legacy_topic = f'/plant/leaf_{leaf_id}_contacts'
            native_topic = f'/plant/leaf_{leaf_id}_contacts_gz'
            self._legacy_publishers[leaf_id] = self.create_publisher(
                ContactsState, legacy_topic, 10)
            self._contact_subscriptions.append(self.create_subscription(
                Contacts,
                native_topic,
                lambda msg, i=leaf_id: self._publish(msg, i),
                10,
            ))
        # Gazebo Classic's bumper plugin published an empty ContactsState even
        # while idle. Native Fortress contact sensors publish only on contact.
        self._idle_timer = self.create_timer(0.01, self._publish_periodic)

    def _publish(self, source, leaf_id):
        target = ContactsState()
        target.header = source.header
        if not target.header.frame_id:
            target.header.frame_id = 'world'

        for contact in source.contacts:
            state = ContactState()
            state.collision1_name = contact.collision1.name
            state.collision2_name = contact.collision2.name
            state.contact_positions = list(contact.positions)
            state.contact_normals = list(contact.normals)
            state.depths = list(contact.depths)

            for joint_wrench in contact.wrenches:
                wrench = joint_wrench.body_1_wrench
                state.wrenches.append(wrench)
                state.total_wrench.force.x += wrench.force.x
                state.total_wrench.force.y += wrench.force.y
                state.total_wrench.force.z += wrench.force.z
                state.total_wrench.torque.x += wrench.torque.x
                state.total_wrench.torque.y += wrench.torque.y
                state.total_wrench.torque.z += wrench.torque.z

            if not state.wrenches:
                state.total_wrench = Wrench()
            target.states.append(state)

        self._latest[leaf_id] = target
        self._latest_time_ns[leaf_id] = self.get_clock().now().nanoseconds

    def _publish_periodic(self):
        now = self.get_clock().now()
        for leaf_id, publisher in self._legacy_publishers.items():
            message = self._latest[leaf_id]
            if (message is None or
                    now.nanoseconds - self._latest_time_ns[leaf_id] > 50_000_000):
                message = ContactsState()
                message.header.stamp = now.to_msg()
                message.header.frame_id = 'world'
            publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ContactBridgeAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

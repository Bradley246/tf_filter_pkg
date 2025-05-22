# ~/brad/ros2_ws/src/tf_filter_pkg/tf_filter_pkg/tf_filter_node.py

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TfFilter(Node):
    def __init__(self):
        super().__init__('tf_filter_node_script')

        # Parameters for customization
        self.declare_parameter('source_tf_topic', 'tf')
        # self.declare_parameter('source_tf_static_topic', 'tf_static') # REMOVE
        self.declare_parameter('filtered_tf_topic', 'tf_filtered')
        # self.declare_parameter('filtered_tf_static_topic', 'tf_static_filtered') # REMOVE
        
        self.declare_parameter('parent_frame_to_block', 'default_parent')
        self.declare_parameter('child_frame_to_block', 'default_child')

        source_tf_topic_name = self.get_parameter('source_tf_topic').get_parameter_value().string_value
        # source_tf_static_topic_name = self.get_parameter('source_tf_static_topic').get_parameter_value().string_value # REMOVE
        filtered_tf_topic_name = self.get_parameter('filtered_tf_topic').get_parameter_value().string_value
        # filtered_tf_static_topic_name = self.get_parameter('filtered_tf_static_topic').get_parameter_value().string_value # REMOVE
        
        self.blocked_parent_frame = self.get_parameter('parent_frame_to_block').get_parameter_value().string_value
        self.blocked_child_frame = self.get_parameter('child_frame_to_block').get_parameter_value().string_value

        self.get_logger().info(f"TF Filter started in actual node namespace: {self.get_namespace()}")
        self.get_logger().info(f"Blocking transform: '{self.blocked_parent_frame}' -> '{self.blocked_child_frame}'")
        self.get_logger().info(f"Subscribing to TF on relative topic: '{source_tf_topic_name}'")
        self.get_logger().info(f"Publishing filtered TF to relative topic: '{filtered_tf_topic_name}'")

        # Dynamic TF subscription and publication
        self.tf_sub = self.create_subscription(
            TFMessage,
            source_tf_topic_name,
            self.tf_callback,
            10)
        self.tf_pub = self.create_publisher(
            TFMessage,
            filtered_tf_topic_name,
            10)

        # REMOVE STATIC TF SUBSCRIPTION AND PUBLICATION
        # self.tf_static_sub = self.create_subscription(...)
        # self.tf_static_pub = self.create_publisher(...)

    # process_tf_message remains the same as it's generic
    def process_tf_message(self, msg: TFMessage, publisher):
        filtered_msg = TFMessage()
        transforms_blocked_count = 0
        for transform_stamped in msg.transforms:
            parent = transform_stamped.header.frame_id
            child = transform_stamped.child_frame_id

            if parent == self.blocked_parent_frame and child == self.blocked_child_frame:
                transforms_blocked_count +=1
            else:
                filtered_msg.transforms.append(transform_stamped)
        
        if transforms_blocked_count > 0:
             # self.get_logger().debug(f"Blocked {transforms_blocked_count} instance(s) of {self.blocked_parent_frame} -> {self.blocked_child_frame} in this message.")
             pass # Keep logging minimal here

        if filtered_msg.transforms:
            publisher.publish(filtered_msg)
        elif not filtered_msg.transforms and msg.transforms:
             # self.get_logger().debug(f"All transforms in message were blocked. Original message had {len(msg.transforms)} transforms.")
             pass # Keep logging minimal here


    def tf_callback(self, msg: TFMessage):
        self.process_tf_message(msg, self.tf_pub)

    # REMOVE tf_static_callback METHOD
    # def tf_static_callback(self, msg: TFMessage):
    #     self.get_logger().debug(f"Processing /tf_static message from {self.tf_static_sub.topic_name}")
    #     self.process_tf_message(msg, self.tf_static_pub)

def main(args=None):
    rclpy.init(args=args)
    node = TfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_transforms)  # 20 Hz
        
        # Define transformations
        self.child1_transform = (0.0, 0.0, 0.59696, 0.0, 0.0, 0.0, 1.0)  # (x, y, z, qx, qy, qz, qw)
        self.child2_transform = (0.07402, 0.009, 0.41, 0.0, 0.0, 0.0, 1.0)
        self.child3_transform = (0.0, 0.19, 0.46, 0.0, 0.0, 0.0, 1.0)

    def broadcast_transforms(self):
        current_time = self.get_clock().now().to_msg()

        # Publish transform for child1
        self.publish_transform("velodyne", self.child1_transform, current_time)
        
        # Publish transform for child2
        self.publish_transform("camera_color_optical_frame", self.child2_transform, current_time)
        
        # Publish transform for child3
        self.publish_transform("id_D63A", self.child3_transform, current_time)
    
    def publish_transform(self, child_frame, transform, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "vectornav"
        t.child_frame_id = child_frame
        t.transform.translation.x = transform[0]
        t.transform.translation.y = transform[1]
        t.transform.translation.z = transform[2]
        t.transform.rotation.x = transform[3]
        t.transform.rotation.y = transform[4]
        t.transform.rotation.z = transform[5]
        t.transform.rotation.w = transform[6]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published TF: {t.header.frame_id} -> {t.child_frame_id}")


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


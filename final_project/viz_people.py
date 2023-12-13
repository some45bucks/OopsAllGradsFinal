import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path

class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')
        print("start")
        self.lastTF = None
        self.subscription = self.create_subscription(
            Float32,
            '/prob_topic',
            self.callback1,
            10
        )

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.callback2,
            10
        )

        self.publisher = self.create_publisher(Path, '/person_display_topic', 10)

        self.path = Path()

    def callback1(self, msg):

        if self.lastTF != None and msg.data > 0:    
            print(msg.data)
            transform = self.lastTF.transforms[0]

            translation = transform.transform.translation
            quaternion = transform.transform.rotation

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            pose_stamped.pose.position.x = translation.x
            pose_stamped.pose.position.y = translation.y
            pose_stamped.pose.position.z = translation.z
            pose_stamped.pose.orientation = quaternion

            self.path.poses.append(pose_stamped)
            self.path.header = pose_stamped.header

            self.publisher.publish(self.path)

    def callback2(self, msg):
        self.lastTF = msg

def main():
    rclpy.init()
    viz_node = VizNode()
    rclpy.spin(viz_node)
    viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

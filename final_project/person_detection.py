import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from final_project.rgb_ir_integration.integration import integrated_predictions
from ultralytics import YOLO
from std_msgs.msg import Float32

class ImageIntegrationNode(Node):
    def __init__(self):
        super().__init__('image_integration_node')

        # Initialize YOLO models
        print('loading ir model')
        self.ir_model = YOLO('final_project/rgb_ir_integration/best_ir.pt')
        print('loading rgb model')
        self.rgb_model = YOLO('final_project/rgb_ir_integration/best_rgb.pt')
        print('all models are loaded')
        self.prob_publisher = self.create_publisher(Float32, 'prob_topic', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            1
        )
        self.subscription_paused = False

        self.timer = self.create_timer(4.0, self.timer_callback)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        if not self.subscription_paused:
            try:
                self.subscription_paused = True
                rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
                
                integrated_prob = integrated_predictions(
                    ir_model=self.ir_model,
                    rgb_model=self.rgb_model,
                    rgb_img=rgb_image
                )

                self.get_logger().info(f'Integrated Probability: {integrated_prob}')
                if integrated_prob != None:
                    msg = Float32()
                    msg.data = float(integrated_prob)
                    self.prob_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error processing image: {str(e)}')

    def timer_callback(self):
        self.get_logger().info('Toggling subscription...')
        self.subscription_paused = not self.subscription_paused
        if self.subscription_paused:
            self.get_logger().info('Subscription paused')
        else:
            self.get_logger().info('Subscription resumed')

        


def main():
    rclpy.init()
    image_integration_node = ImageIntegrationNode()
    rclpy.spin(image_integration_node)
    image_integration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

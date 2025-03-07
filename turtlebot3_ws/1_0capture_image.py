#import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rclpy
import os

save_directory = "_img_capture"
os.makedirs(save_directory, exist_ok=True)

# del_img = input("Do you want to delete the previous images: (y/n) ")
# if del_img == 'y':
#     file_name = f'{save_directory}\*.*'
#     os.remove(file_name)
#     print(f"{file_name} has been deleted.")

    
    
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # /camera/image_raw í† í”½ì„ êµ¬ë…
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.subscription
        # CvBridge ì´ˆê¸°í™”
        self.bridge = CvBridge()
        self.image_count = 0

    def listener_callback(self, msg):
        try:
            # ROS2 ë©”ì‹œì§€ë¥¼ OpenCV ì´ë¯¸ì§€ë¡œ ë³€í™˜
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # OpenCVë¡œ ì´ë¯¸ì§€ í‘œì‹œ
            cv2.imshow("Camera Feed", cv_image)
            key = cv2.waitKey(1)  # 1ms ëŒ€ê¸°
            if key == ord('c'):

                # change the filename when multiple people are capturing images
                # ex: obj1_img_{image_count}.jpg
                # then all images and txt files generated can be combined for execution of step 3
                file_name = f'{save_directory}/img_{self.image_count}.jpg'  
                
                cv2.imwrite(file_name, cv_image)
                print(f"Image saved. name:{file_name}")
                self.image_count += 1
            
            elif key == ord('q'):
                cv2.destroyAllWindows()  # OpenCV ì°½ ë‹«ê¸°
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # ì°½ì„ ë‹«ìŒ
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





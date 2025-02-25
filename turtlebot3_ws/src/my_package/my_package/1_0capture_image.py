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
        # /camera/image_raw 토픽을 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.subscription
        # CvBridge 초기화
        self.bridge = CvBridge()
        self.image_count = 0

    def listener_callback(self, msg):
        try:
            # ROS2 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # OpenCV로 이미지 표시
            cv2.imshow("Camera Feed", cv_image)
            key = cv2.waitKey(1)  # 1ms 대기
            if key == ord('c'):

                # change the filename when multiple people are capturing images
                # ex: obj1_img_{image_count}.jpg
                # then all images and txt files generated can be combined for execution of step 3
                file_name = f'{save_directory}/img_{self.image_count}.jpg'  
                
                cv2.imwrite(file_name, cv_image)
                print(f"Image saved. name:{file_name}")
                self.image_count += 1
            
            elif key == ord('q'):
                cv2.destroyAllWindows()  # OpenCV 창 닫기
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
        cv2.destroyAllWindows()  # 창을 닫음
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






from flask import Flask, render_template, Response
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2
import threading
import time

cv_image = None
lock = threading.Lock()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        global cv_image
        with lock:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.waitKey(1)

def ros2_main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)  # Blocking spin, no custom stop mechanism
    finally:
        node.destroy_node()
        rclpy.shutdown()


# -----------------------------------------------------------------------------------------


app = Flask(__name__, template_folder="/home/soo/turtlebot3_ws/templates")# 절대 경로로 바꾸기

def generate_frames(camera_id):
    camera = cv2.VideoCapture(camera_id)
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

def generate_frames1():
    global cv_image
    while True:
        with lock:
            if cv_image is None:
                time.sleep(0.1)
                continue
            
            
        ret, buffer = cv2.imencode('.jpg', cv_image)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template('two_cam_index.html')

@app.route('/video_feed1')
def video_feed1():
    return Response(generate_frames(0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames1(), mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    ros2_thread = threading.Thread(target=ros2_main, daemon=True)
    ros2_thread.start()
    app.run(debug=True, use_reloader=False)
    
if __name__ == "__main__":
    main()
    
    

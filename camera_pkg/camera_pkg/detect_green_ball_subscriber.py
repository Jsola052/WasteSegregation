import numpy as np
import cv2
import pickle
from sklearn.neighbors import KNeighborsClassifier
from pyfirmata import Arduino
import urx
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Global variables for interactive ball selection
selected_ball_center = None
selected_ball_depth = None
depth_image = None
color_image = None

# Function to preprocess the image for feature extraction
def preprocess_image(image):
    if image is None or image.size == 0:
        print("Error: Ball image is empty or None.")
        return None
    
    try:
        resized_image = cv2.resize(image, (30, 30))  # Resize to 30x30 pixels
        flattened_image = resized_image.flatten()  # Flatten the image
        return flattened_image
    except Exception as e:
        print(f"Error in preprocessing image: {e}")
        return None

# Mouse callback function for interactive ball selection
def select_ball(event, x, y, flags, param):
    global selected_ball_center, selected_ball_depth

    if event == cv2.EVENT_LBUTTONDOWN:
        depth_value = depth_image[y, x]
        if depth_value > 0:
            selected_ball_center = (x, y)
            selected_ball_depth = depth_value
        else:
            print("Invalid depth value.")

def deg2rad(deg):
    return deg * np.pi / 180            
            
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.bridge = CvBridge()
        self.color_subscriber = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.color_callback, 
            10
        )
        self.depth_subscriber = self.create_subscription(
            Image, 
            '/camera/depth/image_raw', 
            self.depth_callback, 
            10
        )
        self.knn = KNeighborsClassifier(n_neighbors=3)
        self.training_data = []
        self.labels = []

    def color_callback(self, msg):
        global color_image
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def depth_callback(self, msg):
        global depth_image
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def train_knn(self):
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        for i in range(10):  # Collect 10 samples for each ball
            if color_image is None or depth_image is None:
                print("Waiting for images...")
                time.sleep(1)
                continue

            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_green:
                area = cv2.contourArea(contour)
                if area > 500:  # Filter small contours
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    ball_image = color_image[center[1]-radius:center[1]+radius, center[0]-radius:center[0]+radius]
                    features = preprocess_image(ball_image)
                    if features is not None:
                        self.training_data.append(features)
                        self.labels.append('green')

                        cv2.putText(color_image, 'green', (center[0], center[1] - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        cv2.circle(color_image, center, radius, (0, 255, 0), 2)

            cv2.imshow('RealSense', color_image)
            cv2.setMouseCallback('RealSense', select_ball)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if self.training_data and self.labels:
            self.knn.fit(self.training_data, self.labels)
            with open('ball_classifier.pkl', 'wb') as file:
                pickle.dump(self.knn, file)
            print("Training completed. Waiting for ball selection.")

    def detect_ball(self):
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        while True:
            if color_image is None or depth_image is None:
                print("Waiting for images...")
                time.sleep(1)
                continue

            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_green:
                area = cv2.contourArea(contour)
                if area > 500:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    ball_image = color_image[center[1]-radius:center[1]+radius, center[0]-radius:center[0]+radius]
                    features = preprocess_image(ball_image)
                    if features is not None:
                        label = self.knn.predict([features])[0]
                        if label == 'green':
                            cv2.putText(color_image, label, (center[0], center[1] - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            cv2.circle(color_image, center, radius, (0, 255, 0), 2)

            cv2.imshow('RealSense', color_image)
            cv2.setMouseCallback('RealSense', select_ball)
            if selected_ball_center is not None:
                depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [selected_ball_center[0], selected_ball_center[1]], selected_ball_depth)
                print(f"Selected green ball 3D coordinates: {depth_point}")
                break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main():
    connected = False
    tries = 0
    max_tries = 5
    while not connected and tries < max_tries:
        try:
            time.sleep(0.3)
            robot = urx.Robot("172.16.3.114")
            time.sleep(0.3)
            connected = True
        except:
            tries += 1
            print(f"Connection attempt {tries} failed.")
            time.sleep(1)  # Wait for a second before next attempt

    if connected:
        global selected_ball_center, selected_ball_depth
        board = Arduino('/dev/arduino')
        relay_pin_number = 7
        relay_pin = board.get_pin(f'd:{relay_pin_number}:o')
        home(robot, 0.5, 0.5)
        relay_pin.write(0)

        rclpy.init()
        ball_detector = BallDetector()
        ball_detector.train_knn()

        while rclpy.ok():
            rclpy.spin_once(ball_detector)
            ball_detector.detect_ball()

            robot.movel((-0.19260, -0.55264, 0.2109, 0.001, 3.145, -0.001), 0.2, 0.2)
            robot.movel((-0.19260, -0.55264, 0.19596, 0.001, 3.145, -0.001), 0.2, 0.2)
            time.sleep(1)
            relay_pin.write(1)
            time.sleep(1)
            robot.movel((-0.19260, -0.55264, 0.2109, 0.001, 3.145, -0.001), 0.2, 0.2)
            home(robot, 0.5, 0.5)
            robot.movel((-0.59987, -0.56180, 0.45339, 0.001, 3.145, -0.001), 0.2, 0.2)
            time.sleep(1)
            relay_pin.write(0)
            time.sleep(1)
            home(robot, 0.5, 0.5)
            robot.close()

        ball_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
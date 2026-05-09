# ROS2 Colour Detection and Blue Box Following
# Detects RED, GREEN and BLUE boxes
# Robot patrols using Nav2 goals
# When BLUE is detected:
#   -> robot turns toward it
#   -> drives toward it
#   -> stops roughly 1 metre away

import threading
import signal
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nav2_msgs.action import NavigateToPose

from math import sin, cos


class Robot(Node):

    def __init__(self):

        super().__init__('robot')

        # ---------------- NAVIGATION ---------------- #

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Patrol waypoints
        self.goals = [
            (0.732, -7.45, 0.1),
            (2.0, -4.7, 0.0),
            (1.5, -1.5, 0.145),
            (-1.8, 5.0, 0.2)
        ]

        self.goal_index = 0
        self.goal_active = False

        # ---------------- CAMERA ---------------- #

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # ---------------- MOVEMENT ---------------- #

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ---------------- FLAGS ---------------- #

        self.blue_detected = False
        self.searching_blue = True

        print("Robot node started")

    # ==========================================================
    # NAVIGATION FUNCTIONS
    # ==========================================================

    def send_goal(self, x, y, yaw):

        if self.goal_active:
            return

        self.goal_active = True

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        print(f"Sending goal: ({x}, {y})")

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            print("Goal rejected")
            self.goal_active = False
            return

        print("Goal accepted")

        self.result_future = goal_handle.get_result_async()

        self.result_future.add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(self, future):

        print("Goal reached")

        self.goal_active = False

    # ==========================================================
    # ROBOT MOVEMENT
    # ==========================================================

    def stop_robot(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.publisher.publish(twist)

    # ==========================================================
    # DRAW DETECTIONS
    # ==========================================================

    def draw_detection(self, image, mask, label, colour):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected = False
        largest_contour = None

        if len(contours) > 0:

            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(largest_contour)

            if area > 300:

                detected = True

                x, y, w, h = cv2.boundingRect(
                    largest_contour
                )

                # Draw rectangle
                cv2.rectangle(
                    image,
                    (x, y),
                    (x + w, y + h),
                    colour,
                    2
                )

                # Draw label
                cv2.putText(
                    image,
                    label,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    colour,
                    2
                )

        return detected, largest_contour

    # ==========================================================
    # CAMERA CALLBACK
    # ==========================================================

    def camera_callback(self, data):

        try:

            image = self.bridge.imgmsg_to_cv2(
                data,
                'bgr8'
            )

        except Exception as e:

            print(e)
            return

        # Convert to HSV
        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )

        # ======================================================
        # HSV COLOUR RANGES
        # ======================================================

        # GREEN
        green_lower = np.array([50, 100, 100])
        green_upper = np.array([70, 255, 255])

        # BLUE
        blue_lower = np.array([110, 100, 100])
        blue_upper = np.array([130, 255, 255])

        # RED
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])

        red_lower2 = np.array([170, 100, 100])
        red_upper2 = np.array([180, 255, 255])

        # ======================================================
        # CREATE MASKS
        # ======================================================

        green_mask = cv2.inRange(
            hsv,
            green_lower,
            green_upper
        )

        blue_mask = cv2.inRange(
            hsv,
            blue_lower,
            blue_upper
        )

        red_mask1 = cv2.inRange(
            hsv,
            red_lower1,
            red_upper1
        )

        red_mask2 = cv2.inRange(
            hsv,
            red_lower2,
            red_upper2
        )

        red_mask = cv2.bitwise_or(
            red_mask1,
            red_mask2
        )

        # ======================================================
        # DETECT AND DRAW ALL COLOURS
        # ======================================================

        self.draw_detection(
            image,
            red_mask,
            "RED",
            (0, 0, 255)
        )

        self.draw_detection(
            image,
            green_mask,
            "GREEN",
            (0, 255, 0)
        )

        blue_found, blue_contour = self.draw_detection(
            image,
            blue_mask,
            "BLUE",
            (255, 0, 0)
        )

        # ======================================================
        # DISPLAY CAMERA FEED
        # ======================================================

        cv2.namedWindow(
            "Camera Feed",
            cv2.WINDOW_NORMAL
        )

        cv2.imshow(
            "Camera Feed",
            image
        )

        cv2.resizeWindow(
            "Camera Feed",
            640,
            480
        )

        cv2.waitKey(1)

        # ======================================================
        # BLUE BOX FOLLOWING
        # ======================================================

        if blue_found:

            self.blue_detected = True

            area = cv2.contourArea(
                blue_contour
            )

            M = cv2.moments(
                blue_contour
            )

            if M['m00'] != 0:

                cx = int(
                    M['m10'] / M['m00']
                )

                image_center_x = image.shape[1] / 2

                error_x = cx - image_center_x

                twist = Twist()

                # Turn toward blue object
                twist.angular.z = -0.002 * error_x

                # ==================================================
                # STOP DISTANCE
                #
                # Larger contour area = closer object
                #
                # Tune this value if needed
                # ==================================================

                STOP_AREA = 12000

                if area < STOP_AREA:

                    print("Moving toward blue box")

                    twist.linear.x = 0.15

                else:

                    print("Stopped near blue box")

                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher.publish(twist)

                return

        # ======================================================
        # PATROL MODE
        # ======================================================

        if not self.goal_active:

            x, y, yaw = self.goals[self.goal_index]

            self.send_goal(x, y, yaw)

            self.goal_index += 1

            if self.goal_index >= len(self.goals):

                self.goal_index = 0


# ==============================================================
# MAIN
# ==============================================================

def main():

    rclpy.init(args=None)

    robot = Robot()

    def signal_handler(sig, frame):

        print("Shutting down...")

        robot.stop_robot()

        cv2.destroyAllWindows()

        rclpy.shutdown()

        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(
        target=rclpy.spin,
        args=(robot,),
        daemon=True
    )

    thread.start()

    try:

        while rclpy.ok():

            time.sleep(0.1)

    except ROSInterruptException:

        pass

    finally:

        robot.stop_robot()

        cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

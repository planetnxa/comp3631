# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
    
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use

        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10 # can never be too careful
        self.blueFlag = False # iniit false flag
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) # publisher to send messages to robo base, 10 rate
        self.rate = self.create_rate(10)  # 10 Hz
        self.too_close = False



    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
    
        hsv_red_11 = np.array([0-self.sensitivity,100,100]) # will hsv ever e negative?
        hsv_red_12 = np.array([0+self.sensitivity,255,255])

        hsv_red_21 = np.array([180-self.sensitivity,100,100])
        hsv_red_22 = np.array([180+self.sensitivity,255,255])

        hsv_blue_low = np.array([120-self.sensitivity,100,100])
        hsv_blue_uo = np.array([120+self.sensitivity,255,255])

    

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        green_mask = cv2.inRange(Hsv_image, hsv_green_low, hsv_green_up)
        blue_mask = cv2.inRange(hsv_img, hsv_blue_low,hsv_blue_up)
        red_mask1 = cv2.inRange(hsv_img, hsv_red_11,hsv_red_12)
        red_mask2 = cv2.inRange(hsv_img, hsv_red_21,hsv_red_22)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        mask_image1 = cv2.bitwise_or(green_mask, red_mask)
        mask_image = cv2.bitwise_or(mask_image1,blue_mask)
     
        final_image = cv2.bitwise_and(image,image,mask = mask_image)
        # we can see all colours
        
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(blue_mask, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
       # self.blueFlag = False - alr established
       # conotour colour determined in find contours!!joy

        if len(contours) > 0: # oh but what if no
            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            c = max(contours, key=cv2.contourArea) # laargest spot

            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00']) # read

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            print(cv2.contourArea(c))
            if cv2.contourArea(c) > 200: #suitable area......determine on examination

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                # (x, y), radius = cv2.minEnclosingCircle(c)
                # center = (int(x), int(y)) 
                # radius = int(radius) 

                # cv2.circle(image, center, radius, (255,255,0) ,1)

                # Then alter the values of any flags
                self.blueFlag = True
            else:
                self.blueFlag = False   

        #if the flag is true (colour has been detected)
            #print the flag or colour to test that it has been detected
            #alternatively you could publish to the lab1 talker/listener
        if self.blueFlag == True:
                print("Blue found") # and then move towards it
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('threshold_Feed2', cv2.WINDOW_NORMAL) 
        cv2.imshow('threshold_Feed2', image)
        cv2.resizeWindow('threshold_Feed2', 320, 240)
        cv2.waitKey(3)
        
        #Check if a flag has been set = colour object detected - follow the colour object
        if self.blueFlag == True:
            if cv2.contourArea(c) > 1000:
                # Too close to object, need to move backwards
                print("backward")
                self.too_close = True
                #self.walk_backward()
            elif cv2.contourArea(c) <= 30000 :
                print("forward")
                self.too_close = False
                # Too far away from object, need to move forwards
                #self.walk_forward()
        else:
            self.stop()


            # Be sure to do this for the other colour as well
            # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

        # Publish moves

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)



# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.blueFlag == True:
                if robot.too_close == True:
                    robot.walk_backward()
                else:
                    robot.walk_forward()
            #else:
            #    robot.stop()
    except ROSInterruptException:
        pass
# Check if the node is executing in the main path

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()


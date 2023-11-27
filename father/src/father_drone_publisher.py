#!/usr/bin/env python3
from djitellopy import tello 
from time import sleep
#
import cv2
from cv_bridge import CvBridge, CvBridgeError
#
import matplotlib.pyplot as plt
import copy
import numpy as np
#
from sensor_msgs.msg import Image
import rospy
#
from operator import mul
from functools import reduce
#
from ringbuff import RingBuffer

# ----------------
CAM_WIDTH = 960
CAM_HEIGHT = 720
CAM_CHANNELS = 3

# ----------------
IMG_SHAPE_DRONE = (CAM_HEIGHT, CAM_WIDTH, CAM_CHANNELS)
IMG_SIZE_DRONE = reduce(mul, IMG_SHAPE_DRONE)

### Buffer Initialization
BUFF_SIZE = 30
RESULT_BUFF_SIZE = IMG_SIZE_DRONE * BUFF_SIZE
# ----------------
result_buffer = RingBuffer(RESULT_BUFF_SIZE, 'float32')
# result_buffer_lock = threading.RLock()
# ----------------
WINDOW_NAME_RESULT = "RESULT"


class drone_img_publisher():
    def __init__(self):
        self.drone = tello.Tello()
        #
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/drone_img/low_image', Image, queue_size=20)
        #
        self.drone_initialization()

    def drone_initialization(self):
        self.drone.connect()
        print("BATTERY LEVEL: ", self.drone.get_battery())
    
    def imagePublish(self):
        BUFF_COUNT = 0
        #
        self.drone.streamon()
        while not rospy.is_shutdown():
            image = self.drone.get_frame_read().frame
            image = cv2.cvtColor(cv2.resize(image,(CAM_WIDTH,CAM_HEIGHT)), cv2.COLOR_BGR2RGB)
            #
            if not result_buffer.is_full() and reduce(mul, image.shape) > 0:
                #
                result_buffer.push(image.flatten().astype('float32'))
                BUFF_COUNT += 1
                print('The number of images buffered: ', BUFF_COUNT)
            #
            if result_buffer.nb_data > 0:
                popped_image = result_buffer.get(IMG_SIZE_DRONE)
                result_buffer.pop(IMG_SIZE_DRONE)
                #
                popped_image = popped_image.reshape(IMG_SHAPE_DRONE).astype('uint8')
                print(popped_image.shape)
                #
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(popped_image, 'bgr8'))
                #
                # plt.imshow(popped_image)
                # plt.show(block=False)
                # plt.pause(1/100000)
                #
                BUFF_COUNT -= 1
                print('The number of images buffered: ', BUFF_COUNT)
            


if __name__ == '__main__':
    rospy.init_node('drone_img_publisher')
    #
    drone_publisher = drone_img_publisher()
    #
    try:
        drone_publisher.imagePublish()
    except KeyboardInterrupt:
        print("Shutting down")

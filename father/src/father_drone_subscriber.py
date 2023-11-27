#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from functools import reduce
from operator import mul
from ringbuff import RingBuffer

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

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.color_sub = rospy.Subscriber("/drone_img/low_image", Image,self.callback_rgb)
  def callback_rgb(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imshow('rgb', cv_image)
      cv2.waitKey(1)
      BUFF_COUNT = 0
      if not result_buffer.is_full() and reduce(mul, cv_image.shape) > 0:
          #
          result_buffer.push(cv_image.flatten().astype('float32'))
          BUFF_COUNT += 1
          print('The number of images buffered: ', BUFF_COUNT)
    except:
      pass


def main():
    ic = image_converter()
    rospy.init_node('drone_img_subscriber', anonymous=True)
    try:
      rospy.spin()
    except keyboardInterrupt:
      print("shutting down")
      cv2.destroyAllWindows()

if __name__=='__main__':
  main()

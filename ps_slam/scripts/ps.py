#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header



class image_converter:
	def __init__(self):    
		# 创建cv_bridge，声明图像的发布者和订阅者
		self.image_pub = rospy.Publisher("ps_node", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/orbslam2/rgb", Image, self.callback)
		self.counter=0

	def callback(self,data):
		# 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e
		
		self.counter+=1
		rospy.loginfo(str(self.counter))
#		if(self.counter%10!=0):
#			return

		# 在opencv的显示窗口中绘制一个圆，作为标记
#		(rows,cols,channels) = cv_image.shape
#		if cols > 60 and rows > 60 :
#			cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
			
		
		
		# 再将opencv格式额数据转换成ros image格式的数据发布
		try:
			msg_send=self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
			msg_send.header=data.header
			print(msg_send.header)
			self.image_pub.publish(msg_send)
		except CvBridgeError as e:
			print e

if __name__ == '__main__':
	try:
		rospy.init_node("ps_node")
		rospy.loginfo("Starting Panoptic Segmentation node")
		image_converter()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down Panoptic Segmentation node."
#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('game_controller')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Measurement:
    def __init__(self, coordinates,index):
        self.coordinates = coordinates
        self.index = index

class game_controller:
	def __init__(self):
		self.pub = rospy.Publisher("buttons",String,queue_size=1)
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
		
		self.ready = 0
		
		#TODO centri za sve (center_1...5)
		self.x_center = 430
		self.y_center = 610
		self.radius = 115
		
	def compareimages(self,image1,image2,mask):
		points1=self.getpoints(image1,mask)
		points2=self.getpoints(image2,mask)
		sum=0
		minDistance=10000
		for i in range(len(points1)):
		    sum+=self.findnearestNeighbourDistance(points1[i],points2)

		print ("pins position diff=",sum)
		
		#TODO ako je vece od praga vrati 1
		return "0"


	def findnearestNeighbourDistance(self,point1,points2):
		nearestpoint=points2[0]
		minDistance=100000;
		for j in range(len(points2)):
		    if(self.pointdistance(point1,points2[j])<minDistance):
		        minDistance=self.pointdistance(point1,points2[j])
		        nearestpoint=points2[j]
		#print("nearest point to ",point1," is ",nearestpoint,"with distance",minDistance)
		return minDistance


	def pointdistance(self,point1,point2):
		sum=0
		sum += abs(point1[0] - point2[0])
		sum += abs(point1[1] - point2[1])
		return sum


	def getpoints(self, cv_image, mask_circle):
		points=[]
		
		(rows,cols,channels) = cv_image.shape
		img2gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		img2gray = cv2.bitwise_not(img2gray)

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		hsv = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))
		hsv = cv2.bitwise_not(hsv)
		
		mask = cv2.bitwise_and(img2gray, img2gray, mask=hsv)
		ret, mask = cv2.threshold(mask, 40, 255,cv2.THRESH_BINARY)  
		
		kernel = np.ones((3, 3), np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		mask = cv2.bitwise_and(mask, mask, mask=mask_circle)

		# find contours in the thresholded image
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[1]
		opening = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
		
		# loop over the contours
		for c in cnts:
		    # compute the center of the contour
		    M = cv2.moments(c)
		    if(M["m00"]!=0):
		        cX = int(M["m10"] / M["m00"])
		        cY = int(M["m01"] / M["m00"])

		        # draw the contour and center of the shape on the image
		        #print(cv2.contourArea(c))
		        if(80<cv2.contourArea(c)):

		            cv2.drawContours(opening, [c], -1, (0, 255, 0), 2)
		            cv2.circle(opening, (cX, cY), 2, (0, 0, 255), -1)
		            points.append((cX,cY))
		#print (points)
		return points

	def create_mask(self, cv_image, x_center, y_center, radius, rows, cols):
		mask_circle = np.zeros((rows, cols, 1),np.uint8)  
		cv2.circle(mask_circle, (y_center, x_center), radius, (255), -1)
		return mask_circle
		
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		buttons = ""
			
		if self.ready:
			#TODO za svih 5
			buttons += self.compareimages(cv_image, self.initial_img, self.mask)
			#buttons += self.compareimages(cv_image, self.initial_img, self.mask_2)
			#buttons += self.compareimages(cv_image, self.initial_img, self.mask_3)
			#buttons += self.compareimages(cv_image, self.initial_img, self.mask_4)
			#buttons += self.compareimages(cv_image, self.initial_img, self.mask_5)
			
			self.pub.publish(buttons)
		else:			
			#TODO maske i pocetne za sve (mask_1...5)
			(rows,cols,channels) = cv_image.shape
			self.mask = self.create_mask(cv_image, self.x_center, self.y_center, self.radius, rows, cols)
			
			self.initial_img = cv_image
			self.ready = 1
		
		#cv2.imshow("mask", self.mask)
		#cv2.waitKey(10)
		
		#cv2.imshow("initial", self.initial_img)
		#cv2.waitKey(10)
		

def main(args):
	rospy.init_node('game_controller', anonymous=True)
	gc = game_controller()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)

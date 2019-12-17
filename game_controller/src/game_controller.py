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

class game_controller:
	def __init__(self):
		self.pub = rospy.Publisher("buttons",String,queue_size=1)
		self.sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
		self.bridge = CvBridge()
		
		self.ready = 0
		self.last_string = ""

		self.initial_cnt_points = []
		self.rolling_sums=[ LinkedRing(5), LinkedRing(5), LinkedRing(5), LinkedRing(5), LinkedRing(5)]
		
		
		
	def compareimages(self, current_image):
		current_cnt_points = self.getpoints(current_image)
		current_cnt_points = self.sortPoints(current_cnt_points)
		if (len(current_cnt_points[0]) == 0 and len(current_cnt_points[1]) == 0 and len(current_cnt_points[2]) == 0 and len(current_cnt_points[3]) == 0 and len(current_cnt_points[4]) == 0):
			return "00000"
		
		string = ""
		for i in range(len(current_cnt_points)):
			sum = 0
			for j in range(min(len(current_cnt_points[i]),len(self.initial_cnt_points[i]))):
		    		sum += self.findnearestNeighbourDistance(current_cnt_points[i][j], self.initial_cnt_points[i])
			print(sum)
			if( self.last_string!="00000"  and self.rolling_sums[i]>sum and self.last_string!=""):
				self.intial_cnt_points=current_cnt_points
				string += "0"
			elif (sum > 50):
				string += "1"
			
			else:
				string += "0"
			self.rolling_sums[i].add_val(sum)
			print(">> Average: %f" % self.rolling_sums[i].average())
		self.last_string = string

		
		#if (string == "00000"):
		#	self.initial_cnt_points = current_cnt_points
		return string


	def findnearestNeighbourDistance(self,point1,points2):
		nearestpoint=points2[0]
		minDistance=100000;
		for j in range(len(points2)):
		    if(self.pointdistance(point1,points2[j])<minDistance):
		        minDistance=self.pointdistance(point1,points2[j])
		        nearestpoint=points2[j]
		return minDistance


	def pointdistance(self,point1,point2):
		return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

	def sortPoints(self, contour_points):
		points_up = []
		points_down = []
		points_left = []
		points_right = []
		points_space = []

		for c in contour_points:
			if(c[0] < 310):
				points_space.append(c)
			elif(c[0] > 700 and c[1] > 400):
				points_right.append(c)
			elif(c[0] < 650 and c[1] < 370):
				points_left.append(c)
			elif(c[0] > 650 and c[1] < 370):
				points_up.append(c)
			elif(c[0] < 650 and c[1] > 400):
				points_down.append(c)
		
		points = [points_up, points_down, points_left, points_right, points_space]
		return points


	def getpoints(self, cv_image):
		points = []
		
		(rows,cols,channels) = cv_image.shape
		img2gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		img2gray = cv2.bitwise_not(img2gray)

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		hsv = cv2.inRange(hsv, (40,0,0), (90, 255, 255))
		hsv = cv2.bitwise_not(hsv)
		
		mask = cv2.bitwise_and(img2gray, img2gray, mask=hsv)
		ret, mask = cv2.threshold(mask, 90 , 255, cv2.THRESH_BINARY) 

		mask2 = np.zeros((rows+2, cols+2), np.uint8)
		cv2.floodFill(mask, mask2, (0,0), 0)
		
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

		# find contours in the thresholded image
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[1]
		opening = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

		# loop over the contours
		for c in cnts:
			# compute the center of the contour
			M = cv2.moments(c)
			if(M["m00"]!=0 and 150 < cv2.contourArea(c) < 900):
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				cv2.drawContours(opening, [c], -1, (0, 255, 0), 2)
				cv2.circle(opening, (cX, cY), 2, (0, 0, 255), -1)
				points.append((cX,cY))

		cv2.imshow("initial", opening)
		cv2.waitKey(10)
		return points

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
					
		if self.ready:
			buttons = self.compareimages(cv_image)
			self.pub.publish(buttons)
		else:			
			self.initial_img = cv_image
			self.ready = 1
			self.initial_cnt_points = self.getpoints(self.initial_img)
			self.initial_cnt_points = self.sortPoints(self.initial_cnt_points)
		
class Link(object):
    def __init__(self, value=0.0):
        self.next = None
        self.value = value

class LinkedRing(object):
    def __init__(self, length):
        self.sum = 0.0
        self.length = length
        self.current = Link()

        # Initialize all the nodes:
        last = self.current
        for i in xrange(length-1):  # one link is already created
            last.next = Link()
            last = last.next
        last.next = self.current  # close the ring

    def add_val(self, val):
        self.sum -= self.current.value
        self.sum += val
        self.current.value = val
        self.current = self.current.next

    def average(self):
        return self.sum / self.length


def main(args):
	rospy.init_node('game_controller', anonymous=True)
	gc = game_controller()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)


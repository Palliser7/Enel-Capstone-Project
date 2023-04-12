#Author: Nicholas Altwasser
#SSID: 200389010
#
#
#
#
import cv2
import numpy as np
import time
import yaml



class camera_tracking():
	def __init__(self):
		self.thread_enable = 1
	def position(self,queue_x, queue_y, queue_time_pos, queue_stop):
		# Set up video capture device (0 for default camera)
		cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

		#lower and upper bounds for the silver in hsv colour
		silver_color_lower = np.array([110, 50, 50])
		silver_color_upper = np.array([155, 255, 255])
		
		
		#lower and upper bounds for the brass in hsv colour
		brass_color_lower = np.array([0,0,100])
		brass_color_upper = np.array([35,50,255])
		
		#load calibration data
	
		cv_file = cv2.FileStorage('/home/pi/Desktop/Camera Calibration/calibration.yaml', cv2.FILE_STORAGE_READ)
	
		camera_matrix = cv_file.getNode("K").mat()
	
		dist_matrix = cv_file.getNode("D").mat()
	
		cv_file.release()	
		
		#hieght and width of the mat in pixels
		h = 850
		w = 1000
		#hieght and width of the mat in true diemensions
		h_t= 42#cm
		w_t= 58#cm
		
		
		while self.thread_enable:
			#read each frame of the camera
			ret, frame = cap.read()
			
			# Undistort frame
			frame = cv2.undistort(frame, camera_matrix, dist_matrix, None, camera_matrix) 
			cropped_frame = frame[0:0 + h, 0:0 + w]

			if not ret:
				# exit if there is an error reading the frame
				print("i have error")
				break
			
			hsv_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

			#mask to only get colours of the silver ball

			silver_mask= cv2.inRange(hsv_frame, silver_color_lower, silver_color_upper)
			#brass_mask= cv2.inRange(hsv_frame, brass_color_lower, brass_color_upper)
			
			
			#find the contours in the silver ball colour range
			contours_cropped , _ = cv2.findContours(silver_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		
			if len(contours_cropped) > 0:
				#select the largested contour (ball)
				ball_contour_cropped = max(contours_cropped, key=cv2.contourArea)

				#get the center of the ball centroid
				M = cv2.moments(ball_contour_cropped)
				if M["m00"] != 0:
					cx = int(M["m10"] / M["m00"])
					cy = int(M["m01"] / M["m00"])
					
					cx= cx*(w_t/w)
					cy= cy*(h_t/h)
					
			
					queue_x.put(cx)
							
					queue_y.put(cy)
						
					queue_time_pos.put(time.time())
							
				
				
				#cv2.imshow("Frame", silver_mask)
				
				try:
					if queue_stop.get() == 1:
						self.thread_enable = 0
				except:
					pass

				time.sleep(0.5)
				



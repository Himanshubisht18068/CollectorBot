import sys
import vrep
import serial
from xbee import XBee 



import time
import math

import numpy as np
import cv2
import cv2.aruco as aruco
from ArUco_library import *

# # PYTHON CONSTANTS # #
X , Y , Z = 0 , 1, 2

CB_ARUCO_MARKER_ID = 0            # CB Aruco Marker ID
LEFT_TOP_ARUCO_MARKER_ID = 10         # Left Top Aruco Marker ID
RIGHT_BOTTOM_ARUCO_MARKER_ID = 11     # Right Bottom Aruco Marker ID


# Dimension of Flex
FLEX_ORIGINAL_LENGTH = 2.3            # Flex Length in Meters
FLEX_ORIGINAL_WIDTH = 1.87             # Flex Width in Meters



# # Vrep CONSTANTS # #
WORLD_REFERENCE = -1

OBJECT_BOUNDING_BOX_MIN_X = 15          
OBJECT_BOUNDING_BOX_MAX_X = 18

OBJECT_BOUNDING_BOX_MIN_Y = 16
OBJECT_BOUNDING_BOX_MAX_Y = 19

debug = True
a , b , c, d = 0,0,0,0





		    





def UpdateBotInVrep(VideoFrames,a,b,c,d,Vrep_ResizeableFloor_Length,Vrep_ResizeableFloor_Width,clientID,robot_handle):

	while(True):
		# Getting a Frame
		IsFrameAvailable, frame = VideoFrames.read()


		# Frames are available
		if IsFrameAvailable == True:

			 

			# Scaling the Image using the CENTER Coordinates of LeftTOp and RightBottom Aruco Marker
		 	
			frame = frame[ int(a) : int(b)  ,  int(c) : int (d) ] 


			
		    # Displaying the Frame
			cv2.imshow('frame1',frame)


			# Function to Detect all the Aruco Markers Present in the Frame
			Detected_ArUco_markers = detect_ArUco(frame)
			


		    	# If Necessary Aruco Markers are UnDetected, Loop Back
		   	try:
		   		
				# Getting CB Cornes Coodinates
				CB = Detected_ArUco_markers [ CB_ARUCO_MARKER_ID ]

		   	except Exception as e:
				# If Exception occured

				# print Missing Aruco Marker ID
				if debug:
					print "Exception Occured..... Aruco Marker ID = " + (str(e) + " Un-Decteded.....")

				# Loop back
				continue


		   		
		    

		    	# Calulating Orientation of each Aruco Marker in ROI
		   	ArucoMarkes_Angle_Dict = Calculate_orientation_in_degree(Detected_ArUco_markers)	      
		   	
		    	# Marking the ARUCO Marker in the Image
		   	frame = mark_ArUco(frame,Detected_ArUco_markers,ArucoMarkes_Angle_Dict)						

		    	# Showing the Frame with Marker Aruco Markers
			if debug:
				cv2.imshow('frame',frame)        
		
			FrameLength = frame.shape[1]    
	    		FrameWidth = frame.shape[0]     

		    	
			ArucoMarkers_Centers_List = GetCenterCoordinates([CB])
		    	CB_Position_On_Image = ArucoMarkers_Centers_List[0]

		    	# Getting Center X,Y Coordinates of CB on Image
		    	CB_Position_On_Image_X = CB_Position_On_Image[0]
		    	CB_Position_On_Image_Y = CB_Position_On_Image[1]




		    	# Calculating the Actual Position of CB on Flex Sheet
		     
		      	# X Coordinate of CB on Flex
		    	CB_Position_On_Flex_X = (CB_Position_On_Image_X / FrameLength ) * FLEX_ORIGINAL_LENGTH
		      
		      	# Y Coordinate of CB on Flex
		    	CB_Position_On_Flex_Y = (CB_Position_On_Image_Y / FrameWidth ) * FLEX_ORIGINAL_WIDTH





		    	# Calculating the Position of CB in Vrep , by using the Actual position of CB on Flex Sheet

		      	# X Coordinate of CB on Vrep Floor
		    	CB_Position_On_VrepFloor_X = CB_Position_On_Flex_X / FLEX_ORIGINAL_LENGTH * Vrep_ResizeableFloor_Length
		      
		      	# Adjusting the X Coordinate of CB on Vrep Floor, since Vrep Coordiantes are from -x to +x
		    	CB_Position_On_VrepFloor_X =  CB_Position_On_VrepFloor_X - (Vrep_ResizeableFloor_Length / 2)


		      	# Y Coordinate of CB on Vrep Floor
		    	CB_Position_On_VrepFloor_Y = CB_Position_On_Flex_Y / FLEX_ORIGINAL_WIDTH * Vrep_ResizeableFloor_Width
		     
		      	# Adjusting the Y Coordinate of CB on Vrep Floor, since Vrep Coordiantes are from -y to +y
		    	CB_Position_On_VrepFloor_Y =  (Vrep_ResizeableFloor_Width / 2) - CB_Position_On_VrepFloor_Y 



		    	# Calculating CB orientation using the Orientation of CB Aruco Marker
		    	CB_Angle = ArucoMarkes_Angle_Dict[CB_ARUCO_MARKER_ID]   


		    	# Setting the Position of CB on Vrep floor
		    	returnCode = vrep.simxSetObjectPosition(clientID, robot_handle, WORLD_REFERENCE, [CB_Position_On_VrepFloor_X,CB_Position_On_VrepFloor_Y, 0.05], vrep.simx_opmode_oneshot)


		    	# # Calling Function in vrep to Set the Orientation of CB
		    	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "Find_Euler_Angle", [], [CB_Angle], [], "",vrep.simx_opmode_blocking)

			
			break


import sys
import vrep
import time
import math
import serial
from xbee import XBee 
import numpy as np
import cv2
import cv2.aruco as aruco
from ArUco_library import *



Serial_port = serial.Serial('/dev/ttyUSB0',9600)
xbee = XBee(Serial_port)

# # PYTHON CONSTANTS # #
X , Y , Z = 0 , 1, 2


a,b,c,d = 0,0,0,0

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

VideoFrames = cv2.VideoCapture(1)    



def UpdateBotInVrep():

	# Getting a Frame
	IsFrameAvailable, frame = VideoFrames.read()


	# Frames are available
	if IsFrameAvailable == True:

		 

	 # Scaling the Image using the CENTER Coordinates of LeftTOp and RightBottom Aruco Marker
		frame = frame[ int(a) : int(b)  ,  int(c) : int (d) ] 


		
	    # Displaying the Frame
		cv2.imshow('frame1',frame)

	    # Come Out of Loop
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
			return

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
			UpdateBotInVrep()
			return


	   		
	    

	    # Calulating Orientation of each Aruco Marker in ROI
	   	ArucoMarkes_Angle_Dict = Calculate_orientation_in_degree(Detected_ArUco_markers)	      
	   	
	    # Marking the ARUCO Marker in the Image
	   	frame = mark_ArUco(frame,Detected_ArUco_markers,ArucoMarkes_Angle_Dict)						

	    # Showing the Frame with Marker Aruco Markers
		if debug:
			cv2.imshow('frame',frame)        
	
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
			return




	    # Now We will use the Coordinates of CB in Image to Calculate the Actual Position of CB in the Flex.
	    # then Using the Actual Position of CB in FLex , we will map CB position in Vrep Floor
	    # We will Map the Whole Vrep Floor as the ROI

	    # Size of Frame
    		FrameLength = frame.shape[1]      # Frame Length
    		FrameWidth = frame.shape[0]       # Frame Width


	    # Getting Center Coordinates of CB on Image
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



# Function Where all ONE time initialization take place
def Initialization():

	# Getting ObjectHandle of Collector Bot "left_wheel_visible"
	# return[1] Result of the Function Call
	# return[2] Handle of Collector Bot "left_wheel_visible"
	returnCode , leftWheelHandle = vrep.simxGetObjectHandle(clientID,'left_wheel_visible',vrep.simx_opmode_oneshot_wait)
	
	# Getting ObjectHandle of Collector Bot "right_wheel_visible"	
	# return[1] Result of the Function Call
	# return[2] Handle of Collector Bot "right_wheel_visible"
	returnCode , RightWheelHandle = vrep.simxGetObjectHandle(clientID,'right_wheel_visible',vrep.simx_opmode_oneshot_wait)
	
	# Calling a Function "VrepInitialization" Define in the LuaFunction Script
	# This Function calculates and returns the WheelRadius And WheelSeperation of the Collector Bot
	# Parameter[1][1] Handle of Collector Bot "left_wheel_visible" 
	# Parameter[1][2] Handle of Collector Bot "right_wheel_visible" 
	# return[1] Result of the Function Call
	# return[2] List of Floating Points number
		# return[2][1] Collector Bot WheelRadius
		# return[2][2] Collector Bot WheelSeperation
	returnCode , _, returnFloats, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "VrepInitialization", [leftWheelHandle , RightWheelHandle], [], [], "",vrep.simx_opmode_blocking)
	

	# Storing  Collector Bot WheelRadius && Collector Bot WheelSeperation
	WheelRadius = returnFloats[0]
	WheelSeperation = returnFloats[1]


	if debug:
		print WheelRadius
		print WheelSeperation


	# Returing Collector Bot WheelRadius , WheelSeperation
	return WheelRadius , WheelSeperation



# Write a function here to choose a goal.
def ChooseNextTargetCylinder(Fruits):

	# List to Store Fruits => Fruits which are still not accomplished
	RemaningFruits = []

	# Calling a Function "VrepChooseNextTargetCylinder" Define in the LuaFunction Script
	# This Function Finds the Next TargetCylinder and returns its Handle 
	# return[1] Result of the Function Call
	# return[2] List of Integers
		# return[2][1] TargetCylinder Handle
	returnCode , outInts, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "VrepChooseNextTargetCylinder", Fruits, [], [], "",vrep.simx_opmode_blocking)
	
	# Storing TargetCylinder handle 
	TargetCylinder = outInts[0]
	
	# Looping For all the Fruit present in Fruits List
	# Adding all Fruit in Fruits List except the Fruit which is the Current Target
	# Because this Target will be accomplished, so we should remove this Fruit from the Fruit List
	for Fruit in Fruits:

		if Fruit == TargetCylinder:
			# Fruit Matches With The TargetCylinder Do Nothing
			pass

		else:
			# Add this Fruit in the Remainig Fruits List
			RemaningFruits.append(Fruit)


	# Return[1] List of Remaning Fruits
	# Return[2] Handle of TargetCylinder 
	return RemaningFruits , TargetCylinder 
	



# Function to REMOVE current TargetCylinder from Vrep Collection list
def RemoveTargetCylinderFromCollection(TargetCylinder):

	# Calling a Function "AlterCollection" Define in the LuaFunction Script
	# This Function Alter the Vrep Obstacle Collection  
	# Parameter[1][1] Handle of TargetCylinder 
	# Parameter[1][2] Interger, Which indicates Whether To [0]Add or [1]Remove a Object From Collection
				# Here we Remove a Object from Collection
	# return[1] Result of the Function Call
	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "AlterCollection", [TargetCylinder , 1], [], [], "",vrep.simx_opmode_oneshot_wait)
	
	if debug :
		if returnCode == 0 :
			print "Operation Sucessful -- RemoveObjectFromCollection"







# Write a function(s) to set/reset goal and other so that you can iterate the process of path planning
def SetGoalDummyPosition_To_TargetCylinderPosition(TargetCylinder):	

	# Calling a Function "SetGoalDummyPosition_To_TargetCylinderPosition" Define in the LuaFunction Script
	# This Function places the GoalDummy at the Center of the TargetCylinder  
	# Parameter[1][1] Handle of TargetCylinder 
	# return[1] Result of the Function Call
	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "SetGoalDummyPosition_To_TargetCylinderPosition", [TargetCylinder], [], [], "",vrep.simx_opmode_blocking)
	
	if debug :	
		if returnCode == 0:
			print "Operation Sucessful -- Goal Position Set"	




# Function to Adjust Search Parameters (BOX)
def AdjustSearchParameters():

	# Calling a Function "AdjustSearchParameters" Define in the LuaFunction Script
	# This Function Re-Adjust the Seacrh area to be same as the Area of the Floor for the PathPlanning
	# return[1] Result of the Function Call
	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "AdjustSearchParameters", [], [], [], "",vrep.simx_opmode_blocking)



# Write a function to create a path from Start to GoalDummy
def GenerateNewPath():

	# Calling a Function "GenerateNewPath" Define in the LuaFunction Script
	# This Function Create a Path starting From the Start Dummy to the Goal Dummy
	# return[1] Result of the Function Call
	returnCode , returnInts, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "GenerateNewPath", [], [], [], "", vrep.simx_opmode_blocking)

	if debug :
		if returnCode == 0:
			print "Operation Sucessful -- path Generated"





# Write a function to make the robot move in the generated path. 
# Make sure that you give target velocities to the motors here in python script rather than giving in lua.
# Note that the your algorithm should also solve the conditions where partial paths are generated.
def print_data(data):
    """
    This method is called whenever data is received
    from the associated XBee device. Its first and
    only argument is the data contained within the
    frame.
    """
    print data

def MoveRobotToTargetFruit(TargetCylinder):

	# {PathPointer} value is between 0 - 1, and based on this value we calculate a point on the Path Created Above
	# This point act as temporary Destination for the Collector Bot 
	PathPointer = 0

	# {DistanceBetweenRobotAndPointOnPath} stores the Distance between the Colletor Bot and the Point on Path Created Above
	DistanceBetweenRobotAndPointOnPath = 0

	# {IsTargetAccomplished} is set to 1 if the Collector Bot is inside Vicinity i.e Goal is accomplished
	IsTargetAccomplished = 0

	# Getting ObjectHandle of Disc Of the Current TargetCylinder
	# return[1] Result of the Function Call
	# return[2] Handle of Disc of the Current TargetCylinder
	returnCode, DiscHandle = vrep.simxGetObjectChild( clientID, TargetCylinder, 0, vrep.simx_opmode_blocking)
	

	ThresholdAngle = 10



	# Loops 
	while True:

		# Calling a Function "MoveRobotOnNewPath" Define in the LuaFunction Script
		# This Function makes the Collector Bot to move on the path Created
		# Parameter[1][1] Handle Disc of Current TargetCylinder
		# Parameter[1][2] PathPointer Variable which points on a point on the path 
		# return[1] Result of the Function Call
		# return[2] List of Integers
			# return[2][1] a value 0 or 1 which tells whether Collector Bot is inside Vicinity Area or Not
		# return[3] List of floating points numbers
			# return[3][1] Distance Between Collector bot And PointOnPath
			# return[3][2] Angle Between Collector bot And PointOnPath
		returnCode , returnInts, returnFloats, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "MoveRobotToTargetFruit", [DiscHandle], [PathPointer], [], "",vrep.simx_opmode_blocking)
		
		# Storing result whether IsInsideVicinityArea or not
		IsInsideVicinityArea = returnInts[0]

		if(IsInsideVicinityArea == 1):
			IsTargetAccomplished = 1

		if(IsTargetAccomplished == 1):
			PathPointer = 0


			if debug :
				print "break Loop"

			# Value is set to 1 if the Collector bot is insideVicinity Area
			break
			# Break out of the Loop
			

	
		DistanceBetweenRobotAndPointOnPath = returnFloats[0]
		AngleBetweenRobotAndPointOnPath = returnFloats[1]
		AngleBetweenRobotAndPointOnPath = AngleBetweenRobotAndPointOnPath * 180.0 / math.pi
		AngleBetweenRobotAndPointOnPath = AngleBetweenRobotAndPointOnPath % 360

		if( abs(AngleBetweenRobotAndPointOnPath) > 25.0 ):

			if(AngleBetweenRobotAndPointOnPath > 0):
				Serial_port.write( "A" + "+" + str(int(AngleBetweenRobotAndPointOnPath)) + "*")
			else:
				Serial_port.write( "A" + str(int(AngleBetweenRobotAndPointOnPath)) + "*")
			# xbee.wait_read_frame()
			time.sleep(2)
			 
		

		# Storing Distance Between Cylinder And CollectorBot
		DistanceBetweenCylinderAndRobot = returnFloats[2]


		Serial_port.write("T")
		# xbee.wait_read_frame()
		time.sleep(5)
		
		
		# wait 


		#  Aruco 

		UpdateBotInVrep()





		if(DistanceBetweenRobotAndPointOnPath < 0.9):
			PathPointer = PathPointer + 0.01




		# Handle Partial Path Generated
		# if path is partial i.e Collector Bot cannot move to the Vicinity
		# we traverse the path to the end
		if(PathPointer >= 1):

			# Then we again calculate the path
			PathPointer = 0
			AdjustSearchParameters()
			GenerateNewPath()
			DistanceBetweenRobotAndPointOnPath = 0
			IsTargetAccomplished = 0






		
			



# Function for Setting the StartDummy Coordinates same as that of Collector Bot
def SetStartDummyPosition_To_CollectorBotPosition():

	# Calling a Function "SetStartDummyPosition_To_CollectorBotPosition" Define in the LuaFunction Script
	# This Function places the StartDummy at the Center of the CollectorBot  
	# return[1] Result of the Function Call
	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "SetStartDummyPosition_To_CollectorBotPosition", [], [], [], "",vrep.simx_opmode_blocking)

	if debug :
		if returnCode == 0:
			print "Operation Sucessful -- SetStartDummyPosition_To_CollectorBotPosition"


# Function to Add current Target to Collection list
def AddTargetCylinderToCollection(TargetCylinder):

	# Calling a Function "AlterCollection" Define in the LuaFunction Script
	# This Function Alter the Vrep Obstacle Collection  
	# Parameter[1][1] Handle of TargetCylinder 
	# Parameter[1][2] Interger, Which indicates Whether To [0]Add or [1]Remove a Object From Collection
				# Here we Add a Object to Collection
	# return[1] Result of the Function Call
	returnCode , _, _, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "AlterCollection", [TargetCylinder , 0], [], [], "",vrep.simx_opmode_blocking)
	
	if debug :
		if returnCode == 0:
			print "Operation Sucessful -- AddCylinderToCollection"
	    







################ Initialization of handles. Do not change the following section ###################################

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
	print "connected to remote api server"
else:
	print 'connection not successful'
	sys.exit("could not connect")

returnCode = vrep.simxStartSimulation(clientID , vrep.simx_opmode_oneshot)

returnCode,robot_handle=vrep.simxGetObjectHandle(clientID,'CollectorBot',vrep.simx_opmode_oneshot_wait)
returnCode,leftjoint_handle=vrep.simxGetObjectHandle(clientID,'left_joint',vrep.simx_opmode_oneshot_wait)
returnCode,rightjoint_handle=vrep.simxGetObjectHandle(clientID,'right_joint',vrep.simx_opmode_oneshot_wait)
returnCode,start_dummy_handle = vrep.simxGetObjectHandle(clientID,'Start',vrep.simx_opmode_oneshot_wait)
returnCode,goal_dummy_handle = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_oneshot_wait)



returnCode,Fruit1=vrep.simxGetObjectHandle(clientID,'Fruit1',vrep.simx_opmode_oneshot_wait )
# returnCode,Fruit2=vrep.simxGetObjectHandle(clientID,'Fruit2',vrep.simx_opmode_oneshot_wait )
# returnCode,Fruit3=vrep.simxGetObjectHandle(clientID,'Fruit3',vrep.simx_opmode_oneshot_wait )
# returnCode,Fruit4=vrep.simxGetObjectHandle(clientID,'Fruit4',vrep.simx_opmode_oneshot_wait )
# returnCode,DamagedFruit1=vrep.simxGetObjectHandle(clientID,'DamagedFruit1',vrep.simx_opmode_oneshot_wait )
# returnCode,DamagedFruit2=vrep.simxGetObjectHandle(clientID,'DamagedFruit2',vrep.simx_opmode_oneshot_wait )
# returnCode,DamagedFruit3=vrep.simxGetObjectHandle(clientID,'DamagedFruit3',vrep.simx_opmode_oneshot_wait )
# returnCode,DamagedFruit4=vrep.simxGetObjectHandle(clientID,'DamagedFruit4',vrep.simx_opmode_oneshot_wait )




# Getting Object Handle of ResizableFloor_5_25 in Vrep
returnCode, Vrep_Floor = vrep.simxGetObjectHandle( clientID, "ResizableFloor_5_25_element", vrep.simx_opmode_blocking )



# Getting Dimension of ResizableFloor_5_25_element

# Getting X Length in Left side of Origin
returnCode, Min_X = vrep.simxGetObjectFloatParameter( clientID, Vrep_Floor, OBJECT_BOUNDING_BOX_MIN_X , vrep.simx_opmode_blocking)

# Getting X Length in Right side of Origin
returnCode, Max_X = vrep.simxGetObjectFloatParameter( clientID, Vrep_Floor, OBJECT_BOUNDING_BOX_MAX_X , vrep.simx_opmode_blocking)

# Calculating Length of ResizableFloor_5_25
Vrep_ResizeableFloor_Length = (Max_X - Min_X ) 



# Getting Y Width in Down side of Origin
returnCode, Min_X = vrep.simxGetObjectFloatParameter( clientID, Vrep_Floor, OBJECT_BOUNDING_BOX_MIN_Y , vrep.simx_opmode_blocking)

# Getting Y Width in Up side of Origin
returnCode, Max_X = vrep.simxGetObjectFloatParameter( clientID, Vrep_Floor, OBJECT_BOUNDING_BOX_MAX_Y , vrep.simx_opmode_blocking)

# Calculating Width of ResizableFloor_5_25
Vrep_ResizeableFloor_Width = (Max_X - Min_X ) 

#####################################################################################################################


def MapToVrepFloor(XCoord, YCoord):
	# Calculating the Position of CB in Vrep , by using the Actual position of CB on Flex Sheet

	# X Coordinate of CB on Vrep Floor
	XCoordOnVrepFloor = XCoord / FLEX_ORIGINAL_LENGTH * Vrep_ResizeableFloor_Length

	# Adjusting the X Coordinate of CB on Vrep Floor, since Vrep Coordiantes are from -x to +x
	XCoordOnVrepFloor =  XCoordOnVrepFloor - (Vrep_ResizeableFloor_Length / 2)


	# Y Coordinate of CB on Vrep Floor
	YCoordOnVrepFloor = YCoord / FLEX_ORIGINAL_WIDTH * Vrep_ResizeableFloor_Width

	# Adjusting the Y Coordinate of CB on Vrep Floor, since Vrep Coordiantes are from -y to +y
	YCoordOnVrepFloor =  (Vrep_ResizeableFloor_Width / 2) - YCoordOnVrepFloor 

	return XCoordOnVrepFloor, YCoordOnVrepFloor


def Vrep_Setup(ListOfFruits, ListOfDamagedFruits):

	for Fruit in xrange(0,len(ListOfFruits)):
		XCoord = (0.06 + 0.04) * (ord(ListOfFruits[Fruit][-1:]) - ord("A"))
		YCoord = (0.06 + 0.05) * (int(ListOfFruits[Fruit][:-1]))

		# print XCoord, YCoord
		XCoord, YCoord = MapToVrepFloor(XCoord, YCoord)
		print XCoord, YCoord

		# Placing the Fruits on Vrep floor
		returnCode = vrep.simxSetObjectPosition(clientID, Fruits[Fruit], WORLD_REFERENCE, [XCoord,YCoord, 0.05], vrep.simx_opmode_oneshot)


	for DamagedFruit in xrange(0,len(ListOfDamagedFruits)):
		XCoord = (0.06 + 0.04) * (ord(ListOfDamagedFruits[DamagedFruit][-1:]) - ord("A"))
		YCoord = (0.06 + 0.05) * (int(ListOfDamagedFruits[DamagedFruit][:-1]))

		# print XCoord, YCoord
		XCoord, YCoord = MapToVrepFloor(XCoord, YCoord)
		# print XCoord, YCoord

		# Placing the Fruits on Vrep floor
		returnCode = vrep.simxSetObjectPosition(clientID, DamagedFruits[DamagedFruit], WORLD_REFERENCE, [XCoord,YCoord, 0.05], vrep.simx_opmode_oneshot)


# Write your code here

# Fruits = [ Fruit1, Fruit2, Fruit3, Fruit4]
Fruits = [ Fruit1]
# DamagedFruits = [ DamagedFruit1, DamagedFruit2, DamagedFruit3, DamagedFruit4]



# ListOfFruits = ["17A", "8P", "10I", "11R"]
# ListOfDamagedFruits = ["7F", "10M", "4H", "8T"]

ListOfFruits = ["13R"]
ListOfDamagedFruits = []


Vrep_Setup(ListOfFruits, ListOfDamagedFruits)

WheelRadius , WheelSeperation =  Initialization()

def GetImageScale():
	IsImageScaled = False
	while  IsImageScaled == False:
		
		IsFrameAvailable, frame = VideoFrames.read()

		if IsFrameAvailable == True:
			cv2.imshow('frame',frame)  

			if cv2.waitKey(1) & 0xFF == ord('q'):
				print "dasd\n\n"


			Detected_ArUco_markers = detect_ArUco(frame)


		try:
			LeftTopArucoMarker = Detected_ArUco_markers [ LEFT_TOP_ARUCO_MARKER_ID ]
			print "LeftDone"	
			# Getting RightBottomArucoMarker Coordinates
			RightBottomArucoMarker = Detected_ArUco_markers [ RIGHT_BOTTOM_ARUCO_MARKER_ID ]
			print "RightDone"
			IsImageScaled = True
 
	
		
		except Exception as e:
			print "Ecev"
			continue

	    # Getting Center Coordinates of LeftTop and RightBottom Aruco Markers
		ArucoMarkers_Centers_List = GetCenterCoordinates([ LeftTopArucoMarker , RightBottomArucoMarker ])

	    # Storing Center Coordinates of LeftTop and RightBottom Aruco Markers
	   	__LeftTopArucoMarkerCenter , __RightBottomArucoMarkerCenter = ArucoMarkers_Centers_List[0] , ArucoMarkers_Centers_List[1]

	   	frame = frame[ int(__LeftTopArucoMarkerCenter[1]) : int(__RightBottomArucoMarkerCenter[1])  ,  int(__LeftTopArucoMarkerCenter[0]) : int (__RightBottomArucoMarkerCenter[0]) ] 
		a = __LeftTopArucoMarkerCenter[1]
		b = __RightBottomArucoMarkerCenter[1]
		c = __LeftTopArucoMarkerCenter[0]
		d = __RightBottomArucoMarkerCenter[0]

		return a,b,c,d






a,b,c,d = GetImageScale()
print "Done"

# looping Fruits List , untill all Targets are Accomplished
while Fruits:

	# Choosing Next Cylinder as Target
	# Parameter[1] List of Fruits (Target we have to achieve)
	# Return[1]  List of Remaining Fruits
	# Return[2]  Current TargetCylinder, Which we have to achieve
	Fruits , TargetCylinder = ChooseNextTargetCylinder(Fruits)


	# Removing Current TargetCylinder From the Vrep-Collection, so that Vrep do not Consider Current TargetCylinder as a Obstacle
	# Parameter[1] Current TargetCylinder Handle
	RemoveTargetCylinderFromCollection(TargetCylinder)



	# Setting GoalDummyPosition  same as TargetCylinderPosition
	# Parameter[1]  Current TargetCylinder Handle
	SetGoalDummyPosition_To_TargetCylinderPosition(TargetCylinder)


	# Re-Adjusting Search Parameter Area
	# This is Necessary because as the Associated start Dummy Moves the Search Area also changes relative to the start dummy 
	AdjustSearchParameters()


	# Generating a path from the Collector Bot to the Current TargetCylinder
	GenerateNewPath()

	# Function which makes the Collector bot Follow the Path Generated by the Vrep
	# Parameter[1] Current Target Cylinder
	MoveRobotToTargetFruit(TargetCylinder)


	# Function for Setting the StartDummy Coordinates same as that of Collector Bot
 	SetStartDummyPosition_To_CollectorBotPosition()



	# After Collector Bot has Accomplished the Current Target , We should add The Current TargetCylinder to the Collection list, so that in the process of finding a path to next TargetCylinder , Vrep Consider Current TargetCylinder as a obstacle 
	AddTargetCylinderToCollection(TargetCylinder)







#end of simulation


VideoFrames.release()

# Closing All Windows
cv2.destroyAllWindows()


vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
time.sleep(0.1)

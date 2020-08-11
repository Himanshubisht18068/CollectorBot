## Task 1.2 - Path Planning in V-REP ##
# Import modules

import sys
import vrep
import time
import math


# Set this variable to TRUE, if want to run in Debug Mode
debug = False


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

def MoveRobotOnNewPath(TargetCylinder):

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
	
	# {DesireVelocity} is used for calculating the velocity of the wheels of the Collector Bot
	DesireVelocity = 0

	# A variable which Stores the Threshold Angle
	# If the Angle between the Robot and the pointonPath is Greater than this value
	# then Collector Bot DoesNot Move
	# It Only Rotates and comes on the path
	ThresholdDegree = 20


	# Maximum and the Minimum velocity, Collector Bot can have
	MaxVelocityOfTheCollectorBot = 0.2
	MinVelocityOfTheCollectorBot = 0

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
		returnCode , returnInts, returnFloats, _, _ = vrep.simxCallScriptFunction(clientID, "LuaFunctions", 1, "MoveRobotOnNewPath", [DiscHandle], [PathPointer], [], "",vrep.simx_opmode_blocking)
		
		# Storing result whether IsInsideVicinityArea or not
		IsInsideVicinityArea = returnInts[0]
	

		try:
			# Storing Distance Between Collector Bot And PointOnPath
			DistanceBetweenRobotAndPointOnPath = returnFloats[0]
		except:
			DistanceBetweenRobotAndPointOnPath = 0

		try:
			# Storing Angle Between Collector Bot And PointOnPath
			AngleBetweenRobotAndPointOnPath = returnFloats[1]
		except:
			AngleBetweenRobotAndPointOnPath = 0

		
		# Storing Distance Between Cylinder And CollectorBot
		DistanceBetweenCylinderAndRobot = returnFloats[2]


		# Here we Calculate the Linear and angular velocity of the Wheels of the Collector bot
		# And Makes the Collector Bot Move



		# Here we basically moves Collector Bot, similarly as we Drive a Car
		# if path is Straight , we Accelerate
		# if path is Curved , we retardate , Moves with small speed		

		# and if the Collector Bot is about to reach the destination, we decrease Collector Bot speed
		# so that we have smooth stopping



		if(IsInsideVicinityArea == 0):
			# Collector Bot is Still not inside the vicinity area


			if abs(AngleBetweenRobotAndPointOnPath * 180 / math.pi) > ThresholdDegree :
				# Angle Between Robot And PointOnPath is Greater than Threshold Angle
				# That means path too curved
				# Reduce the Velocity

				if(DesireVelocity > MinVelocityOfTheCollectorBot):
					# if Velocity is not zero 
					# produce a Retarding acceleration

					# Reducing the Velocity by 0.025
					DesireVelocity = DesireVelocity - 0.025

				else:
					DesireVelocity = 0 # just for safety so that velocity dont get negative

			else: 

				# Angle Between Robot And PointOnPath is less than Threshold Angle
				# That means path quite straight
				

				# Here we Check if the distance Between Collector Bot and Vicinity is Large
				# Then Collector Bot can accelerate

				if DistanceBetweenCylinderAndRobot > 0.025 : # Distance after which we should continoulsy apply retardation
					# Collector bot is far from  the Cylinder , we can accelearte

					# Increase the Velocity
					if DesireVelocity < MaxVelocityOfTheCollectorBot : # Still Not TopSpeed
						# if Velocity is less than max Velocity
						# produce acceleration

						# increasing the Velocity by 0.025
						DesireVelocity = DesireVelocity + 0.025

					else:
						DesireVelocity = MaxVelocityOfTheCollectorBot # for safety so that velocity dont get bigger 

				else:

					# Collector Bot is about to reach the Vicinity

					# Then we Should start decreasing Collector Bot Speed

					if DesireVelocity > 0.1: # 0.1 Is Breaking Speed, from which if Speed Suddenly becomes 0 , no jerk on collector bot 
						DesireVelocity = DesireVelocity - 0.025
					else:
						DesireVelocity = 0.1


			# Setting the Desire Angular Speed proortional to Angle Between Collector And PointOnPath
			DesireAngularSpeed = 1.0 * AngleBetweenRobotAndPointOnPath

		else:
			# Collector Bot is Inside Vicinity Area
			DesireVelocity = DesireAngularSpeed = 0

			# Task Completed
			# Setting value of IsTargetAccomplished to 1 , in order to break out of loop
			IsTargetAccomplished = 1


		# Setting VelocityOfRightWheel and VelocityOfLeftWheel of the Collector Bot
		VelocityOfRightWheel = DesireVelocity + (WheelSeperation/2) * DesireAngularSpeed
		VelocityOfLeftWheel = DesireVelocity - (WheelSeperation/2) * DesireAngularSpeed

	  	
		# Setting AngularVelocityOfRightWheel and AngularVelocityOfLeftWheel of the Collector Bot
		AngularVelocityOfRightWheel = VelocityOfRightWheel / WheelRadius
		AngularVelocityOfLeftWheel = VelocityOfLeftWheel / WheelRadius


		# Setting rotational speed of the Joints(MOTORS) in the Collector Bot
		vrep.simxSetJointTargetVelocity(clientID , leftjoint_handle ,AngularVelocityOfLeftWheel , vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(clientID ,rightjoint_handle ,AngularVelocityOfRightWheel ,vrep.simx_opmode_oneshot)

		# if the Collector bot has reached the temporary destination point on the path
		# Shift the point forward by 0.1
		if(DistanceBetweenRobotAndPointOnPath < 0.1):
			PathPointer = PathPointer + 0.01

		print PathPointer



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






		if(IsTargetAccomplished == 1):
			PathPointer = 0
			# Value is set to 1 if the Collector bot is insideVicinity Area
			break
			# Break out of the Loop
			

			if debug :
				print "break Loop"
			


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
	    




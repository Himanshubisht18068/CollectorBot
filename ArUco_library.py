############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math


def detect_ArUco(img):
    ## function to detect ArUco markers in the image using ArUco library
    ## argument: img is the test image
    ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    ##         for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    ##              {0: array([[315, 163],
    #                           [319, 263],
    #                           [219, 267],
    #                           [215,167]], dtype=float32)}


    

    Detected_ArUco_markers = {}
    ## enter your code here ##

    grayImage = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    ## Reading CornerPoints and Ids of the Marker from the Given Image
    CornerPoints , Ids , _ = aruco.detectMarkers(grayImage,aruco_dict,parameters = parameters)

    counter = 0; 
    # for accessing  next matrix


    try:
    # Since Sometimes Aruco is not able to read the Ids from the given Image due to some orientation problem or anyother issue
    # We read Id from Ids in the Try Block
        for Id in Ids:
        #Looping for All the Ids present in the Image            
            Detected_ArUco_markers[Id[0]] = CornerPoints[counter][0]
            # storing Each ID along with its Corners in the Detected_ArUco_markers dictionary

            counter = counter + 1
            # increment $counter for accessing the next Id
    except:
        # If their is Exception i.e Aruco Library failed to read the Id from the Image 
        # we Simply  return the Detected_ArUco_markers dictionary, without any Ids and Corners
        pass

    return Detected_ArUco_markers



def GetCenterCoordinates(Aruco_Markes_List):
    ## function to Find Center Coordinates of ArUco markers in the List 
    ## argument: Aruco_Markes_List is the List of Aruco_Markes Coordiantes
    ## return: Array named Aruco_Markes_Center 
    

    Aruco_Markes_Center = []

    for Marker in Aruco_Markes_List:     
        # Looping for all Marker Coordinates List

        # Getting TopLeft X,Y Coordinate List
        topLeft = Marker[0]

        # Getting BottomRight X,Y Coordinate List
        bottomRight = Marker[2]

        # Calulating and Storing Center Coordinates in the List
        Aruco_Markes_Center.append((topLeft + bottomRight) / 2 )

    # returning the List of Aruco Marker Center
    return Aruco_Markes_Center



def Remove_ArucoMarker_Not_In_ROI(Detected_ArUco_markers , LeftTopArucoMarkerCenter , RightBottomArucoMarkerCenter):
    ## function to Remove un-necessary Aruco Marker in Detected_ArUco_markers Dictionary
    ## argument: 
                # Detected_ArUco_markers ---> Dictionary of Aruco Markers
                # LeftTopArucoMarkerCenter --> List of X,Y Coordinate of LeftTopArucoMarkerCenter
                # RightBottomArucoMarkerCenter --> List of X,Y Coordinate of RightBottomArucoMarkerCenter

    ## return: Dictionary  named Detected_ArucoMarker_Inside_ROI which contain the Aruco Markers inside ROI
    

    ## Here we check if the all the 4 corner Coordinate of Aruco Marker lies between LeftTop and RightBottom Center of Corner Aruco Maker 
    ## if so we save it in the Dictionary
    ## otherwise we reject that aruco marker

    NumberOfCorners = 4
    NumberOfCoordinates = 2

    flag = 1            # Flag = 1 , means store the Aruco in Dictionary
    Detected_ArucoMarker_Inside_ROI = {}

    for Marker in Detected_ArUco_markers:
        # Selecting a Marker

        for Corners in xrange(0,NumberOfCorners):
            # Looping for all 4 Corner Coordinates
            
            for Coordinates in xrange(0,NumberOfCoordinates): 
                # Looping for both X and Y Coordinate of a Corner Point
       
                if Detected_ArUco_markers[Marker][Corners][Coordinates] < LeftTopArucoMarkerCenter[Coordinates] or Detected_ArUco_markers[Marker][Corners][Coordinates] > RightBottomArucoMarkerCenter[Coordinates] :

                    # Some Corner Coordinate Current Aruco Marker lies Outside the ROI


                    flag = 0    # set Flag = 0, 
                    break;      

            if flag == 0:
                break;
        
        if flag == 1: # if Flag =1 Store the Cuurent ARUCO marker in The Dictionary
            Detected_ArucoMarker_Inside_ROI[Marker] = Detected_ArUco_markers[Marker]

    # return Detected_ArucoMarker_Inside_ROI dictionary
    return Detected_ArucoMarker_Inside_ROI 




def ResetCoordinates_WRT_LeftTopArucoMargin(Detected_ArUco_markers , LeftTopArucoMarkerCenter):
    ## function to Reset the Coordinate of Aruco Marker in Detected_ArUco_markers Dictionary WRT LeftTopArucoMarkerCenter
    ## argument: 
                # Detected_ArUco_markers ---> Dictionary of Aruco Markers
                # LeftTopArucoMarkerCenter --> List of X,Y Coordinate of LeftTopArucoMarkerCenter

    ## return: Dictionary  named Detected_ArUco_markers which contain the updated Aruco Markers Coordinates 



    NumberOfCorners = 4
    NumberOfCoordinates = 2


    for Marker in Detected_ArUco_markers:
        # Selecting a Marker

        for Corners in xrange(0,NumberOfCorners):
            # Looping for all 4 Corner Coordinates
            
            for Coordinates in xrange(0,NumberOfCoordinates): 
                # Looping for both X and Y Coordinate of a Corner Point

                Detected_ArUco_markers[Marker][Corners][Coordinates] = Detected_ArUco_markers[Marker][Corners][Coordinates] - LeftTopArucoMarkerCenter[Coordinates]
                # New Coordianate = Old Coordiante - Coordianate of Left Top Marker

    # return Detected_ArucoMarker
    return Detected_ArUco_markers    
            





def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in Problem_Statement.pdf
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the ProblemStatement.pdf)
    ##          for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
    ##          function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}
    ## enter your code here ##

    for Marker in Detected_ArUco_markers:
        # for Each marker present in the Detected_ArUco_markers dictionary
        # we calculate the topleft , topRight , bottomRight and Bottomleft corners Coordinates
        topLeft = Detected_ArUco_markers[Marker][0]
        topRight = Detected_ArUco_markers[Marker][1]
        bottomRight = Detected_ArUco_markers[Marker][2]
        bottomLeft = Detected_ArUco_markers[Marker][3]


        # Calculating TopCenter and MarkerCenter Coordinates

        # Calculating TopCenter Coordinates
        # TopCenter is halfway between TopLeft and TopRight Coordinates
        # Using the Mid Point Theorem
        topCenter =  (topLeft + topRight) / 2

        # Calculating MarkerCenter Coordinates
        # MarkerCenter is halfway Between , the center of any 2 opposites corners Coordinates of the Marker
        # using the Mid Point Theorem
        markerCenter = (topLeft + bottomRight) / 2 

        # Generating a Vector using from MarkerCenter to Marker TopCenter coordinates
        # Vector from MarkerCenter to Topcenter using 2 points is point2 - point1
        vecEqOfLine = topCenter - markerCenter

        # calculate angle between two Vectors
        # First Vector is the $vecEqOfLine we Just calculated
        # Second Vector is the X-axis (y = 0)
        # Angle between two Vectors is :  = acos ( (vector1 * vector2) / ( |vector1| *  | vector2| ))
        angle = (vecEqOfLine[0] * 1) / ( math.sqrt( math.pow(vecEqOfLine[0] ,2) + math.pow(vecEqOfLine[1] ,2) ) ) 
        angle = math.acos(angle) # angle is in radians
        angle = angle * 180 / math.pi # converting the angle in Degrees
        # angle = int(angle)



        # Since cos@ always returns the acute angle between 2 vectors
        # We manually calculate , in which Quadrant does the angle lies or LineVector is present
        # if angle lies in the Quadrant 3 or 4 , we manually determine the anticlockwsie angle by subtracting the  Calulated angle from 360
        # because math.acos always returns the acute angle 

        # Checking if the X coordiantes of the topCenter is Greater than the X Coordinate of MarkerCenter
        # Than the quadrant must be either FIRST OR FOURTH Quadrant
        # else Quadrant must be either SECOND OR THIRD Quadrant
        if(topCenter[0] >= markerCenter[0]):
            # Quadrant must be either FIRST OR FOURTH Quadrant

            if(topCenter[1] > markerCenter[1]):
                # if Y Coordinate of the TopCenter is Greater than the Y Coordinate of the Marker Center
                # than Quadrant must be FOURTH

                # Since Quadrant is FOURTH , acos@ must have return the clockwise angle measure
                # but we want anticlockwise measure
                # To Measure the angle in anticlockwise , we Must Subtract $angle from 360 degree
                angle = 360 - angle # Converting clockwise angle measure to anticlockwise 
        else:
            # Quadrant must be either SECOND OR THIRD

            if(topCenter[1] > markerCenter[1]):
                # if Y Coordinate of the TopCenter is Greater than the Y Coordinate of the Marker Center
                # than Quadrant must be THIRD

                # Since Quadrant is THIRD , acos@ must have return the clockwise angle measure
                # but we want anticlockwise measure
                # To Measure the angle in anticlockwise , we Must Subtract $angle from 360 degree
                angle = 360 - angle # Converting clockwise angle measure to anticlockwise 

        # Storing the Marker ID and its Angle in $ArUco_marker_angles
        ArUco_marker_angles[Marker] = angle

    return ArUco_marker_angles 


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
    ## function to mark ArUco in the  image 
    ## arguments: img is the test image 
    ##            Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    ##            ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    ## return: image namely img after marking the aruco as per the instruction given in Problem_statement.pdf

    ## enter your code here ##

    dotSize = 2 # Dots Radius
    lineWidth = 2 # Line Width
    offset = 20 # Offset for starting position of the TEXT (Marker ID and Marker angle) from the center of the Marker


    for Marker in Detected_ArUco_markers:
        topLeft = Detected_ArUco_markers[Marker][0]
        topRight = Detected_ArUco_markers[Marker][1]
        bottomRight = Detected_ArUco_markers[Marker][2]
        bottomLeft = Detected_ArUco_markers[Marker][3]

        topCenter = (topLeft + topRight) / 2
        markerCenter = (topLeft + bottomRight) / 2 

        # # Marking the TopLeft Dot
        # cv2.circle( img , ( topLeft[0] , topLeft[1] ) , dotSize , (125,125,125) , -1 ) # TopLeft DOT

        # # Marking the TopRight Dot
        # cv2.circle( img , ( topRight[0] , topRight[1] ) , dotSize , (0  ,255,  0) , -1 ) #TopRight DOT

        # # Marking the BottomRight Dot
        # cv2.circle( img , (bottomRight[0] ,bottomRight[1] ) , dotSize , (180,105,255) , -1 ) #bottomRight DOT

        # # Marking the BottomLeft Dot
        # cv2.circle( img , (bottomLeft[0] ,bottomLeft[1] ) , dotSize , (255,255,255) , -1 ) #bottomLeft DOT

        # # Marking the Center Dot
        # cv2.circle(img , (markerCenter[0] , markerCenter[1] ) , dotSize ,(0,0,255) ,-1 ) ## center DOT

        # # Drawing the line from MarkerCenter to Marker TopCenter
        # cv2.line( img , (markerCenter[0] , markerCenter[1] ) , (topCenter[0] , topCenter[1]) , (255,0,0) ,lineWidth) # Line From Center to TopCenter
        
        # # Writing Marker ID on the Marker
        # cv2.putText(img ,str(Marker) ,(int(markerCenter[0]) + offset , int(markerCenter[1]) ), cv2.FONT_HERSHEY_SIMPLEX ,.75,(0,0,255) ,2 ) 

        # # Writing Marker Angle on the Marker
        cv2.putText(img ,str(ArUco_marker_angles[Marker]) ,( int(markerCenter[0]) - offset, int(markerCenter[1]) ), cv2.FONT_HERSHEY_SIMPLEX ,.75,(0,255,0),2 )
    

        # # Coordinates of Center
        # cv2.putText(img ,str(markerCenter) ,(int(markerCenter[0]) - 2*offset , int(markerCenter[1]) ), cv2.FONT_HERSHEY_SIMPLEX ,.75,(0,0,255) ,2 ) 

    return img 



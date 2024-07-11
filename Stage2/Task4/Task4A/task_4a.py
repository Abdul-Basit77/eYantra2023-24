'''
*****************************************************************************************
*
*        		===============================================
*           		Geo Guide (GG) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 4A of Geo Guide (GG) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ GG3344 ]
# Author List:		[ Abhay Agrawal, Abdul Basit]
# Filename:			task_4a.py


####################### IMPORT MODULES #######################
import cv2
import numpy as np
import time

from sys import platform
import subprocess       # OpenCV Library
import shutil
import ast
import sys
import os

from tensorflow.keras.models import load_model
import tensorflow as tf
import tensorflow_hub as hub                    
##############################################################

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

detected_event=[]
detected_images=[]
classification_number=0

combat = "combat"
rehab = "human_aid_rehabilitatiion"
military_vehicles = "military_vehicles"
fire = "fire"
destroyed_building = "destroyed_building"

################# ADD UTILITY FUNCTIONS HERE #################

"""
You are allowed to add any number of functions to this code.
"""

#Returning the specified class relative to predicted class.
def event_name(event_key):
    #2fire,4militaryvehicles,3aid,0combat,1destroyed
    name_dict={0:combat,1:destroyed_building,2:fire,3:rehab,4:military_vehicles}
    return name_dict[event_key]

# Event Detection
def classify_event():
    for image in detected_images:
        brighteness_factor = 40
        image = np.clip(image.astype(int)+brighteness_factor,0,255).astype(np.uint8)
        image=cv2.resize(image,(224,224))
        normalized_image=image/255
        CNN_model = load_model(os.path.join('','resizemodel3000.h5'),custom_objects={'KerasLayer':hub.KerasLayer})
        event_key=np.argmax(CNN_model.predict(np.expand_dims(normalized_image, 0)))
        event = event_name(event_key)
        detected_event.append(event)
    return

#detecting Images contours in  the arena
def detect_events(transformed_frame):
    # Define a brightness adjustment factor (negative for decrease)
    detected_images.clear()
    brightness_factor = -0
    # Adjust the pixel values
    darkened_image = np.clip(transformed_frame.astype(int) + brightness_factor, 0, 255).astype(np.uint8)
    
    gray_image = cv2.cvtColor(darkened_image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (3,3), 0)
    # blurred_image = cv2.GaussianBlur(blurred_image, (1,1), 0)
    
    # Create CLAHE object
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    # Apply CLAHE
    clahe_image = clahe.apply(blurred_image)
    # gray_image = cv2.equalizeHist(gray_image)
    # blurred_image = cv2.GaussianBlur(clahe_image, (5,5), 0)
    _, threshold_image = cv2.threshold(clahe_image, 220, 255, cv2.THRESH_TOZERO)
    #cv2.imshow("thresh",threshold_image)

    contours, _ = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #Searching the event boxes by removing the aruco contours from detected contours
    event_coordinates=[]
    for cnt in contours:
        if (5800 < cv2.contourArea(cnt) < 12000):
            x, y, w, h = cv2.boundingRect(cnt)
            if w/h>0.9 and w/h<1.1:
                event_coordinates.append([x,y])
                roi=transformed_frame[y:y+h, x:x+w]
                detected_images.append(roi)
                cv2.rectangle(transformed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #returning after updating the detected event images to camera feed
    return len(detected_images),event_coordinates

    
def camera(frame):
    corners, ids, _ = detector.detectMarkers(frame)
    desired_ids = [4,5, 6, 7]
    centroids = {}
    ends_found = 0
    centroidslist=[]
    for i, marker_id in enumerate(ids):
        if marker_id in desired_ids:
            index = np.where(ids == marker_id)[0][0]  # Find the index of the marker in the list
            # Calculate the centroid of the marker
            corners_of_marker = corners[index][0]
            centroids[int(marker_id[0])] = (int(np.mean(corners_of_marker[:,0], axis=0)),int(np.mean(corners_of_marker[:,1], axis=0)))
            centroidslist.append([int(np.mean(corners_of_marker[:,0], axis=0)),int(np.mean(corners_of_marker[:,1], axis=0))])
            ends_found +=  1
    #Checking whether 4 corners detected or not to tranform the arena
    #print(f"Corners detected: {ends_found}")
    if ends_found<4:
        return
    # Transforming the arena wrt four corners and cropping it to four corners
    pts1 = np.float32(centroidslist)
    pts2=np.float32([[0,800],[800,800],[0,0],[800,0]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    transformed_frame = cv2.warpPerspective(frame,M,(800,800))
    # transformed_frame=frame
    number_of_event,event_coordinates=detect_events(transformed_frame)
    #print(number_of_event)
    # For the first run when all 5 events are detected classifying the events.
    global classification_number
    if classification_number==0 and number_of_event==5:
        classify_event()
        print(detected_event)
        classification_number=1
        pass
    if detect_events is not None and number_of_event==5:
        k=0
        for event in detected_event:
            x,y=event_coordinates[k]
            k+=1
            cv2.putText(transformed_frame,event,(x-10,y-20),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        
    cv2.imshow("Transformed_frame",transformed_frame)  


##############################################################


def task_4a_return():
    """
    Purpose:
    ---
    Only for returning the final dictionary variable
    
    Arguments:
    ---
    You are not allowed to define any input arguments for this function. You can 
    return the dictionary from a user-defined function and just call the 
    function here

    Returns:
    ---
    `identified_labels` : { dictionary }
        dictionary containing the labels of the events detected
    """  
    identified_labels = {}  
    
##############	ADD YOUR CODE HERE	##############
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,2000)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,2000)
    print("starting")
    while True:
        ret, frame = cap.read()
        copy_frame=frame.copy()
        # cv2.imshow('Detected Events', cv2.resize(frame,(224,224)))
        #time.sleep(20)
        camera(copy_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #print("hello i am here")
            break
    cap.release()
    cv2.destroyAllWindows()
    n=65
    
    for event in detected_event:
        identified_labels[chr(n)]=event
        n+=1
##################################################
    return identified_labels


###############	Main Function	#################
if __name__ == "__main__":
    identified_labels = task_4a_return()
    print(identified_labels)
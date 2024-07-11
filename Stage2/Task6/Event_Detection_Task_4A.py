'''
* Team Id : GG_3344
* Author List : Abhay Agrawal, Abdul Basit
* Filename : Event_Detection_Task_4A.py
* Theme : GeoGuide eYRC 2023-24
* Functions :   get_transformed_frame(frame,aruco_detector),detect_events_roi(transformed_frame),
                get_event_label(event_key),check_event_if_empty(roi),classify_event(events_roi),
                 open_live_feed_event_detection_prediction()                
* Global Variables : combat,rehab,military_vehicles,fire,destroyed_buildings,event_labels_to_class
'''

##############################  IMPORT MODULES  ######################################################
import cv2
import numpy as np
import os
import pandas as pd
import csv
from tensorflow.keras.models import load_model
import tensorflow_hub as hub

#############################  GLOBAL VARIABLES  #####################################################
combat = "combat"
rehab = "humanitarian_aid"
military_vehicles = "military_vehicles"
fire = "fire"
destroyed_building = "destroyed_buildings"
event_labels_to_class={"combat":"Combat","humanitarian_aid":"Humanitarian Aid and rehabilitatiion","military_vehicles":"Military Vehicles","fire":"Fire","destroyed_buildings":"Destroyed buildings"}

######################################################################################################
'''
* Function Name- get_transformed_image
* Input - frame - This is the captured frame by the camera of the original size, 
          aruco_detector - aruco detector of the opencv to detect the aruco markers on the original frame
* Output- transformed_frame- This is the transformed frame to the four corners aruco on the arena of id - 4,5,6,7
* Logic - 1.To detect all the 4 corner aruco markers and get their centroid.
          2.Transform the original frame from centroids of corner aruco markers into 800*810  transformed_frame by changing perspective 
          from the centroid of the fetched corner aruco if detected. 
* Example Call - get_transformed_frame(frame,aruco_detector)
'''
def get_transformed_frame(frame,aruco_detector):
    # Creating the transformed frame and initialising it to zero array so that is all 4 corners are not detected it can be checked.
    transformed_frame = np.zeros(frame.shape) 
    aruco_corners, aruco_ids, _ = aruco_detector.detectMarkers(frame)#Fetch all aruco on the frame and their details
    desired_ids = [4,5, 6, 7] #Corners aruco ids
    ends_found = 0
    centroidslist_of_4_corners=[] #[(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
    
    for corner_id in desired_ids: #Searching for desired ids in total detected ids
        aruco_corner_id_index=np.where(aruco_ids==corner_id)[0]
        if len(aruco_corner_id_index)>0:  # Whether corner aruco in total detected if        
            # Calculate the centroid of the marker
            corners_of_marker = aruco_corners[aruco_corner_id_index[0]]
            corner_centroid = np.mean(corners_of_marker[0], axis=0)
            centroidslist_of_4_corners.append(corner_centroid)
            ends_found +=  1
    #Checking whether 4 corners detected or not to tranform the arena
    if ends_found<4:
        return transformed_frame # This will return transformed_frame as zero array since all four corner arucos are not detected.
    
    # Transforming the arena wrt four corners and cropping it to four corners if detected.
    xy_centroids_corners_list = np.float32(centroidslist_of_4_corners)
    xy_desired_corners_list=np.float32([[800,0],[0,0],[800,810],[0,810]])
    #Transforming the frame and wrapping
    M = cv2.getPerspectiveTransform(xy_centroids_corners_list,xy_desired_corners_list)
    transformed_frame = cv2.warpPerspective(frame,M,(800,810))
    return transformed_frame #Will return transformed_frame after tranformation and mapping to 4 corner arucos.



##########################################################################
'''
* Function Name- detect_events_roi
* Input - transformed_frame
* Output - events_roi - It is a list of arrays of images of the event location on the frame(white boxes) ,
           event_starting_coordinate - It is a nested list where each sublist contains the starting coordinates(x,y) of the location of the event to be stored in csv file.
* Logic - 1.Processing transformed frame and applying thresholding and contours detection.
          2.Filtering contours based on area and shape to remove aruco and other miscellaneous contours get required event contours.
          3.Fetching coordinates of those contours and drawing rectangle box on event contours in transformed frame.
'''
def detect_events_roi(transformed_frame):
    events_roi =[]
    #Image processing
    # Define a brightness adjustment factor and adjusting pixel values 
    brightness_factor = -0
    darkened_frame = np.clip(transformed_frame.astype(int) + brightness_factor, 0, 255).astype(np.uint8)
    gray_frame = cv2.cvtColor(darkened_frame, cv2.COLOR_BGR2GRAY)
    blurred_frame = cv2.GaussianBlur(gray_frame, (3,3), 0)
    # Creating CLAHE object and Applying CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    clahe_frame = clahe.apply(blurred_frame)
    #Applying thresholding to detect contours in frame
    _, threshold_frame = cv2.threshold(clahe_frame, 210, 255, cv2.THRESH_TOZERO)
    #Finding contours
    detected_contours, _ = cv2.findContours(threshold_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #Filtering the detected contours by removing aruco and other miscellaneous contours based on area and shape
    event_starting_coordinates=[]
    for cnt in detected_contours:
        if (5800 < cv2.contourArea(cnt) < 7000):
            x, y, w, h = cv2.boundingRect(cnt)
            if w/h>0.95 and w/h<1.05:
                #Fetching event contours starting coordinates
                event_starting_coordinates.append([x,y,w])
                #Fetching Region of Interest (Event contours)
                roi=transformed_frame[y+7:y+h-7, x+7:x+w-7]
                events_roi.append(roi)
                #Drawing rectangle on transformed frame on event contours
                cv2.rectangle(transformed_frame, (x+7, y+7), (x + w-7, y + h-7), (0, 255, 0), 3)
    #Returning events_roi(event contours) and event contours starting coordinates
    return events_roi,event_starting_coordinates

##########################################################################
'''
* Function Name- get_event_label
* Input - event_key - It is the classification number of highest probable class as predicted by ML Model.
* Output - label_dict(event_key) - It returns the corresponding event class(golbal variables) that has the event label.
* Logic - 1.It takes event_key as input and returns corresponding event label class
* Example Call - get_event_label(event_key)
'''
def get_event_label(event_key):
    #2fire,4militaryvehicles,3aid,0combat,1destroyed
    label_dict={0:combat,1:destroyed_building,2:fire,3:rehab,4:military_vehicles}
    return label_dict[event_key] #will return event label for classified event

##########################################################################
'''
* Function Name- check_event_if_empty
* Input - roi - coordinates of event locations on frame
* Output - Boolean - True / False 
* Logic - 1.To check for empty event locations if empty or they contains event.
          2.Applying mask for green pixels with specific pixel intensity range.
          3.Calculating the area of masked frame and filtering roi if percentage green area is greater than 30.
          4.Returning False if percentage area is greater that 30 i.e., event location is not empty and True if 
            percentage area less that 30 i.e., event location is empty
* Example Call - check_event_if_empty(event)
'''
def check_event_if_empty(roi):
    #image = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    image=roi
    # Create a mask for green pixels with intensity in the range (70, 130)
    green_mask = (image[:, :, 1] >= 80) & (image[:, :, 1] <= 130)
    # Count the number of green pixels
    green_area = green_mask.sum()
    # Calculate the percentage of green area
    total_area = image.shape[0] * image.shape[1]
    percentage_green_area = (green_area / total_area) * 100
    if percentage_green_area >30 :
        return False #return False if event is not empty
    return True #return True if event is empty

##########################################################################
'''
* Function Name- check_event_if_empty
* Input - events_roi - coordinates of event locations on frame
* Output - predicted_event_labels - predicted event labels classified by ML Model.
* Logic - 1.To check if event is empty or not then resize and normalize non empty event.
          2.Classify normalized event and return predicted event labels.
* Example Call - classify_event(events_roi)
'''
def classify_event(events_roi):
    predicted_event_labels=[]
    for event in events_roi:
        #checking if event is empty or not using "check_event_if_empty" function
        empty_or_not=check_event_if_empty(event)
        if empty_or_not==True:
            #image processing and resizing
            brighteness_factor = 15
            event = np.clip(event.astype(int)+brighteness_factor,0,255).astype(np.uint8)
            event=cv2.resize(event,(224,224))
            #normalizing and classifying event
            normalized_event_image=event/255
            CNN_model = load_model(os.path.join('','resizemodel3000.h5'),custom_objects={'KerasLayer':hub.KerasLayer})
            event_key=np.argmax(CNN_model.predict(np.expand_dims(normalized_event_image, 0)))
            #fetching event label for classified event.
            event_label = get_event_label(event_key)
            #append event label
            predicted_event_labels.append(event_label)
        else:
            #append none if event is empty
            predicted_event_labels.append("none")
    
    # Printing the Events Class Detected in form of dictionary
    n=65
    classified_events_class={}
    for event in predicted_event_labels:
        if event !="none":
            classified_events_class[chr(n)]=event_labels_to_class[event]
        n+=1
    print(classified_events_class)
    return predicted_event_labels #will return list containing event label for classified event and none for empty event. 

##########################################################################
'''
* Function Name- open_live_feed_event_detection_prediction
* Input - none
* Output - identified_labels - dictionary containing A/B/C/D/E as key and predicted event label as value.
* Logic - 1.To open live feed and transform frame then detect and classify events.
          2.Live feed is opened then the frame is transformed using get_transformed_frame function.
          3.Get event locations and coordinates using detect_events_roi function.
          4.If number of events detected is 5 and classification is not done then write event coordinates in event_location.csv
          and classify events using classify_event function and put event label text on transformed frame.
          5.When frame is closed after classification, get event class name from event labels and append predicted event class 
          as value and A/B/C/D/E as key in classified_events_class dictionary.

* Example Call - classify_event(events_roi)
'''
def open_live_feed_event_detection_prediction():
    
    #######################################################
    #Parameters for aruco detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)
    #######################################################
    identified_labels = {}
    classified_events_class={}
    predicted_event_labels=[]
    classification_number=0
    #open live feed and set frame height, width.
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,2000)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,2000)
    print("starting")
    while True:
        ret, frame = cap.read()
        #get transformed frame.
        transformed_frame=get_transformed_frame(frame,aruco_detector)
        if not transformed_frame.any():
            continue
        #get event locations and starting coordinates.
        events_roi,event_coordinates=detect_events_roi(transformed_frame)
        number_of_event=len(events_roi)#get number of events detected.
        #if event classification is not done and number of event detected is 5
        if classification_number==0 and number_of_event==5:
            #write events starting coordinates in event_location.csv
            with open('event_location.csv','w',newline="") as handle:
                writer = csv.writer(handle, delimiter=",", skipinitialspace=True)
                for loc in event_coordinates:
                    writer.writerow(loc)
            #classify detected events and get predicted event labels.
            predicted_event_labels=classify_event(events_roi)
            
            #incriment classification number so that events are not clasified again.
            classification_number=1
        if number_of_event==5:
            k=0
            for event_label in predicted_event_labels:
                #get event starting coordinates.
                x,y,_=event_coordinates[k]
                k+=1
                #if event is not none put event label text on transformed frame.
                if event_label!="none":
                    cv2.putText(transformed_frame,event_label,(x-10,y-20),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),3,cv2.LINE_AA)
            
        cv2.imshow("Transformed_frame",transformed_frame)  
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #Closing the camera
    cap.release()
    cv2.destroyAllWindows()
    n=65
    for event in predicted_event_labels:
        #if event is not none.
        if event !="none":
            #add event label in dictionary as value with A/B/C/D/E as key.
            identified_labels[chr(n)]=event
            #get event class from event label and add in dictionary as value with A/B/C/D/E as key. 
            classified_events_class[chr(n)]=event_labels_to_class[event]
        else:
            #if event is none add none as value.
            classified_events_class[chr(n)]="none"
        n+=1
    #write event classified_events_class dictionary to classified_events_class_file.csv file.
    (pd.DataFrame.from_dict(data=classified_events_class, orient='index')
    .to_csv('classified_events_class_file.csv', header=False))
##################################################
    return identified_labels


###############	Main Function	#################
if __name__ == "__main__":
    identified_labels = open_live_feed_event_detection_prediction()
    #print(identified_labels)
    
    
'''
* Team Id : GG_3344
* Author List : Abhay Agrawal, Abdul Basit
* Filename : Geo_Tracking_Task_4B.py
* Theme : GeoGuide eYRC 2023-24
* Functions :   read_csv_to_dictionary(csv_name), update_QGIS_csv(coordinate,csv_name), 
                nearest_arena_point(frame,aruco_detector,points_loc), main()
* Global Variables : combat,rehab,military_vehicles,fire,destroyed_buildings,event_labels_to_class
'''

##############################  IMPORT MODULES  ######################################################
import numpy as np
import cv2
import time 
import csv
from Event_Detection_Task_4A import get_transformed_frame

######################################################################################################
'''
* Function Name- read_csv_to_dictionary
* Input - csv_name -  CSV file name to be read into dictionary
* Output- x_y_coordinate - returns dictionary containing csv file data
* Logic - 1.creating cvs file object in read mode
          2.enumerating csv file and storing data in dictionary
* Example Call - read_csv_to_dictionary("points_location.csv"), read_csv_to_dictionary("points_coordinate.csv")
'''
def read_csv_to_dictionary(csv_name):
    x_y_coordinate = {}
    #Reading csv file data
    with open(csv_name, 'r') as csv_file_object:
        csv_reader = csv.reader(csv_file_object, delimiter=',')
        for index, row in enumerate(csv_reader):
            #Fetching CSV data into dictionary.
            x_y_coordinate[index] = [float(coord) for coord in row]
    return x_y_coordinate

######################################################################################################
'''
* Function Name- update_QGIS_csv
* Input - coordinate - live latitude longitude of vanguard
          csv_name - csv file name
* Output- none 
* Logic - 1.Creating cvs file object in write mode
          2.Storing live latitude longitude row wise in live_data.csv
* Example Call - update_QGIS_csv(current_coordinate,"live_data.csv")
'''   
def update_QGIS_csv(coordinate,csv_name):
    #Writing latitude longitude in live_data.csv
    with open(csv_name,"w") as csv_file:
        csv_file_writer = csv.writer(csv_file)
        csv_file_writer.writerows((["lat","lon"],coordinate))

######################################################################################################
'''
* Function Name- nearest_arena_point
* Input - frame - live transformed frame
          aruco_detector - aruco detection parameter
          points_loc - dictionary of defined points (x,y) on transformed frame (for geo tracking)
* Output- index_nearest_point - index of nearest coordinate to vanguard from points_loc 
* Logic - 1.Detecting id 100 aruco marker and calculating its centroid coordinate in frame.
          2.Creating CVS file object in write mode and adding live id 100 aruco centroid coordinates.  
          3.Calculating distance of id 100 aruco with defined points on frame from point_loc for geo tracking.
          4.Determining nearest point and returning its index.
          ** We have not used the aruco markers for geotracking as it was not accurate due to height of the bot.
            we rather prefered to mark the x,y of the road points in a csv and map the geolocation respectively.
            Csv files used points_location.csv , points_coordinate.csv
* Example Call - nearest_arena_point(transformed_frame,aruco_detector,arena_points_xy)
''' 
def nearest_arena_point(frame,aruco_detector,points_loc):
    target_aruco_id = 100 #ID of aruco on Vanguard 
    index_nearest_point = None
    #ArUco marker detection.
    corners, ids, rejected = aruco_detector.detectMarkers(frame)
    #Fetching index of ID 100 aruco marker.
    target_aruco_index = np.where(ids == target_aruco_id)[0]
    try:
        if len(target_aruco_index) > 0:
            #Get the corners of ID 100 aruco
            target_corners = corners[target_aruco_index[0]]
            #Calculate the centroid of ID 100 aruco
            target_centroid = np.mean(target_corners[0], axis=0)
            #Writing coordinates of centroid in coordinate_100.csv
            with open('coordinate_100.csv','w') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(target_centroid)
            #Calculate the distance of centroid from all defined points on frame.
            distances = [np.linalg.norm(target_centroid - np.array([float(coord) for coord in coords])) for coords in points_loc.values()]
            if distances:
                #Find the index of the nearest defined point
                index_nearest_point = np.argmin(distances)
            else:
                print("No other marker found in the image")
        else:
            print(f"Marker {target_aruco_id} not found in the map.") 
  
    except Exception as e:
        print(f"An error occurred: {e}")
    return index_nearest_point #Will return index of nearest point.

######################################################################################################
'''
* Function Name- main
* Input - none
* Output- none
* Logic - 1.Store defined points coordinates and geolocation latitude longitude in dictionary.
          2.Open live camera feed and get transformed frame.  
          3.Calculating index of nearest point using nearest_arena_point function and updating live_data.csv using update_QGIS_csv function.
* Example Call - main()
''' 
def main():
    #######################################################
    #ArUco detection parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)
    #######################################################
    #Fetching (x,y) coordinates of defined points on frame
    arena_points_xy=read_csv_to_dictionary("points_location.csv")
    #Fetching latitude longitude for geotracking
    geolocation_coordinates=read_csv_to_dictionary("points_coordinate.csv")
    cap = cv2.VideoCapture(0) #Open live feed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2000)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2000)
    print("Starting")
    time.sleep(2)
    while True:
        ret,frame=cap.read()
        #Get transformed frame using function defined in Event_Detection_Task_4A.py
        transformed_frame=get_transformed_frame(frame,aruco_detector)
        if not transformed_frame.any():
            continue
        #Get index of nearest defined point from vanguard
        index=nearest_arena_point(transformed_frame,aruco_detector,arena_points_xy)
        if index is not None:
            #Fetching live latitude longitude of vanguard and updating it in live_data.csv
            current_coordinate=geolocation_coordinates[index]
            update_QGIS_csv(current_coordinate,"live_data.csv")
        cv2.imshow("Transformed_frame",transformed_frame) 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
             
if __name__ == "__main__":
    main()
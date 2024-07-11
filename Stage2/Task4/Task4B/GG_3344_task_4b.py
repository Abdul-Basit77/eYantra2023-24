import numpy as np
import cv2
import time 
import csv

# ArUco marker parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

coordinates=[39.6128542,-74.3629792]
#Reading all latitude and longitude corresponding to ids
def read_lat_lon(csv_name):
    lat_lon = {}
    with open(csv_name, 'r') as csv_file_object:
        csv_reader = csv.reader(csv_file_object, delimiter=',')
        for row in csv_reader:
            lat_lon[row[0]] = [row[1], row[2]]
    return lat_lon

#Getting the Latitude and Longitude corresponding to the nearest aruco id 
def tracker_id(id,lat_lon):
    global coordinate
    try:
        coordinate=lat_lon[str(id)[1:-1]]
    except:
        pass
    return coordinate

#Writing the Csv File to update QGIS file.   
def update_QGIS(coordinate,csv_name):
    with open(csv_name,"w") as csv_file:
        csv_file_writer = csv.writer(csv_file)
        csv_file_writer.writerows((["lat","lon"],coordinate))

def nearest_aruco(frame):
    marker_size = 10.0 
    target_id = 100
    id_nearest_marker=0
    # ArUco marker detection
    corners, ids, rejected = detector.detectMarkers(frame)
    # print(ids)
    # cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

    # Display the frame with detected events and ArUco markers
    cv2.imshow('Detected Events and ArUco Markers', frame)
    target_index = np.where(ids == target_id)[0]

    try:
        if len(target_index) > 0:
            # Get the corners of the marker with ID 100
            target_corners = corners[target_index[0]]
            # Calculate the centroid of the marker with ID 100
            target_centroid = np.mean(target_corners[0], axis=0)
            # Calculate the distances to all other markers
            distances = [np.linalg.norm(target_centroid - np.mean(corners[i][0], axis=0)) for i in range(len(ids)) if i != target_index[0]]
            if distances:
                # Find the index of the nearest marker but it has some issue to be checked in if statement
                index_nearest_marker = np.argmin(distances)
                if index_nearest_marker>=target_index[0]:
                    index_nearest_marker = index_nearest_marker+1 # this is done because the distances array is one size smaller than ids
                # Get the ID and corners of the nearest marker
                id_nearest_marker = ids[index_nearest_marker]
                corners_nearest_marker = corners[index_nearest_marker][0]
                # Calculate the distance between the markers
                distance = np.linalg.norm(target_centroid - np.mean(corners_nearest_marker, axis=0)) * marker_size / 2.0
                print(f"Distance between marker {target_id} and nearest marker (ID {id_nearest_marker}): {distance} units")
            else:
                print("No other marker found in the image")
        else:
            print(f"Marker {target_id} not found in the map.")
            
    except Exception as e:
        print(f"An error occurred: {e}")
    return id_nearest_marker

def cornerdetection(frame):
    corners, ids, _ = detector.detectMarkers(frame)
    desired_ids = [4,5, 6, 7]
    centroids = {}
    ends_found = 0
    centroidslist=[]
    if ids is not None:
        for i, marker_id in enumerate(ids):
            if marker_id in desired_ids:
                index = np.where(ids == marker_id)[0][0]  # Find the index of the marker in the list
                # Calculate the centroid of the marker
                corners_of_marker = corners[index][0]
                centroids[int(marker_id[0])] = (int(np.mean(corners_of_marker[:,0], axis=0)),int(np.mean(corners_of_marker[:,1], axis=0)))
                centroidslist.append([int(np.mean(corners_of_marker[:,0], axis=0)),int(np.mean(corners_of_marker[:,1], axis=0))])
                ends_found +=  1
    return ends_found,centroidslist
def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2000)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2000)
    lat_lon = read_lat_lon("lat_long.csv")
    time.sleep(2)
    
    while True:
        ret, frame = cap.read()
        ends_found,centroidslist=cornerdetection(frame)
        if ends_found<4:
            continue
        break
    pts1 = np.float32(centroidslist)
    pts2=np.float32([[0,900],[900,900],[900,0],[0,0]])
    while True:
        ret, frame = cap.read()
        M = cv2.getPerspectiveTransform(pts1,pts2)
        transformed_frame = cv2.warpPerspective(frame,M,(900,900))
        id=nearest_aruco(transformed_frame)
        if id!=0:
            coordinate=tracker_id(id,lat_lon)
            update_QGIS(coordinate,"live_data.csv")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
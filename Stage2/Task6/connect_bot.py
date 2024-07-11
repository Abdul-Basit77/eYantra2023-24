'''
* Team Id : GG_3344
* Author List : Abhay Agrawal, Abdul Basit,Mohammad Kaif , Shiwam Kumar
* Filename : connect_bot.py
* Theme : GeoGuide eYRC 2023-24
* Class : Dijkstra-->Function : shortestPath(self, n, m, edges, source, destination),printAdjacencyList(self)
* Functions : get_shortest_path_and_string(path_start_node,path_end_node), arrange_events_in_priority(all_classified_events,event_locations_xyw),
              calculate_path_and_Uturn(arranged_all_classified_events),  Stop_bot_if_event_reached(event_location_to_visit),main()        
* Global Variables : None
'''

##############################  IMPORT MODULES  ######################################################
import heapq
import numpy as np
import socket
from time import sleep
import csv
import pandas as pd

######################################################################################################

#class for Implementing Dijkstra's algorithm for calculating shortest path
class Dijkstra:
    '''
    * Function Name - shortestPath
    * Input - n: The number of nodes in the graph.
              m: The number of edges in the graph.
              edges: A list of tuples representing edges in the graph. Each tuple contains three elements: (source_node, destination_node, weight).
              source: The source node from which to find the shortest path.
              destination: The destination node to which to find the shortest path.
    * Output- path: The shortest path from the source node to the destination node. If no path exists, it returns [-1].
    * Logic - 1.Creating Adjacency List: The code begins by creating an adjacency list representation of the graph. This 
                list stores the neighbors of each node along with the corresponding edge weights.
              2.Dijkstra's Algorithm Initialization: It initializes a priority queue pq with a tuple containing (distance, node) 
                where distance represents the distance from the source to the node, and node represents the node itself. Initially, the 
                distance to all nodes except the source is set to infinity, and the parent of each node is set to itself.
              3.Dijkstra's Algorithm Loop: The main loop of Dijkstra's algorithm iterates until the priority queue is empty.
                In each iteration, it extracts the node with the minimum distance from the priority queue and relaxes its neighbors if a shorter path is found.
              4.Updating Distances and Parents: It updates the distance to each neighboring node if the sum of the distance 
                to the current node and the edge weight is smaller than the previously recorded distance. It also updates the parent of each neighboring node to keep track of the shortest path.
              5.Path Reconstruction: Once Dijkstra's algorithm completes, it reconstructs the shortest path from the source 
                to the destination by backtracking from the destination node through the parent pointers.
              6.If a path exists from the source to the destination, it returns the shortest path as a list of nodes. 
                If no path exists, it returns [-1].
    * Example Call - path = obj.shortestPath(V, E, edges, source, destination)
    ''' 
    def shortestPath(self, n, m, edges, source, destination):
        # Create an adjacency list
        adj = {i: [] for i in range(1, n + 1)}
        for edge in edges:
            adj[edge[0]].append((edge[1], edge[2]))
            adj[edge[1]].append((edge[0], edge[2]))
        self.adjacency_list = adj  # Store the adjacency list as a class attribute
        # Create a priority queue for storing the nodes along with distances 
        # in the form of a tuple (dist, node).
        pq = [(0, source)]
        # Create a dist dictionary for storing the updated distances and a parent dictionary
        # for storing the nodes from where the current nodes came from.
        dist = {i: float('inf') for i in range(1, n + 1)}
        parent = {i: i for i in range(1, n + 1)}
        dist[source] = 0
        while pq:
            # Topmost element of the priority queue is with minimum distance value.
            dis, node = heapq.heappop(pq)
            # Iterate through the adjacent nodes of the current popped node.
            for adjNode, edW in adj[node]:
                # Check if the previously stored distance value is 
                # greater than the current computed value or not, 
                # if yes then update the distance value.
                if dis + edW < dist[adjNode]:
                    dist[adjNode] = dis + edW
                    heapq.heappush(pq, (dis + edW, adjNode))
                    # Update the parent of the adjNode to the recent 
                    # node where it came from.
                    parent[adjNode] = node
        # If distance to the specified destination node could not be found, return an array containing -1.
        if dist[destination] == float('inf'):
            return [-1]
        # Store the final path in the 'path' array.
        path = []
        node = destination
        # Iterate backwards from destination to source through the parent dictionary.
        while parent[node] != node:
            path.append(node)
            node = parent[node]
        path.append(source)
        # Since the path stored is in reverse order, reverse the array to get the final answer and return it.
        return path[::-1]

    ######################################################################################################
    '''
    * Function Name- printAdjacencyList
    * Input - none
    * Output- Prints the adjacency list representation of the graph to the console.
    * Logic - 1.Print Header: Prints a header indicating that the following output is the Adjacency List.
              2.Iterate Through Nodes: Iterates through each node in the adjacency list.
              3.Print Node and Its Neighbors: For each node, prints the node number and its associated neighbors.
    * Example Call - printAdjacencyList(self)
    '''
    def printAdjacencyList(self):
        print("Adjacency List:")
        for node, neighbors in self.adjacency_list.items():
            print(f"{node}: {neighbors}")

##############################END OF CLASS##############################################
######################################################################################################
'''
* Function Name- get_shortest_path_and_string
* Input - path_start_node: The starting node of the path of the bot.
          path_end_node: The ending node of the path where the bot has to visit the event to.
* Output- path: Tuple containing the shortest path from the path_start_node to the path_end_node.
          path_string: Tuple containing a string representing the direction of movement between the nodes in the shortest path.
* Logic - 1.Graph Initialization: The function initializes the number of nodes V and edges E in the graph, along with a list 
          edges representing the edges and their weights in the graph.
          2.Dijkstra's Algorithm: It uses Dijkstra's algorithm to find the shortest path between the path_start_node and the path_end_node.
          3.Path Generation: If a path exists, it generates a path string representing the direction of movement between the nodes
          in the shortest path. It does this by examining the connectivity matrix and determining the direction of movement from 
          each node to its adjacent nodes by check the location of the previous node.
* Example Call - get_shortest_path_and_string(Path_start_node,Path_end_node)
'''
def get_shortest_path_and_string(path_start_node,path_end_node):
    V, E = 20, 26
    edges = [[17,1,2],[1,2,2],[2,3,2],[3,4,2],[4,18,2],[18,7,1],[7,19,4],[19,16,2],[1,5,1],[5,8,2],[2,9,3],[3,6,1],[6,10,2],[4,11,3],[8,20,4],[20,14,2],[8,9,2],[9,10,2],[10,11,2],[11,16,3],[10,13,2],[13,15,1],[9,12,2],[12,14,1],[14,15,2],[15,16,2]]
    source, destination =path_start_node,path_end_node
    obj = Dijkstra()
    path = obj.shortestPath(V, E, edges, source, destination)
    if path[0] == -1:
        print("No path from source to destination exists.")
    else:
        print("")
        #print(f"Shortest path from source {source} to destination {destination}: {path}")
    #Finding the route of Left straight right
    # Using the connection matrix to fix the location of decided nodes according to their location on the arena.
    #If any node dpes not exist on the arena we put its value equal to 100 on the matrix showing none in case.
    # 17 is mapped to starting point whereas 1 to node1. 5 to the location of the bot at A.
    #rows of the matrix show the horizontal direction on the arena whereas columns shows the arena in vertical direction.
    #This matrix has to be considered in up down manner to persume on the arena.
    connection_matrix=np.array([[17,100,100,100,100],
                                [1,5,8,100,20],
                                [2,100,9,12,14],
                                [3,6,10,13,15],
                                [4,100,11,100,16],
                                [18,7,100,100,19]])
    # Logic used - To check whether bot needs to turn left ,right or prooceed forward to reach the next node from the current node.
    # For ex - If we have to go to node 9 and currently standing at A event(5). which means we have to move forward to node 8 and then take a left turn to 9.
    # So therefore we have to consider the previous node ,current node and the next node.
    # There corresponding location on the matrix is shown by (iprevious,jprevious),(icuurent,jcurrent),(inext,jnext)
    number_of_nodes=len(path)
    path_string=""
    for node in range(1,number_of_nodes-1): # Traversing from the second point of the path recieved so as to get the previous node.
        if path[node] in [5,12,13,6,18,19,20,7]:# This list contains the node numbers of the event location which will either be first or the last node of any path calculated.
            continue #No need to calculate the path at that node as it is hypothetical just to calculate the shortest path and is npt real node on arena
        ip,jp=np.where(connection_matrix==path[node-1])
        iprevious,jprevious=ip[0],jp[0]
        ic,jc=np.where(connection_matrix==path[node])
        icurrent,jcurrent=ic[0],jc[0]
        inx,jnx=np.where(connection_matrix==path[node+1])
        inext,jnext=inx[0],jnx[0]
        #3 for the right turn
        #2 for Straight
        #1 for left turn
        
        #If the bot comes vertically from previous node
        if(iprevious<icurrent): # if came from the Lower row to the upper row on the arena i.e. moving up on the arena 
            if(jnext>jcurrent): # To go on horizontally positive means right turn is needed.
                path_string+="3"
            elif(jnext<jcurrent):# To go horizontally negative means a left turn is needed.
                path_string+="1"
            elif(jnext==jcurrent):# To go none where horizontally means bot has to traverse straight.
                path_string+="2"
        elif(iprevious>icurrent): # If bot comes from the Upper row to Lower Row on the arena i.e Moving down on arena.
            if(jnext>jcurrent): # The reverse of the previous case takes place.
                path_string+="1"
            elif(jnext<jcurrent):
                path_string+="3"
            elif(jnext==jcurrent):
                path_string+="2"
        # If the Bot comes horizontally from the previous node
        elif(jprevious<jcurrent): 
            if(inext>icurrent):
                path_string+="1"
            elif(inext<icurrent):
                path_string+="3"
            elif(inext==icurrent):
                path_string+="2"
        elif(jprevious>jcurrent):
            if(inext>icurrent):
                path_string+="3"
            elif(inext<icurrent):
                path_string+="1"
            elif(inext==icurrent):
                path_string+="2"   
    return path,path_string #It returns the shortest path calculated as nodesas a tuple and the path string having the turns and straight command. 

######################################################################################################
'''
* Function Name- arrange_events_in_priority
* Input - 1.All_classified_events: A dictionary where keys are event locations and values are event classes.
          2.Event_locations_xyw: A list of tuples representing the (x, y, w) coordinates of event locations. where w is the width of the event.
* Output- 1.Arranged_all_classified_events: Tuple containing a list of nodes representing the arranged events based on priority order.
          2.Arranged_event_location_xyz: Tuple containing a list of tuples representing the (x, y, w) coordinates of the arranged event locations.
* Logic - 1.Priority Order: The function defines a list events_priority_order containing the priority order of event classes.
          2.Event Location Nodes: It defines a dictionary event_location_nodes mapping event locations to their corresponding nodes in the graph.
          3.Arrangement: It iterates through each priority in the events_priority_order list. For each priority, it iterates through the 
          all_classified_events dictionary and checks if the event class matches the current priority. If it does, it appends the corresponding 
          node to arranged_all_classified_events and the location tuple to arranged_event_location_xyw.
4.Adding Node for Home Base: After arranging all events, it appends the node representing the home base to arranged_all_classified_events and adds a default location tuple to arranged_event_location_xyz. 
* Example Call - arrange_events_in_priority(All_Classified_Events,Event_Locations_XYW)
'''
def arrange_events_in_priority(all_classified_events,event_locations_xyw):
    events_priority_order=["Fire","Destroyed buildings","Humanitarian Aid and rehabilitatiion","Military Vehicles","Combat"]
    event_location_nodes={"A":5,"B":12,"C":13,"D":6,"E":7}
    arranged_all_classified_events=[]
    arranged_event_location_xyw=[]
    for priority in events_priority_order:
        j=0
        for event_loc,event_class in all_classified_events.items():
            if event_class == priority:
                arranged_all_classified_events.append(event_location_nodes[event_loc])
                arranged_event_location_xyw.append(event_locations_xyw[j])
            j+=1
    arranged_all_classified_events.append(17)
    arranged_event_location_xyw.append((40,780,20))
    return arranged_all_classified_events,arranged_event_location_xyw #It returns the arranged events and their locations as a tuple.

######################################################################################################
'''
* Function Name- calculate_path_and_Uturn
* Input - arranged_all_classified_events: A list of nodes representing the arranged events.
* Output- Shortest_path_all_events_string: A list Shortest_path_all_events_string containing strings representing the shortest paths between two events.
* Logic - 1.Initialization: The function initializes the start node of the path (Path_start_node), variables to store the current and 
            previous calculated paths (Current_calculated_path, Previous_calculated_path), and a list to store the shortest path strings 
            between events (Shortest_path_all_events_string).
          2.Calculate Shortest Path: For each event node in the arranged_all_classified_events, it calculates the shortest path from 
          the current start node to the event node using the get_shortest_path_and_string function.
          3.Determine U-turn: It checks if the second node of the current calculated path matches the second-to-last node of the 
          previous calculated path. If it does, it indicates that a U-turn is required. It then adds a U-turn command ("4") to the 
          beginning of the shortest path string, indicating that a U-turn should be performed before following the calculated path.
          If no U-turn is required, it adds a straight move command ("2") to the beginning of the shortest path string.
          4.Append to List: It appends the shortest path string to the list Shortest_path_all_events_string.
          5.Update Previous Path and Start Node: It updates the Previous_calculated_path and Path_start_node for the next iteration.
* Example Call - calculate_path_and_Uturn(Arranged_All_Classified_Events)
'''
def calculate_path_and_Uturn(arranged_all_classified_events):
    Path_start_node=17 # base position of the bot node
    Current_calculated_path=[0,0] #initialised to avoid errors
    Previous_calculated_path=[0,0]
    Shortest_path_all_events_string=[]
    for priority_event_node in arranged_all_classified_events:
        Path_end_node=priority_event_node
        Current_calculated_path,Shortest_path_string=get_shortest_path_and_string(Path_start_node,Path_end_node)
        if Current_calculated_path[1]==Previous_calculated_path[-2]: # Check is the node from which bot just came is same 
                                                                      #as the node bot needs to next visit which means Uturn is needed from event locartion
            Shortest_path_string="4"+Shortest_path_string #Command if UTurn is needed to be taken or not if yes then
        else:
            Shortest_path_string="2"+Shortest_path_string
        Shortest_path_all_events_string.append(Shortest_path_string) #number of events +1 since paths to visit event and to return to the base location.
        Previous_calculated_path=Current_calculated_path
        Path_start_node=Path_end_node
    #It returns the list Shortest_path_all_events_string containing the shortest paths between events and U-turn commands. 
    return Shortest_path_all_events_string 

######################################################################################################
'''
* Function Name- Stop_bot_if_event_reached
* Input - event_location_to_visit: A tuple representing the (x, y, w) coordinates of the event location to be visited.
* Output- "Stop" if the event location is reached.
* Logic - 1.Continuous Loop: The function runs in an infinite loop (while True) to continuously monitor the robot's position.
          2.Read Coordinates: It attempts to read the coordinates of the robot's current position from a CSV file 
          named 'coordinate_100.csv'. The file should contain the coordinates of the robot's position.
          3.Check Event Location: It compares the coordinates of the robot's position with the coordinates of the event 
          location to be visited. If the robot's position is within a certain range around the event location, it 
          considers the event location to be reached
* Example Call - Stop_bot_if_event_reached(Event_Location_To_Visit)
'''
def Stop_bot_if_event_reached(event_location_to_visit):
    while True:
        try:
            coordinate_100 = [tuple(x) for x in pd.read_csv('coordinate_100.csv', header=None).values.tolist()]
        except:
            continue
        x_event,y_event,w_event=event_location_to_visit
        x_100_aruco,y_100_aruco=coordinate_100[0]
        x_event_center=x_event+w_event//2
        if x_100_aruco>x_event_center-80 and x_100_aruco<x_event_center+70 and y_100_aruco-30<y_event and y_100_aruco>y_event-60:
            return "Stop"
        print("",end="",sep="")
############################################################################

'''
* Function Name- main()
* Input - None
* Output - None
* Logic - 1.Reads the classified events csv stored by the Event Detection code to All_Classified_Events
          2.Reads The location of the Events stored by the Event Detection code to Event_Locations_XYW
          3.It then arranges all the classified events according to the priority using the arrange_evenrs_in_priority function.
          4.Then it calculates the shortest path and fetch the list of tuples containing all paths and the path sting using the calculate_path_and_Uturn function.
          5.It then finds whether the bot needs to visit the E event as it has to transfer the priority of the E event to the bot. If it does find its index in the arranged events.
            Point 5 is done as the bot has to reduce its spped at the turn and it should be variable according to the priorty of the E.
          6.Then it connects to the bot using socket library and ip .Bot and the system should be on same WIFI.
          7.It then tries to send the number of path needs to be traversed in a run so that bot can accept the same number of paths using connection and the sendall function.
          8.It then sends the path string one by one to the bot comma seperated so that it can be fetched differently.
          9.Once all the paths are send it sends the priority of the E event.
          10.Then the function waits for the user to Give the "RUN" command to start the bot which is then communicated to the bot.
          11.Once the bot starts moving it traverses the event location according to the priority and determines when the bot needs to stop using the "stop_bot_if_event_reached" function.
          12.If the bot reaches the event the "Yes" command is sent to the bot.
          13.After all the events are successfully visited the connection with the bot is closed successfully.
'''
def main():
    with open('classified_events_class_file.csv') as csv_file:
        reader = csv.reader(csv_file)
        All_Classified_Events = dict(reader)
    Event_Locations_XYW = [tuple(x) for x in pd.read_csv('event_location.csv', header=None).values.tolist()]
    Arranged_All_Classified_Events,Arranged_Event_Location_XYW=arrange_events_in_priority(All_Classified_Events,Event_Locations_XYW)
    Shortest_Path_All_Events_String=calculate_path_and_Uturn(Arranged_All_Classified_Events)
    #Finding path number of E
    if 7 in Arranged_All_Classified_Events:
        path_number_of_E=Arranged_All_Classified_Events.index(7)
    else:
        path_number_of_E=10
    ip = "192.168.50.144"   
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: 
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((ip, 8002)) # opening the socket at the 8002 port
        s.listen()#waiting for the connect request 
        conn, addr = s.accept()#Accepting the bot connection
        with conn: # If the bot is connection successfully.
            print(f"Connected by {addr}")
            try: # If any error is raised close the connection and the port socket.
                conn.sendall(str.encode(str(len(Shortest_Path_All_Events_String))+",")) # Sending number of paths
                for path_number in range(0,len(Shortest_Path_All_Events_String)): # sending paths one by one
                    conn.sendall(str.encode(str(Shortest_Path_All_Events_String[path_number])+","))
                conn.sendall(str.encode(str(path_number_of_E)+","))# Sending priority of E
                while True:
                    run_command_input=input('ENTER "RUN" COMMAND TO START THE BOT : ')
                    if run_command_input!="RUN":
                        continue # read Command Until "RUN" command is given
                    conn.sendall(str.encode("RUN"))
                    break
                sleep(3)
                for path_number in range(0,len(Shortest_Path_All_Events_String)):
                    Event_Location_To_Visit=Arranged_Event_Location_XYW[path_number]# traversing the priority event one by one.
                    stop=Stop_bot_if_event_reached(Event_Location_To_Visit)#check is bot reached the location
                    conn.sendall(str.encode("Yes"))
                print("Connection Closed")
                s.close()    
            except:
                s.close()
########################################################################
main() 
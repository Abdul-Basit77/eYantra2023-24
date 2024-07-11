import heapq
import numpy as np
import socket
from time import sleep
import signal		
import sys	
import csv
import pandas as pd

events_priority=["Fire","Destroyed buildings","Humanitarian Aid and rehabilitatiion","Military Vehicles","Combat"]
path_event_node=[]
event_location_nodes={"A":5,"B":12,"C":13,"D":6,"E":7}
event_aruco_location={5:('39.6128389','-74.3625328'),12:('39.6130036','-74.3618029'),13:('39.6132795','-74.3616113'),6:('39.6134519','-74.3624428'),7:('39.613900','-74.3621746'),17:('39.6128542','-74.3629792')}
starting_node=17
end_node=0
total_event_path=[]
previous_path=[0,0]
current_path=[0,0]
def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

class Solution:
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

    def printAdjacencyList(self):
        print("Adjacency List:")
        for node, neighbors in self.adjacency_list.items():
            print(f"{node}: {neighbors}")

    # Driver Code
def get_path(start,end):
    V, E = 20, 26
    edges = [[17,1,2],[1,2,2],[2,3,2],[3,4,2],[4,18,2],[18,7,1],[7,19,4],[19,16,2],[1,5,1],[5,8,2],[2,9,3],[3,6,1],[6,10,2],[4,11,3],[8,20,4],[20,14,2],[8,9,2],[9,10,2],[10,11,2],[11,16,3],[10,13,2],[13,15,1],[9,12,2],[12,14,1],[14,15,2],[15,16,2]]
    source, destination =start,end
    obj = Solution()
    path = obj.shortestPath(V, E, edges, source, destination)

    if path[0] == -1:
        print("No path from source to destination exists.")
    else:
        print(f"Shortest path from source {source} to destination {destination}: {path}")

    # Print the adjacency list
    #obj.printAdjacencyList()

    #Finding the route of LHR
    connection_matrix=np.array([[17,100,100,100,100],
                                [1,5,8,100,20],
                                [2,100,9,12,14],
                                [3,6,10,13,15],
                                [4,100,11,100,16],
                                [18,7,100,100,19]])
    #print(connection_matrix)

    number_of_nodes=len(path)
    path_string=""
    for node in range(1,number_of_nodes-1):
        if path[node] in [5,12,13,6,18,19,20,7]:
            continue
        ip,jp=np.where(connection_matrix==path[node-1])
        iprevious,jprevious=ip[0],jp[0]
        ic,jc=np.where(connection_matrix==path[node])
        icurrent,jcurrent=ic[0],jc[0]
        inx,jnx=np.where(connection_matrix==path[node+1])
        inext,jnext=inx[0],jnx[0]
        r="Right"
        l="Left"
        s="Straight"
        if(iprevious<icurrent):
            if(jnext>jcurrent):
                path_string+="3"
                print(r)
            elif(jnext<jcurrent):
                path_string+="1"
                print(l)
            elif(jnext==jcurrent):
                path_string+="2"
                print(s)
        elif(iprevious>icurrent):
            if(jnext>jcurrent):
                path_string+="1"
                print(l)
            elif(jnext<jcurrent):
                path_string+="3"
                print(r)
            elif(jnext==jcurrent):
                path_string+="2"
                print(s)
        elif(jprevious<jcurrent):
            if(inext>icurrent):
                path_string+="1"
                print(l)
            elif(inext<icurrent):
                path_string+="3"
                print(r)
            elif(inext==icurrent):
                path_string+="2"
                print(s)
        elif(jprevious>jcurrent):
            if(inext>icurrent):
                path_string+="3"
                print(r)
            elif(inext<icurrent):
                path_string+="1"
                print(l)
            elif(inext==icurrent):
                path_string+="2"
                print(s)    
    return path_string,path

with open('dict_file.csv') as csv_file:
    reader = csv.reader(csv_file)
    predicted_events = dict(reader)
    
print(predicted_events)
i_location=[]

for priority in events_priority:
    j=0
    for key,value in predicted_events.items():
        if value== priority:
            path_event_node.append(event_location_nodes[key])#Adding the corresponding key to visit as per priority
            i_location.append(j)
            pass
        j+=1
path_event_node.append(17)#adding the starting node to return at the last
i_location.append(5)
print(i_location)
print(path_event_node)
for event_location in path_event_node:#A,B,C,D,E
    #end_node=event_location_nodes[event_location]#5,12,13,6,7
    end_node=event_location
    shortest_path,current_path=get_path(starting_node,end_node)
    if current_path[1]==previous_path[-2]:
        shortest_path="4"+shortest_path #Command if UTurn is needed to be taken or not if yes then
    else:
        shortest_path="2"+shortest_path
    previous_path=current_path
    total_event_path.append(shortest_path)
    starting_node=end_node


print(total_event_path)
ip = "192.168.131.144"     #Enter IP address of laptop after connecting it to WIFI hotspot
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, 8004))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        try:
            conn.sendall(str.encode(str(len(total_event_path))))
            sleep(3)
            for i in range(0, len(total_event_path)):
                conn.sendall(str.encode(str(total_event_path[i])))
                sleep(5.02)
            while True:
                command=input('ENTER "RUN" TO START THE BOT : ')
                if command!='RUN':
                    continue
                conn.sendall(str.encode(command))
                break
            
            sleep(6)
            print(path_event_node)
            while True:
                    try:
                        event_coordinates = [tuple(x) for x in pd.read_csv('event_location.csv', header=None).values.tolist()]
                        break
                    except:
                        continue
            
            event_coordinates.append((760,40,20))
            print(event_coordinates)
            j=0
            #event_coordinates=[tuple(x) for x in pd.read_csv('live_data.csv', header=None).values.tolist()]
            for event in path_event_node:
                while True:
                    try:
                        coordinate_100 = [tuple(x) for x in pd.read_csv('coordinate_100.csv', header=None).values.tolist()]
                    except:
                        continue
                    print(coordinate_100)
                    x,y,w=event_coordinates[i_location[j]]
                    x_100,y_100=coordinate_100[0]
                    x=x+w//2
                    if x_100>x-60 and x_100<x+80 and y_100-30<y and y_100>y-60:
                        # sleep(2);
                        conn.sendall(str.encode("Yes"))
                        j+=1
                        print("hello")
                        sleep(5)
                        break
                    
                        
            
            print("Connection Closed")
            s.close()
        except:
            s.close()

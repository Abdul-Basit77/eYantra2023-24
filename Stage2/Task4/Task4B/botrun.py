import socket
from time import sleep
import signal		
import sys		

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

ip = "192.168.143.21"     #Enter IP address of laptop after connecting it to WIFI hotspot
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, 8002))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        command=input('ENTER "RUN" TO START THE BOT : ')
        conn.sendall(str.encode(command))
        sleep(10)
        print("Connection Closed")
        s.close()

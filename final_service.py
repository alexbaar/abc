import datetime
import os
import re
import socket
import threading
import time
import numpy as np
#for video
import cv2

import rclpy
from rclpy.node import Node

from drone_interfaces.srv import Move
from drone_interfaces.srv import Battery
from drone_interfaces.srv import Height

from std_srvs.srv import Empty
import numpy as np
from djitellopy import Tello

# for object detection
from ultralytics import YOLO
import cvzone
import math

import socket
from time import sleep
import curses

INTERVAL = 0.2





class DroneServer(Node):
    drone_response = "no_response"
    drone_response_int = 1

    """
    A video stream track that transforms frames from an another track.
    """

    kind = "video"

    
    def __init__(self):
        super().__init__('drone_server')




        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.bind((self.local_ip, self.local_port))



        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.tello_ip = '192.168.10.1'
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.MAX_TIME_OUT = 15.0
        self.socket.sendto('command'.encode('utf-8'), self.tello_address)    
        self._running=False

        
        
 

      
        
# services

        self.srv = self.create_service(Move, 'move_forward', self.move_forward_callback)
        self.srv = self.create_service(Move, 'move_backward', self.move_backward_callback)

                
        self.srv = self.create_service(Move, 'move_left', self.move_left_callback)
        self.srv = self.create_service(Move, 'move_right', self.move_right_callback)

        self.srv = self.create_service(Empty, 'flip_forward', self.flip_forward_callback)
        self.srv = self.create_service(Empty, 'flip_backward', self.flip_backward_callback)
        
        self.srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.srv = self.create_service(Empty, 'land', self.land_callback)
      
        self.srv = self.create_service(Empty, 'streamon', self.video_on_callback)
        self.srv = self.create_service(Empty, 'streamoff', self.video_off_callback)

        self.srv = self.create_service(Battery, 'battery', self.battery_status_callback)
        #self.pub_battery = self.node.create_publisher(BatteryState, 'battery', 1) # for pub sub
        
      
        self.srv = self.create_service(Height, 'height', self.get_height_callback)

    # https://github.com/dji-sdk/Tello-Python/blob/master/tello_state.py
    def report(str):
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.addstr(0, 0, str)
        stdscr.refresh()

    def send_command(self, msg):
        command = msg #the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        print('sending command: %s to %s' % (command, self.tello_ip))
        
        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.tello_ip))

    def terminate(self):
        self._running = False
        self.video.release()
        cv2.destroyAllWindows()




    

    #get video frame
    def recv(self):


        self._running = True  
        # Tello camera                                            
        self.video = cv2.VideoCapture("udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=4912") 
        self.video.set(3,1280)
        self.video.set(4,720)

        # Pre-recorded video
        #self.video = cv2.VideoCapture('/home/vboxuser/dev_ws/src/final_drone/final_drone/videos/vid1.mp4')
        
        # model for obj recognition
        model = YOLO('/home/vboxuser/dev_ws/src/final_drone/final_drone/yoloWeights/yolov8n.pt')

        classNames = ['person','bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train', 'truck', 'boat',
                      'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
                      'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
                      'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
                      'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 
                      'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 
                      'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'sofa', 'pottedplant', 'bed',
                      'diningtable', 'toilet', 'tvmonitor', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 
                      'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                      'teddy bear', 'hair drier', 'toothbrush'

        ]

        
        """ Handler for Tello states message """
        while self._running:
            try:
                ret, frame = self.video.read()
                if ret:
                    # Resize frame
                    height, width, _ = frame.shape
                    new_h = int(height )
                    new_w = int(width )
                    h=new_h
                    w=new_w

                    # Resize for improved performance
                    new_frame = cv2.resize(frame, (new_w, new_h))

                    # object detection -> it really slows things down unfortunately, but works
                    results = model(new_frame,stream=True)
                    for r in results:
                        boxes = r.boxes
                        for box in boxes:
                            # option 1 -> open cv
                            x1,y1,x2,y2 = box.xyxy[0]
                            x1,y1,x2,y2 = int(x1),int(y1),int(x2),int(y2)  
                            #cv2.rectangle(new_frame,(x1,y1),(x2,y2),(0,150,0),2)

                            #option 2  -> cv zone                       
                            w,h = x2-x1,y2-y1
                            cvzone.cornerRect(new_frame,(x1,y1,w,h))

                            # how confident % that the object is what it is; round it up
                            conf = math.ceil((box.conf[0]*100))

                            #show what class was the object detected assigned
                            cls = int(box.cls[0])

                            # display the confidence level and the class name in a rectangle box
                            cvzone.putTextRect(new_frame,f'{classNames[cls]}, conf={conf} %',(max(0,x1),max(35,y1)),scale=1,thickness = 1)

                    # obj detection end

                    #detect face in our frame
                    faceCascade = cv2.CascadeClassifier('/home/vboxuser/dev_ws/src/final_drone/final_drone/cascades/haarcascade_frontalface_default.xml')
                    eyeCascade = cv2.CascadeClassifier('/home/vboxuser/dev_ws/src/final_drone/final_drone/cascades/haarcascade_eye.xml')

                    imgGray = cv2.cvtColor(new_frame,cv2.COLOR_BGR2GRAY)
                    faces = faceCascade.detectMultiScale(imgGray,1.2,4)  #changeable values

                    # detect only the closest face; calc centre point of face
                    faceListCentre =[]
                    faceListArea = []
                        
                    if len(faces) == 0:
                        print("No faces found")
                    else:

                        for(x,y,w,h) in faces:
                        # mark face with rectangle box
                            cv2.rectangle(new_frame,(x,y),(x+w,y+h),(255,0,0),2)
                            

                            roi_gray = imgGray[y:y+h, x:x+w]
                            roi_color = new_frame[y:y+h, x:x+w]
                            eyes = eyeCascade.detectMultiScale(roi_gray,1.2,4)
                            

                        for(ex,ey,ew,eh) in eyes:
                            # mark eyes with rectangle box
                            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(170,0,93),2)
                            
                        
                    # Display the resulting frame
                    cv2.imshow('Tello', new_frame)
                # Wait for display image frame 
                    if cv2.waitKey(1) & 0xFF == ord('q'): 
                        break
                #self.save_frame_camera_cycle(0, 'data/temp', 'camera_capture_cycle', 3)
                cv2.waitKey(1)
            except Exception as err:
                print(err)






    def video_on_callback(self, request, response):
        
        self.get_logger().info('Incoming request: video on')
        command = "streamon"
        time.sleep(3)
        print("sending streamon")
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
    
        print("starting video thread")
        
        recvThread = threading.Thread(target=self.recv)
        recvThread.start()
        print("video thread started")
        #self.save_frame_camera_cycle(0, 'data/temp', 'camera_capture_cycle', 3)
        return response

    def video_off_callback(self, request, response):
        self.get_logger().info('Incoming request: video off')
        command = "streamoff"
        time.sleep(3)
        print("sending streamoff")
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
    
        print("stopping video thread")
        
        termThread = threading.Thread(target=self.terminate)
        termThread.start()
        print("video thread stopped")
        return response
        
    def move_forward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Forward: %dcm' % (request.distance))
        command = "forward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_backward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Backward: %dcm' % (request.distance))
        command = "backward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response
    
    def move_left_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move left: %dcm' % (request.distance))
        command = "left %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response
    
    def move_right_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move right: %dcm' % (request.distance))
        command = "right %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def flip_forward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip forward')
        command = "flip f"
        print(command)
        self.send_command(command)
        return response

    def flip_backward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip backward')
        command = "flip b"
        print(command)
        self.send_command(command)
        return response
    
    
    def takeoff_callback(self, request, response):
        self.get_logger().info('Incoming request: Takeoff')
        command = "takeoff"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response
        
    def land_callback(self, request, response):
        
        self.get_logger().info('Incoming request: Land')
        command = "land"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response
    
    def battery_status_callback(self,request,response):
        global drone_response
        self.get_logger().info('Incoming request: : battery')
        command = "battery?"
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(2) #wait for the response
        response.battery_status = int(float(drone_response[2:4]))
        
        return response
    
    # https://github.com/dji-sdk/Tello-Python/blob/master/tello_state.py
    def get_height_callback(self,request,response):
        try:
            index = 0
            while True:
                index += 1
                response, ip = socket.recvfrom(1024)
                if response == 'ok':
                    continue
                out = response.replace(';', ';\n')
                out = 'Tello State:\n' + out
                self.report(out)
                sleep(INTERVAL)
                response.height = out
                print(response)
                return response
        except KeyboardInterrupt:
            curses.echo()
            curses.nocbreak()
            curses.endwin()

    

    
    def _receive_thread(self):
        global drone_response
        global drone_response_int
        #Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
                drone_response = str(self.response) #convert from byte string to string
                #drone_response_int = int(float((self.response))) # numerical data 
            except socket.error as exc:
                print("Caught exception socket.error : %s" % exc)



                

def main(args=None):
    rclpy.init(args=args)

    node = DroneServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()






if __name__ == "__main__":
    main()

'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information created by David Zhu

   Kre8 Technology retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of Kre8 Technology is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this file.
   ========================================================================*/
'''

import Tkinter as tk
import time
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as np
import threading
from tk_hamster_GUI import *

UPDATE_INTERVAL = 30

comm = None
gMaxRobotNum = 2; # max number of robots to control
# gRobotList = None
gQuit = False
m = None

#objects in the world
rectF = [0, 50, 40, -50]
rectC = [0, 140, -100, 180]
rectB = [-100, 180, -140, 80]
rectE = [0, -140, -100, -180]
rectD = [-100, -180, -140, -80]
rectA = [-220, -20, -260, 20]
robot_index = 0

class VirtualWorldGui:
    def __init__(self, vWorld, joystick, m):
        self.vworld = vWorld
        self.joystick = joystick
        self.navigation_queue = Queue.Queue()
        self.gRobotList = None

        self.button0 = tk.Button(m,text="Grid")
        self.button0.pack(side='left')
        self.button0.bind('<Button-1>', self.drawGrid)

        self.button1 = tk.Button(m,text="Clear")
        self.button1.pack(side='left')
        self.button1.bind('<Button-1>', self.clearCanvas)

        self.button2 = tk.Button(m,text="Reset")
        self.button2.pack(side='left')
        self.button2.bind('<Button-1>', self.resetvRobot)

        self.button3 = tk.Button(m,text="Map")
        self.button3.pack(side='left')
        self.button3.bind('<Button-1>', self.drawMap)

        self.button4 = tk.Button(m,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(m,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(m,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button7 = tk.Button(m,text="Localize")
        self.button7.pack(side='left')
        self.button7.bind('<Button-1>', self.localize)

        self.button8 = tk.Button(m,text="ROBOT1")
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>', self.start_navigating)

        self.button9 = tk.Button(m,text="ROBOT2")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', self.start_navigating_2)

        self.button10 = tk.Button(m,text="ROBOT3")
        self.button10.pack(side='left')
        self.button10.bind('<Button-1>', self.start_navigating_3)

        self.button11 = tk.Button(m,text="Exit")
        self.button11.pack(side='left')
        self.button11.bind('<Button-1>', stopProg)

    # reset to the starting position for 3-2
    def resetvRobot(self, event=None):
        # point 0
        self.vworld.vrobot.y = 150
        self.vworld.vrobot.a = math.pi/2
        # # point 1
        # self.vworld.vrobot.y = 150
        # self.vworld.vrobot.x = 240
        # self.vworld.vrobot.a = math.pi
        # point 2
        # self.vworld.vrobot.y = -140
        # self.vworld.vrobot.x = 240
        # self.vworld.vrobot.a = (3*math.pi)/2
        # point 3
        # self.vworld.vrobot.y = -140
        # self.vworld.vrobot.x = 80
        # self.vworld.vrobot.a = math.pi


    def toggleTrace(self, event=None):
        if self.vworld.trace:
            self.vworld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vworld.trace = True
            self.button4["text"] = "No Trace"

    def toggleProx(self, event=None):
        if self.vworld.prox_dots:
            self.vworld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vworld.prox_dots = True
            self.button5["text"] = "No Prox Dots"

    def toggleFloor(self, event=None):
        if self.vworld.floor_dots:
            self.vworld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vworld.floor_dots = True
            self.button6["text"] = "No Floor Dots"

    def drawMap(self, event=None):
        self.vworld.draw_map()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def clearCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        vrobot = self.vworld.vrobot
        vcanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
        vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
        vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
        while (drawQueue.qsize() > 0):
            drawCommand = drawQueue.get()
            drawCommand()

    def localize(self, i):
        xs = []
        ys = []
        ref = [300, -180, 40, -180, 120]
        vrobot = self.vworld.vrobot
        print "proximity left = ", vrobot.dist_l
        print "proximity right = ", vrobot.dist_r
        for _ in range(10):
            xs.append(2)
            ys.append(vrobot.dist_l)
            xs.append(38)
            ys.append(vrobot.dist_r)
            time.sleep(0.05)
        m, b = calculate_least_sqs(xs, ys)
        perp_dist = sum(ys)/float(len(ys)) + 20 # 20 is half of the robot's length
        angle = math.atan(m)
        self.localize_to(angle, perp_dist, i, ref)

    def localize_to(self, angle, perp_dist, i, ref):
        print "LOCALIZING WITH ", angle, perp_dist
        print "DIST", math.cos(angle)*perp_dist
        vrobot = self.vworld.vrobot
        if i==0:
            vrobot.a = angle + math.pi/2
            vrobot.x = ref[i] - math.cos(angle)*perp_dist
        elif i==1:
            vrobot.a = angle + math.pi
            vrobot.x = ref[i] + math.cos(angle)*perp_dist
        elif i==2:
            vrobot.a = angle + 3*math.pi/2.0
            vrobot.y = ref[i] + math.cos(angle)*perp_dist
        elif i==3:
            vrobot.a = angle + math.pi/2
            vrobot.x = ref[i] + math.cos(angle)*perp_dist
        elif i==4:
            vrobot.a = angle + math.pi/2
            vrobot.x = ref[i] + math.cos(angle)*perp_dist

    def start_navigating(self, event=None):
        print "NAVIGATING ROBOT 1"
        navigate_vrobot_thread = threading.Thread(target=self.navigate_robot, args=(self.navigation_queue,))
        navigate_vrobot_thread.daemon = True
        navigate_vrobot_thread.start()
        self.navigation_queue.put("Start Navigating")

    def start_navigating_2(self, event=None):
        global robot_index
        print "NAVIGATING ROBOT 2"
        robot_index = 1
        self.navigation_queue.put("Start Navigating")

    def start_navigating_3(self, event=None):
        global robot_index
        print "NAVIGATING ROBOT 3"
        robot_index = 2
        self.navigation_queue.put("Start Navigating")

    # aligns robot to face an obstacle head on
    def align_robot(self):
        vrobot = self.vworld.vrobot
        if (vrobot.dist_r > vrobot.dist_l):
            self.joystick.move_left()
            while(vrobot.dist_r > vrobot.dist_l):
                time.sleep(0.05)
        else:
            self.joystick.move_right()
            while(vrobot.dist_l > vrobot.dist_r):
                time.sleep(0.05)
        self.joystick.stop_move()

    def navigate_robot(self, navigation_queue):
        global robot_index
        joystick = self.joystick
        vworld = self.vworld
        time.sleep(1) # give time for robot to connect.
        vrobot = vworld.vrobot

        while not joystick.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        while (robot_index != 3):
            if (navigation_queue.qsize() > 0):
                print navigation_queue.get()
                if (vrobot.y != 150):
                    # need to draw robot at last position
                    print "drawing the last moved robot!"
                    print robot_index
                    coords = self.vworld.canvas.coords(vrobot.poly_id)
                    print coords
                    self.vworld.canvas.create_polygon(coords, outline="blue")

                self.resetvRobot()
                # put a random delay
                print "navigating robot ", robot_index
                time.sleep(0.5)
                # point 1
                joystick.move_up()
                # fake moving
                # time.sleep(1)
                # joystick.stop_move()
                # print "done navigating robot ", robot_index

                # point 1
                self.follow_wall(0)
                joystick.move_right()
                while (vrobot.a < math.pi/2):
                    time.sleep(0.05)
                    print vrobot.a
                joystick.stop_move()
                self.move_to_prox(30)
                joystick.turn_clockwise(math.pi)
                print "first wall done"
                # # point 2
                # # TODO: localize to walls
                self.follow_wall(1)
                self.move_to_prox(30)
                joystick.turn_clockwise((3*math.pi)/2)
                print "second wall done"
                # point 3
                self.follow_wall(2)
                self.move_to_prox(25)
                print "third wall done"
                joystick.turn_counterclockwise(math.pi)
                # point 4
                self.move_to_prox(70)
                joystick.move_down()
                time.sleep(0.4)
                joystick.stop_move()
                joystick.turn_clockwise((3*math.pi)/2)
                self.move_through()
                # TODO: send robots to different corners
                print "finished"

    def follow_wall(self, wall_index):
        joystick = self.joystick
        vworld = self.vworld
        vrobot = vworld.vrobot
        if (wall_index == 0): # TODO: this doesn't work right
            wall_angle = 0.1
            forward_angle = math.pi/2
            follow_range = 300
            wall_localize_index = -1
            prox_value = 25
        elif (wall_index == 1):
            wall_angle = math.pi/2
            forward_angle = math.pi
            follow_range = 70
            wall_localize_index = 0
            prox_value = 25
        elif (wall_index == 2):
            wall_angle = math.pi
            forward_angle = (3*math.pi)/2
            follow_range = 40
            wall_localize_index = 1
            prox_value = 30

        # move then reallign to wall
        while (True):
            prox_l, prox_r = joystick.read_proximity()
            if (prox_l > 60 and prox_r > 60): break
            joystick.move_up()
            for _ in range(follow_range):
                print "follow_wall:", vrobot.dist_l, vrobot.dist_r
                if ((vrobot.dist_l and vrobot.dist_l < 40) or (vrobot.dist_r and vrobot.dist_r < 40)):
                    print "breaking:", vrobot.dist_l, vrobot.dist_r
                    break
                time.sleep(0.05)
            joystick.stop_move()
            joystick.turn_counterclockwise(wall_angle)
            for _ in range(3):
                self.align_robot()
            if (wall_localize_index != -1):
                self.localize(wall_localize_index)  # TODO: this doesn't really work
            self.move_to_prox(prox_value)
            joystick.turn_clockwise(forward_angle)
            prox_l, prox_r = joystick.read_proximity()
            if (prox_l > 60 and prox_r > 60): break


    def move_through(self):
        joystick = self.joystick
        vworld = self.vworld
        vrobot = vworld.vrobot

        for _ in range(80):
            if ((vrobot.dist_l < 35 or vrobot.dist_r < 35) and not (not vrobot.dist_r and not vrobot.dist_l)):
                if (vrobot.dist_l > vrobot.dist_r):
                    # turn right
                    joystick.move_right()
                else:
                    joystick.move_left()
            else:
                joystick.move_up()
            time.sleep(0.1)
        joystick.stop_move()

    def move_to_prox(self, proxValue):
        joystick = self.joystick
        vworld = self.vworld
        vrobot = vworld.vrobot

        if (avg([vrobot.dist_l, vrobot.dist_r]) < proxValue):
            # move back
            joystick.move_down()
            while (vrobot.dist_l < proxValue or vrobot.dist_r < proxValue):
                time.sleep(0.02)
        joystick.move_up()
        while (vrobot.dist_l > proxValue or vrobot.dist_r > proxValue):
            time.sleep(0.02)
        joystick.stop_move()

def avg(values):
    return sum(values)/len(values)

def calculate_least_sqs(xvalues, yvalues):
    x = np.array(xvalues)
    y = np.array(yvalues)
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y)[0]
    return (m, b)

class Joystick:
    def __init__(self, comm, m, rCanvas):
        self.gMaxRobotNum = 1
        self.gRobotList = comm.robotList
        self.m = m
        self.vrobot = virtual_robot()
        self.vrobot.t = time.time()
        # calibrations for robot 1
        self.noise_prox = 25 # noisy level for proximity
        self.noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        self.p_factor = 1.2 #proximity conversion - assuming linear
        self.d_factor = 1.04 #travel distance conversion
        self.a_factor = 17.5 # rotation conversion, assuming linear

        rCanvas.bind_all('<w>', self.move_up)
        rCanvas.bind_all('<s>', self.move_down)
        rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.move_right)
        rCanvas.bind_all('<x>', self.stop_move)
        rCanvas.pack()

    # joysticking the robot
    def move_up(self, event=None):
        if self.gRobotList and robot_index < len(self.gRobotList):
            robot = self.gRobotList[robot_index]
            self.vrobot.sl = 30
            self.vrobot.sr = 30
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList and robot_index < len(self.gRobotList):
            robot = self.gRobotList[robot_index]
            self.vrobot.sl = -30
            self.vrobot.sr = -30
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList and robot_index < len(self.gRobotList):
            robot = self.gRobotList[robot_index]
            self.vrobot.sl = -15
            self.vrobot.sr = 15
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_right(self, event=None):
        if self.gRobotList and robot_index < len(self.gRobotList):
            robot = self.gRobotList[robot_index]
            self.vrobot.sl = 15
            self.vrobot.sr = -15
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def stop_move(self, event=None):
        if self.gRobotList and robot_index < len(self.gRobotList):
            robot = self.gRobotList[robot_index]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def turn_clockwise(self, angle):
        if self.gRobotList and robot_index < len(self.gRobotList):
            self.move_right()
            while((self.vrobot.a % (2*math.pi)) < angle):
                time.sleep(0.1)
            self.stop_move()

    def turn_counterclockwise(self, angle):
        if self.gRobotList and robot_index < len(self.gRobotList):
            self.move_left()
            while((self.vrobot.a % (2*math.pi)) > angle):
                time.sleep(0.1)
            self.stop_move()

    def read_proximity(self):
        robot = self.gRobotList[robot_index]
        return robot.get_proximity(0), robot.get_proximity(1)

    def move_to(self, x, y):
        # adjust x
        print "MOVING TO ", x, y
        if (math.cos(self.vrobot.a) > 0): #facing right
            if (x > self.vrobot.x): # target x is to right of x
                self.move_up()
            else:
                self.move_down()
        else:
            if (x > self.vrobot.x): # target x is to right of x
                self.move_down()
            else:
                print "SETTING WHEELS"
                self.move_up()
        if (x > self.vrobot.x):
            while (self.vrobot.x < x):
                print "MOVING x ", x, y
                time.sleep(0.1)
        else:
            while (self.vrobot.x > x):
                print "MOVING x ", x, y
                time.sleep(0.1)

        # adjust y
        if self.turn_helper(math.sin(self.vrobot.a), math.cos(self.vrobot.a)) == "up":
            if (self.vrobot.y < y):
                self.move_up()
            else:
                self.move_down()
        else:
            if (self.vrobot.y > y):
                self.move_up()
            else:
                self.move_down()
        if (y > self.vrobot.y):
            while (self.vrobot.y < y):
                time.sleep(0.1)
        else:
            while (self.vrobot.y > y):
                time.sleep(0.1)

    def turn_helper(self, sin, cos):
        if (sin > 0 and cos > 0):
            self.turn_counterclockwise(0)
            return "up"
        elif (sin < 0 and cos > 0):
            self.turn_clockwise(math.pi)
            return "down"
        elif (sin < 0 and cos < 0):
            self.turn_counterclockwise(math.pi)
            return "down"
        else:
            self.turn_clockwise(0)
            return "up"

    def update_virtual_robot(self):
        # this is the robot modeling code - below is a very simple and inaccurate
        # model, as example of how to use the GUI toolkit you need to create you
        # own model

        noise_prox = self.noise_prox
        noise_floor = self.noise_floor
        p_factor = self.p_factor
        d_factor = self.d_factor
        a_factor = self.a_factor

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot"

        while not gQuit:
            if self.gRobotList is not None and robot_index < len(self.gRobotList):
                robot = self.gRobotList[robot_index]

                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr:
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor
                #update sensors
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                if (prox_l > noise_prox):
                    self.vrobot.dist_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False

                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False
            time.sleep(0.1)

def stopProg(event=None):
    global gQuit
    global m
    m.quit()
    gQuit = True
    print "Exit"

def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def main(argv=None):
    global m
    global comm
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    m = tk.Tk() #root
    drawQueue = Queue.Queue(0)

    #creating tje virtual appearance of the robot
    canvas_width = 700 # half width
    canvas_height = 380 # half height
    rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)

    joystick = Joystick(comm, m, rCanvas)

    # visual elements of the virtual robot
    poly_points = [0,0,0,0,0,0,0,0]
    joystick.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
    joystick.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
    joystick.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
    joystick.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    joystick.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")

    time.sleep(1)

    update_vrobot_thread = threading.Thread(target=joystick.update_virtual_robot)
    update_vrobot_thread.daemon = True
    update_vrobot_thread.start()

    #create the virtual worlds that contains the virtual robot
    vWorld = virtual_world(drawQueue, joystick.vrobot, rCanvas, canvas_width, canvas_height)

    boxes = [[0, 0, 40, 100] ,[40, 80, 140, 120] ,[120, -80, 200, 0] ,[0, -80, 40, -180] ,[40, -180, 300, -220] ,[300, -180, 340, 200] ,[300, 200, 0, 240]]
    for box in boxes:
        vWorld.add_obstacle(box)

    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, joystick, m)

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    m.mainloop()


    for robot in joystick.gRobotList:
        robot.reset()
    comm.stop()
    comm.join()


if __name__ == "__main__":
    main()
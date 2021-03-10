#!/usr/bin/env python2
import Tkinter as tk
# import PIL
from PIL import Image, ImageTk

import rospy
import motion_stable_control.msg
import actionlib
import os

ratio = 1.0

isAction = False

def DrawScale():
    pass

def g2gStart():
    print "Started"

def g2gFeedback(feedback):
    print (" New Position : %lf , %lf , %lf ")%( feedback.Position.x , feedback.Position.y , feedback.Position.z )

def g2gFinish(state, result):
    print "Finished"

def UpdateXY(event):
    '''
    show x, y coordinates of mouse click position
    event.x, event.y relative to ulc of widget (here root)
    '''
    # xy relative to ulc of root
    # xy = 'root x=%s  y=%s' % (event.x, event.y)
    # optional xy relative to blue rectangle
    px = ( event.x - (x1+x2) / 2 )/20
    py = ( (y1 + y2)/2 - event.y ) / 20
    xy = 'rectangle x=%s  y=%s' % (px, py)
    root.title(xy)
    print(xy)

    global isAction
    if( isAction ):
        g2g_client.cancel_goal()
        # global isAction
        isAction = False
    # Creates a goal to send to the action server.
    goal = motion_stable_control.msg.Go2GoalGoal()
    goal.Goal.x = px
    goal.Goal.y = py
    isAction = True
    g2g_client.send_goal(goal,
                        active_cb=g2gStart,
                        feedback_cb=g2gFeedback,
                        done_cb=g2gFinish
    )

rospy.init_node('set_goal_gui', anonymous=True)
# rate = rospy.Rate(10)
print "INIT_NODE CALLED"


root = tk.Tk()
root.title("Mouse click within blue rectangle ...")

# create a canvas for drawing
w = 400
h = 400
cv = tk.Canvas(root, width=w, height=h, bg='white')

# draw a blue rectangle shape with 
# upper left corner coordinates x1, y1
# lower right corner coordinates x2, y2
x1 = 20
y1 = 30
x2 = 380
y2 = 370
cv.create_rectangle(x1, y1, x2, y2, fill="blue", tag='rectangle')
#cv.create_oval(199, 199, 201, 201, fill="red", tag='oval')

directory_path = os.path.dirname(__file__)
file_path = os.path.join(directory_path, 'turtle2.png')
img = tk.PhotoImage(file=file_path)      
cv.create_image(200,200, image=img)   

# bind left mouse click within shape rectangle
#cv.tag_bind('rectangle', '<Button-1>', showxy)
cv.bind('<Button-1>', UpdateXY)
cv.pack()


g2g_client = actionlib.SimpleActionClient('Go2GoalAction',
    motion_stable_control.msg.Go2GoalAction)
g2g_client.wait_for_server()

root.mainloop()



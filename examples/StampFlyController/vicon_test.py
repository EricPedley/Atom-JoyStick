from pyvicon_datastream import tools
import time
import numpy as np

VICON_TRACKER_IP = "192.168.30.153"
OBJECT_NAME = "stampfly"

mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
while(True):
    print("Getting position...")
    position_data = mytracker.get_position(OBJECT_NAME)
    _, frame_no, objects = position_data
    if len(objects)>0:
        sixdof_pose = np.array(objects[0][2:])
        print(f"Position: {sixdof_pose}")
    else:
        print("No objects")
    time.sleep(0.5)
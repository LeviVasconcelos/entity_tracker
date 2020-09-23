import sys
import cv2
import time

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from entity_tracker.msg import AddEntityRequestMsg, EntitiesFrameMsg, TrackEntityMsg
import numpy as np

class TrackEntity:
    def __init__(self, roi, label, frame):
        self.x, self.y, self.w, self.h = roi
        self.roi = roi
        self.label = label
        self.tracker = None
        self.success = False
        self.lostFrames = 0

    def update(self, frame):
        if self.tracker is not None:
            (self.success, self.roi) = self.tracker.update(frame)
            if self.success:
                self.lostFrames = 0
            else:
                self.lostFrames += 1
            return self.success
        else:
            return None

    def toRosMessage(self):
        msg = TrackEntityMsg()
        msg.label = self.label
        msg.success = self.success
        msg.roi = self.roi
        msg.lostFrames = self.lostFrames
        return msg

class TrackEntityCSRT(TrackEntity):
    def __init__(self, roi, label, frame):
        TrackEntity.__init__(self, roi, label, frame)
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(frame, roi)

class Tracker:
    def __init__(self):
        self.entities = []

    def addEntity(self, entity):
        self.entities.append(entity)
    
    def update(self, frame):
        for entity in self.entities:
            entity.update(frame)

    def toRosMessage(self, frame):
        msg = EntitiesFrameMsg()
        for e in self.entities:
            msg.entities.append(e.toRosMessage())
        # TODO: Add header
        msg.img.format = "jpeg"
        msg.img.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
        return msg

class RosTracker:
    def __init__(self, image_topic, add_topic, publish_topic):
        self.tracker = Tracker()
        self.publish_topic = publish_topic
        self.image_topic = image_topic
        self.add_topic = add_topic
        self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, self._callback, queue_size=5)
        self.add_tracker_sub = rospy.Subscriber(self.add_topic, AddTracker, self._callback_add_tracker, queue_size=1)
        self.track_pub = rospy.Publisher(śelf.publish_topic, EntitiesFrame)

    def _callback(self, data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        self.tracker.update(image_np)
        self.track_pub(self.tracker.toRosMessage(image_np))

    def _callback_add_tracker(self,  ros_data):
        np_arr = np.fromstring(ros_data.img, np.uint8)
        entity = TrackEntityCSRT(ros_data.roi, ros_data.label, np_arr)
        self.tracker.AddEntity(entity)

if __name__ == "__main__":
    image_topic = "/head_front_camera/image_raw/compressed"
    add_topic = "/tracker/add"
    publish_topic = "/tracker/entities"
    tracker = RosTracker(image_topic, add_topic, publish_topic)
    rospy.init_node('entity_tracker', anonymous=True)
    try:
        rospy.spin()
    except: KeyboardInterrupt:
        print("Shutting down ROS Entity Tracker module")

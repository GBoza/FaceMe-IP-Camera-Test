"""
In this example, we demonstrate how to create simple camera viewer using Opencv3 and PyQt5

Author: Berrouba.A
Last edited: 21 Feb 2018
"""

# import system module
import sys

# import Opencv module
import cv2
#import faceme
import facemecv
#
import paho.mqtt.client as mqtt
import json
import datetime
import threading
from threading import Lock

from os.path import exists
import os

# import config file
from config import *

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    #client.subscribe("gb/#")

def on_message(client, userdata, msg):
    print(msg.topic+" " +str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)


class Camera:
    last_frame = None
    last_ready = None
    lock = Lock()

    def __init__(self, rtsp_link):
        capture = cv2.VideoCapture(rtsp_link)
        thread = threading.Thread(target=self.rtsp_cam_buffer, args=(capture,), name="rtsp_read_thread")
        thread.daemon = True
        thread.start()

    def rtsp_cam_buffer(self, capture):
        while True:
            with self.lock:
                self.last_ready, self.last_frame = capture.read()
                #print("frame")


    def getFrame(self):
        if (self.last_ready is not None) and (self.last_frame is not None):
            return self.last_frame.copy()
        else:
            return None

class FaceMeCamTest():
    # class constructor
    def __init__(self):

        self.stage = 0
        self.faces_imglist = list()
        self.frameCount = 0
        facemecv.initialize_SDK(license_key = LICENSE_KEY, password = None)

        #self.cap = cv2.VideoCapture('./videosample.mp4')
        #self.cap = cv2.VideoCapture(RTSP_LINK)
        self.cam = Camera(RTSP_LINK)

    # view camera
    def procFrame(self):
        # read image in BGR format
        #ret, image = self.cap.read()
        #if ret == False:
        #    return
        image = self.cam.getFrame()

        if image is None:
            return
        #resize frame
        #image = cv2.resize(image, (1024, 768), interpolation = cv2.INTER_NEAREST)

        # flip image (not used with test video file)
        #image = cv2.flip(image, 1)
        
        # vertical flip image 
        image = cv2.flip(image, 0)

        # convert image to RGB format
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        facemeimage = facemecv.convert_image_to_faceimage(image)
        
        #orig_image = image.copy()
        
        detect_results = facemecv.detect_face_from_faceimage(facemeimage)
        
        #print("Frame:", self.frameCount, image.shape)
        #print("Time:", str(datetime.datetime.now()))
        if len(detect_results) > 0:
            print("Faces: ", len(detect_results)) #faces detectados
            for detect_result in detect_results:
                #face_img = cv2.rectangle(image, detect_result['boundingBox'][0], detect_result['boundingBox'][1], (255,0,0),2)
                
                #crop face from image
                face_img = image[detect_result['boundingBox'][0][1]:detect_result['boundingBox'][1][1],detect_result['boundingBox'][0][0]:detect_result['boundingBox'][1][0]]
                print(detect_result['boundingBox'])
                
                #pose = facemecv.get_pose_from_faceimage(facemeimage)
                #print(pose)
                #if pose is None:
                #    continue
                #(pose_yaw, pose_pitch, pose_roll) = pose

                # convert image to FaceMeImage format
                faceme_face_img = facemecv.convert_image_to_faceimage(face_img)
                # search similar faces
                results = facemecv.search_similar_face(faceme_face_img)

                if results == None: # failed or similar face not found
                    print("No similar faces.")
                    json_data = {}
                    json_data['datetime'] = str(datetime.datetime.now())
                    json_data['name'] = "Visitor"
                    client.publish(MQTT_TOPIC_VISITOR, payload=json.dumps(json_data), qos=0, retain=False)
                    with open(LOG_FILENAME, 'a') as fp:
                        fp.write("{}: Name=Visitor FaceConfidence={} Rect={}\r\n".format(datetime.datetime.now(), str(detect_result['faceConfidence']), str(detect_result['boundingBox'])))
                    filename = "./images/detection/visitor_{}.jpg".format(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
                    filename = os.path.abspath(filename)
                    if exists(filename) == False:
                        cv2.imwrite(filename, face_img)
                else:
                    print("Similar Faces: ", len(results))
                    for result in results:
                        print("Name:",result['name'])
                        print("Confidence:", result['confidence'])
                        json_data = {}
                        json_data['datetime'] = str(datetime.datetime.now())
                        json_data['name'] = result['name']
                        json_data['confidence'] = result['confidence']
                        client.publish(MQTT_TOPIC_USER, payload=json.dumps(json_data), qos=0, retain=False)
                        with open(LOG_FILENAME, 'a') as fp:
                            fp.write("{}: Name={} Confidence={} Rect={}\r\n".format(datetime.datetime.now(), result['name'], result['confidence'], str(detect_result['boundingBox'])))
                        filename = "./images/detection/user_{}_{}.jpg".format(result['name'], datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
                        filename = os.path.abspath(filename)
                        if exists(filename) == False:
                            cv2.imwrite(filename, face_img)
        self.frameCount += 1


    def pushButtonRegister_pressed(self):

        if len(self.faces_imglist) == 0:
            return

        results = None
        for face in self.faces_imglist:
            facemeimg = facemecv.convert_image_to_faceimage(face)
            results = facemecv.search_similar_face(facemeimg)

        if results is None:
            facemeimg = facemecv.convert_image_to_faceimage(self.faces_imglist[0])
            facemecv.register_user_with_faceimage(username, facemeimg)
            for face in self.faces_imglist[1:]:
                facemeimg = facemecv.convert_image_to_faceimage(face)
                results = facemecv.add_face_faceimage(username, facemeimg)
        elif len(results) == 0: ###
            facemeimg = facemecv.convert_image_to_faceimage(self.faces_imglist[0])
            facemecv.register_user_with_faceimage(username, facemeimg)
            for face in self.faces_imglist[1:]:
                facemeimg = facemecv.convert_image_to_faceimage(face)
                results = facemecv.add_face_faceimage(username, facemeimg)

if __name__ == '__main__':

    print("Starting at ", str(datetime.datetime.now()))
    with open(LOG_FILENAME, 'a') as fp:
        fp.write("Starting at {}\r\n".format(str(datetime.datetime.now())))
    test = FaceMeCamTest()
    while True:
        test.procFrame()
        #client.loop()
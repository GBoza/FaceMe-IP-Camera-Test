import os
import threading
import cv2
import array
import numpy as np
import queue
import time
import FaceMe.FaceMePython3SDK as FaceMe
from FaceMe.FaceMeSDK import FaceMeSDK, FR_FAILED, FR_SUCC
import sys
import platform
import tty
import sys
import termios
from fcntl import ioctl
import ctypes


def initialize_SDK(license_key, password = None):
    global faceMe_sdk
    faceMe_sdk = FaceMeSDK()
    options = {}
    options['chipset'] = 'NV'
    options['fdm'] = 'DEFAULT'
    options['frm'] = 'DEFAULT'
    options['minFaceWidthRatio'] = 0.05
    options['advfrm'] = 'DEFAULT'
    options['maxDetectionThreads'] = 0
    options['maxExtractionThreads'] = 0
    options['maxAdvancedExtractionThreads'] = 0
    options['preferredDetectionBatchSize'] = 0
    options['preferredExtractionBatchSize'] = 0
    options['searchPreference'] = 2

    ###
    if password != None:
        options['dataEncryption'] = True
        options['password'] = password

    app_bundle_path = os.path.dirname(os.path.realpath(__file__))
    app_cache_path = os.path.join(os.getenv('HOME'), ".cache")
    app_data_path = os.path.join(os.getenv('HOME'), ".local", "share")

    global cam
    #cam = cv2.VideoCapture(0)
    #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

    ret = faceMe_sdk.initialize(license_key, app_bundle_path, app_cache_path, app_data_path, options)
    if FR_FAILED(ret):
        print("Register license failed, return:", ret)
        return ret
    return ret

def get_frame_from_cam():
    s, img = cam.read()
    if s:
        return img
    return

def get_faceimage_from_cam():
    if  cam is None:
        return
    s, img = cam.read()
    if s == False:
        return
    cv2.imwrite('/dev/shm/tmpjpg.bmp', img) #temporary file
    ret, faceMe_img = faceMe_sdk.convert_to_faceMe_image('/dev/shm/tmpjpg.bmp')
    if FR_FAILED(ret):
        print('convert to Faceme Image failed')
        return
    return faceMe_img

def convert_image_to_faceimage(img):
    cv2.imwrite('/dev/shm/tmpbmp.bmp', img) #temporary file
    ret, faceMe_img = faceMe_sdk.convert_to_faceMe_image('/dev/shm/tmpbmp.bmp')
    if FR_FAILED(ret):
        print('convert to Faceme Image failed')
        return
    return faceMe_img

def detect_face_from_faceimage(faceme_img):
    options = {'detectOptions': FaceMe.FR_FEATURE_OPTION_ALL}
    ret, detect_result = faceMe_sdk.detect_face(faceme_img, options)
    if FR_FAILED(ret):
        print('detect_face failed, return: ', ret)
        return
    return detect_result

def extract_face_from_faceimage(faceme_img, detection_results):
    options = {'extractOptions': FaceMe.FR_FEATURE_OPTION_ALL}
    ret, extract_result = faceMe_sdk.extract_face(faceme_img, detection_results, options)
    if FR_FAILED(ret):
        print("extract_face failed, return: ", ret)
        return
    return extract_result

def recognize_face_from_faceimage(faceme_img):
    images = [faceme_img]
    options = {'extractOptions': FaceMe.FR_FEATURE_OPTION_ALL}
    ret, recognize_results = faceMe_sdk.recognize_faces(images, options)
    if FR_FAILED(ret):
        print("recognize face failed, return: ", ret)
        return
    return recognize_results

def get_pose_from_cam():
    faceme_image = get_faceimage_from_cam()
    if faceme_image == None:
        return
    recognize_results = recognize_face_from_faceimage(faceme_image)
    if recognize_results == None:
        return
    if len(recognize_results) > 0:
        pose_yaw = recognize_results[0]['pose_yaw']
        pose_pitch = recognize_results[0]['pose_pitch']
        pose_roll = recognize_results[0]['pose_roll']

        return (pose_yaw, pose_pitch, pose_roll)
    return

def get_pose_from_faceimage(faceme_image):
    if faceme_image is None:
        return
    recognize_results = recognize_face_from_faceimage(faceme_image)
    if recognize_results is None:
        return
    if len(recognize_results) > 0:
        pose_yaw = recognize_results[0]['pose_yaw']
        pose_pitch = recognize_results[0]['pose_pitch']
        pose_roll = recognize_results[0]['pose_roll']

        return (pose_yaw, pose_pitch, pose_roll)
    return

def register_user(name, img_path):
    global faceMe_sdk
    ret, image = faceMe_sdk.convert_to_faceMe_image(img_path)
    if FR_FAILED(ret):
        print("Add Face failed")
        return

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.register_user(name, image, register_config)
    if FR_FAILED(ret):
        print('Add Face failed')
        return

    print("Add face succ, faceId :", face_id)

def register_user_with_faceimage(name, faceme_image):
    global faceMe_sdk

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.register_user(name, faceme_image, register_config)
    if FR_FAILED(ret):
        print('Add Face failed')
        return

    print("Add face succ, faceId :", face_id)

def add_face(name, img_path):
    global faceMe_sdk
    ret, image = faceMe_sdk.convert_to_faceMe_image(img_path)
    if FR_FAILED(ret):
        print("Add Face failed")
        return

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.add_face(name, image, register_config)
    if FR_FAILED(ret):
        print('Add Face failed')
        return

    print("Add face succ, faceId :", face_id)

def add_face_faceimage(name, faceme_image):
    global faceMe_sdk

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.add_face(name, faceme_image, register_config)
    if FR_FAILED(ret):
        print('Add Face failed')
        return

    print("Add face succ, faceId :", face_id)

def unregister_user(name):
    global faceMe_sdk

    ret = faceMe_sdk.unregister_user(name)
    if FR_FAILED(ret):
        print("Unregister user failed, reutrn:", ret)
        return

    print("Unregister user succ")

def remove_face(face_id):
    global faceMe_sdk
    ret = faceMe_sdk.remove_face(face_id)
    if FR_FAILED(ret):
        print("Remove face failed, return:", ret)

    print("Remove face succ")


def compare(faceme_image, name):
    global faceMe_sdk

    images = [faceme_image]
    ret, recognizeResults = faceMe_sdk.recognize_faces(images)
    if FR_FAILED(ret):
        print("Recognize face failed, return: ", ret)
        return
    if len(recognizeResults) != 1:
        print("No face or too many face for searching")
        return

    search_config = {'maxNumOfCandidates': 1, 'collectionName': name, 'far': "1E-5"}
    ret, similar_faces = faceMe_sdk.search_similar_faces(recognizeResults[0]['faceFeatureStruct'], search_config)
    if ret == FaceMe.FR_RETURN_NOT_FOUND:
        print("Similar face not found")
        return
    if FR_FAILED(ret): 
        print("Search similar faces failed, return: ", ret)
        return
    print(similar_faces)

def compare_images(faceme_image_1, faceme_image_2):
    global faceMe_sdk

    ret, recognize_results1 = faceMe_sdk.recognize_faces([faceme_image_1])
    if FR_FAILED(ret):
        print("Recognize image1 failed, return: ", ret)
        return
    if len(recognize_results1) == 0:
        print("No face found in image1")
        return 
    ret, recognize_results2 = faceMe_sdk.recognize_faces([faceme_image_2])
    if FR_FAILED(ret):
        print("Recognize image2 failed, return: ", ret)
        return
    if len(recognize_results2) == 0:
        print("No face found in image2")
        return 

    ret, compare_result = faceMe_sdk.compare_face_feature(recognize_results1[0]['faceFeatureStruct'], recognize_results2[0]['faceFeatureStruct'])
    if FR_FAILED(ret):
        print("Compare face feature failed, return: ", ret)
        return

    return compare_result

def search_similar_face(faceme_img):

    global faceMe_sdk

    images = [faceme_img]
    ret, recognize_results = faceMe_sdk.recognize_faces(images)
    if FR_FAILED(ret):
        print("Recognize face failed, return: ", ret)
        return
    if len(recognize_results) != 1:
        print("No face or too many face for searching")
        return

    search_config = {'maxNumOfCandidates': 1, 'far': "1E-5"}
    ret, similar_faces = faceMe_sdk.search_similar_faces(recognize_results[0]['faceFeatureStruct'], search_config)
    if ret == FaceMe.FR_RETURN_NOT_FOUND or len(similar_faces) == 0:
        print("Similar face not found")
        return
    if FR_FAILED(ret):
        print("Search similar faces failed, return: ", ret)
        return
    print(similar_faces)
    return similar_faces

def get_face_feature(faceId):
    global faceMe_sdk
    ret, feature = faceMe_sdk.get_face_feature(faceId)
    if ret == FaceMe.FR_RETURN_NOT_FOUND:
        print("Face feature not found")
        return
    if FR_FAILED(ret):
        print("Get face feature failed, ret: ", ret)
        return
    print("Get face feature succ.")
    return feature

def get_face_thumbnail(faceId):
    if faceId == -1:
        return 
    global faceMe_sdk
    ret, face_image = faceMe_sdk.get_face_thumbnail(faceId)
    if FR_FAILED(ret):
        print("Get face thumbnail failed, return: ", ret)
        return
    return face_image

def list_users():
    global faceMe_sdk
    options={}
    # options['userNames'] = ['Albert Chang', 'Carol']
    # options['userIds'] = [1,5]
    ret, result = faceMe_sdk.list_users(options)
    if FR_FAILED(ret):
        print("List user failed, return:", ret)
        return

    print(result)

def save_facemeimage(faceMe_image, save_path):
    height = faceMe_image.height
    width = faceMe_image.width
    channel = faceMe_image.channel
    data = faceMe_image.data
    stride = faceMe_image.stride
    pixelFormat = faceMe_image.pixelFormat
    image = np.asarray(data)
    image.shape = (height, width, channel)
    write_status = cv2.imwrite(save_path, image)
    if not write_status:
        return ""
    return save_path

def SaveFaceMeImageToPath(faceMe_image, save_path):
    height = faceMe_image.height
    width = faceMe_image.width
    channel = faceMe_image.channel
    data = faceMe_image.data
    stride = faceMe_image.stride
    pixelFormat = faceMe_image.pixelFormat
    #image = np.asarray(data, dtype=np.uint8)
    #image.shape = (height, width, channel)
    array = (ctypes.c_ubyte * height * width * channel).from_address(ctypes.addressof(data.contents))
    image = np.ndarray(buffer=array, dtype=np.uint8, shape = (height, width, channel))
    write_status = cv2.imwrite(save_path, image)
    if not write_status:
        return ""
    return save_path


import os
import platform
import sys
import time
import queue
import threading
import cv2
import array
import numpy as np
import FaceMe.FaceMePython3SDK as FaceMe
from FaceMe.FaceMeSDK import FaceMeSDK, FR_FAILED, FR_SUCC
if platform.system() == "Windows":
    import msvcrt
else:
    import tty
    import sys
    import termios
    from fcntl import ioctl


class KeyBoardPress():
    def __init__(self):
        if platform.system() != "Windows":
            self.__orig_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)

    def kbhit(self):
        if platform.system() == "Windows":
            return msvcrt.kbhit()
        else:
            buf = array.array('i', [0])
            ioctl(sys.stdin.fileno(), termios.FIONREAD, buf)
            return bool(buf[0])

    def getch(self):
        if platform.system() == "Windows":
            ch = msvcrt.getch()
            return ch
        else:
            ch = sys.stdin.buffer.read(1)
            return ch

    def __del__(self):
        if platform.system() != "Windows":
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__orig_settings)


def ToQualityIssueString(issue, occlusion_reason, tab=""):
    issue_string = ""
    issue_string += tab
    issue_string += "Size (IOD > 36 pixel):"
    issue_string += "NG\n" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_UNDERSIZED else "Quality OK\n"
    issue_string += tab
    issue_string += "Angle (<20 degrees):"
    issue_string += "NG\n" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_WRONG_ANGLE else "Quality OK\n"
    issue_string += tab
    issue_string += "Occlusion:"
    issue_string += "NG\n" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_OCCLUDED else "Quality OK\n"
    issue_string += tab
    issue_string += " -Suitable for occlusion detection:"
    issue_string += "NG\n" if occlusion_reason & FaceMe.FR_OCCLUSION_UNKNOWN else "Quality OK\n"
    issue_string += tab
    issue_string += " -Left eye occlusion detection:"
    issue_string += "NG\n" if occlusion_reason & FaceMe.FR_OCCLUSION_LEFTEYE else "Quality OK\n"
    issue_string += tab
    issue_string += " -Right eye occlusion detection:"
    issue_string += "NG\n" if occlusion_reason & FaceMe.FR_OCCLUSION_RIGHTEYE else "Quality OK\n"
    issue_string += tab
    issue_string += " -Nose occlusion detection:"
    issue_string += "NG\n" if occlusion_reason & FaceMe.FR_OCCLUSION_NOSE else "Quality OK\n"
    issue_string += tab
    issue_string += " -Mouth eye occlusion detection:"
    issue_string += "NG\n" if occlusion_reason & FaceMe.FR_OCCLUSION_MOUTH else "Quality OK\n"
    issue_string += tab

    issue_string += "Camera Focus:"
    issue_string += "NG\n" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_BLUR else "Quality OK\n"
    issue_string += tab
    issue_string += "Lighting:"
    issue_string += "NG\n" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_EXPOSURE else "Quality OK\n"
    issue_string += tab
    issue_string += "Color:"
    issue_string += "NG" if issue & FaceMe.FR_QUALITY_ISSUE_OPTION_GRAYSCALE else "Quality OK"
    return issue_string


def SaveFaceMeImage(faceMe_image, image_name):
    if platform.system() == "Windows":
        save_path = os.getenv('PROGRAMDATA')
    elif platform.system() == "Linux":
        save_path = os.path.join(os.getenv('HOME'), ".local", "share")
    save_path = os.path.join(save_path, "CyberLink", "FaceMeSDK", "python_dump")
    if not os.path.isdir(save_path):
        try:
            os.makedirs(save_path)
        except OSError:
            print(f"Creation of the directory {save_path} failed")
            return ""
    save_path = os.path.join(save_path, image_name)
    height = faceMe_image['height']
    width = faceMe_image['width']
    channel = faceMe_image['channel']
    data = faceMe_image['data']
    stride = faceMe_image['stride']
    pixelFormat = faceMe_image['pixelFormat']
    image = np.zeros([height, width, channel], np.uint8)
    for row in range(0, height):
        for col in range(0, width):
            if pixelFormat == FaceMe.FR_PIXEL_FORMAT_BGR:
                index = row*stride + col*3
                image[row, col] = (data[index], data[index+1], data[index+2])
            else:
                pass
    write_status = cv2.imwrite(save_path, image)
    if not write_status:
        return ""
    return save_path


class CameraRecognizerCallbackHandler(FaceMe.ICameraRecognizerHandler, threading.Thread):
    def __init__(self):
        FaceMe.ICameraRecognizerHandler.__init__(self)
        threading.Thread.__init__(self)
        self.__needImageQualityCheck = False
        self.__advExtractImageOnly = False
        self.__needExtractFaceImages = False
        self.__needExtractFrameImage = False
        self.__isAdvExtractionEnabled = False
        self.__adv_extraction_interval = 500
        self.__advance_extraction_times = {}

        self.__exit = False
        self.__recognizeQueue = queue.Queue(maxsize=20)
        self.__faceMe_sdk = None

    def Initialize(self, faceMeSDK, advExtractImageOnly, needImageQualityCheck, needExtractFaceImages, needExtractFrameImage, isAdvExtractionEnabled):
        self.__faceMe_sdk = faceMeSDK
        self.__advExtractImageOnly = advExtractImageOnly
        self.__needImageQualityCheck = needImageQualityCheck
        self.__needExtractFaceImages = needExtractFaceImages
        self.__needExtractFrameImage = needExtractFrameImage
        self.__isAdvExtractionEnabled = isAdvExtractionEnabled

    def SetAdvExtractionInterval(self, interval):
        self.__adv_extraction_interval = interval

    def Stop(self):
        self.__exit = True

    def OnFaceDetectCompleted(self, cameraId, frameInfo, faceDetectResult, nextActions):
        nextActions.extractOptions = FaceMe.FR_FEATURE_OPTION_ALL if faceDetectResult.faceCount > 0 else FaceMe.FR_FEATURE_OPTION_NONE
        nextActions.advancedExtractOptions = FaceMe.FR_FEATURE_OPTION_NONE
        nextActions.needExtractFaceImages = False
        nextActions.needExtractImage = False

    def OnFaceDetectError(self, errorCode):
        print("Face Detect error, errorCode: ", errorCode)

    def OnFaceExtractCompleted(self, cameraId, frameInfo, faceExtractResult, nextActions):
        if faceExtractResult.isAdvancedExtraction:
            # put result into queue for another thread to do search similar faces
            if faceExtractResult.faceCount > 0:
                recognizeInfo = {}
                recognizeInfo['cameraId'] = cameraId

                # Maintain the callback result life cycle by yourself
                recognizeInfo['frameInfo'] = self.DeepCopyVideoFrameInfo(frameInfo)
                recognizeInfo['faceExtractResult'] = self.DeepCopyFaceExtractResult(faceExtractResult)
                try:
                    self.__recognizeQueue.put_nowait(recognizeInfo)
                except queue.Full:
                    pass

            # decide next action
            nextActions.advancedExtractOptions = FaceMe.FR_FEATURE_OPTION_NONE
            if self.__needImageQualityCheck or not self.__advExtractImageOnly:
                nextActions.needQualiyCheck = True
                nextActions.needExtractFaceImages = False
                nextActions.needExtractImage = False

            if self.__advExtractImageOnly and not self.__needImageQualityCheck:
                nextActions.needQualiyCheck = False
                if self.__needExtractFaceImages:
                    for i in range(0, faceExtractResult.faceInfos.size()):
                        nextActions.extractFaceImageTypes[i] = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
                        nextActions.extractFaceImageRects[i] = faceExtractResult.faceInfos[i].boundingBox
                        nextActions.extractFaceImageScalingFactors[i] = 1.0

                nextActions.extractImageType = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
                nextActions.extractImageScalingFactor = 1.0
                nextActions.needExtractFaceImages = self.__needExtractFaceImages
                nextActions.needExtractImage = self.__needExtractFrameImage
            else:
                nextActions.needQualiyCheck = self.__needImageQualityCheck
                nextActions.needExtractFaceImages = False
                nextActions.needExtractImage = False
        else:
            # put result into queue for another thread to do search similar faces
            if faceExtractResult.faceCount > 0:
                recognizeInfo = {}
                recognizeInfo['cameraId'] = cameraId

                # Maintain the callback result life cycle by yourself
                recognizeInfo['frameInfo'] = self.DeepCopyVideoFrameInfo(frameInfo)
                recognizeInfo['faceExtractResult'] = self.DeepCopyFaceExtractResult(faceExtractResult)
                try:
                    self.__recognizeQueue.put_nowait(recognizeInfo)
                except queue.Full:
                    pass

            # Decide if it want next advanceExtractionTime  (need fix)
            if self.__isAdvExtractionEnabled:
                last_time_adv_extraction_time = self.__advance_extraction_times.get(str(cameraId), 0)
                if frameInfo.timePosition > (last_time_adv_extraction_time + self.__adv_extraction_interval):
                    print("do adv extraction")
                    self.__advance_extraction_times[str(cameraId)] = frameInfo.timePosition
                    nextActions.advancedExtractOptions = FaceMe.FR_FEATURE_OPTION_ALL
                else:
                    nextActions.advancedExtractOptions = FaceMe.FR_FEATURE_OPTION_NONE
            else:
                nextActions.advancedExtractOptions = FaceMe.FR_FEATURE_OPTION_NONE

            # Decide next action
            if self.__isAdvExtractionEnabled and self.__advExtractImageOnly:
                nextActions.needQualiyCheck = False
                nextActions.needExtractFaceImages = False
                nextActions.needExtractImage = False
            else:
                if self.__needImageQualityCheck:
                    nextActions.needQualiyCheck = True
                    nextActions.needExtractFaceImages = False
                    nextActions.needExtractImage = False
                else:
                    nextActions.needQualiyCheck = False
                    if self.__needExtractFaceImages:
                        for i in range(0, faceExtractResult.faceInfos.size()):
                            nextActions.extractFaceImageTypes[i] = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
                            nextActions.extractFaceImageRects[i] = faceExtractResult.faceInfos[i].boundingBox
                            nextActions.extractFaceImageScalingFactors[i] = 1.0
                    nextActions.extractImageType = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
                    nextActions.extractImageScalingFactor = 1.0
                    nextActions.needExtractFaceImages = self.__needExtractFaceImages
                    nextActions.needExtractImage = self.__needExtractFrameImage

    def OnFaceExtractError(self, errorCode):
        print("Face Extract error, errorCode: ", errorCode)

    def OnFaceImagesExtractCompleted(self, cameraId, frameInfo, faceImageResult):
        # print("Face Images Extract complete")
        pass

    def OnFaceImagesExtractError(self, cameraId, frameInfo, errorCode):
        print("Face Images Extract error, errorCode: ", errorCode)

    def OnImageExtractCompleted(self, cameraId, frameInfo, faceMeImage):
        # print("Image Extract complete")
        pass

    def OnImageExtractError(self, cameraId, frameInfo, errorCode):
        print("Image Extract error, errorCode: ", errorCode)

    def OnFaceQualityCheckCompleted(self, cameraId, frameInfo, qualityCheckResult, nextActions):
        if self.__needExtractFaceImages:
            for i in range(0, qualityCheckResult.faceCount):
                nextActions.extractFaceImageTypes[i] = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
                if qualityCheckResult.results[i].issue == FaceMe.FR_QUALITY_ISSUE_OPTION_NONE:
                    nextActions.extractFaceImageRects[i] = qualityCheckResult.faceInfos[i].boundingBox
                nextActions.extractFaceImageScalingFactors[i] = 1.0

        nextActions.extractImageType = FaceMe.FR_EXTRACT_IMAGE_TYPE_HOST_MEM
        nextActions.extractImageScalingFactor = 1.0
        nextActions.needExtractFaceImages = self.__needExtractFaceImages
        nextActions.needExtractImage = self.__needExtractFrameImage

    def OnFaceQualityCheckError(self, cameraId, frameInfo, errorCode):
        print("Quality Check error, errorCode: ", errorCode)

    def run(self):
        while(True):
            if self.__exit or self.__faceMe_sdk is None:
                break
            try:
                recognizeInfo = self.__recognizeQueue.get(timeout=0.033)
            except queue.Empty:
                continue

            search_config = {'maxNumOfCandidates': 1}
            visitor_count = 0
            name = []
            for faceFeature in recognizeInfo['faceExtractResult'].faceFeatures:
                ret, similarFaces = self.__faceMe_sdk.search_similar_faces(faceFeature, search_config)
                if FR_FAILED(ret):
                    if ret == FaceMe.FR_RETURN_NOT_FOUND:
                        visitor_count += 1
                    else:
                        print("featureType: ", faceFeature.featureType)
                        print("featureSubType: ", faceFeature.featureSubType)
                        print("search_similar_faces failed, return: ", ret)
                else:
                    if len(similarFaces) >= 1:
                        name.append(similarFaces[0]['name'])
            if visitor_count == 1:
                name.append("visitor")
            elif visitor_count > 1:
                name.append("visitors")
            print("cameraId: ", recognizeInfo['cameraId']
                , ", frameIndex: ", recognizeInfo['frameInfo'].frameIndex
                , ", name: ", name
                , ", advance extract: ", recognizeInfo['faceExtractResult'].isAdvancedExtraction
                , ", face count: ", recognizeInfo['faceExtractResult'].faceCount)
            time.sleep(0.01)

    def DeepCopyFaceExtractResult(self, faceExtractResult):
        faceExtractCopy = FaceMe.FaceExtractResult()
        faceExtractCopy.imageWidth = faceExtractResult.imageWidth
        faceExtractCopy.imageHeight = faceExtractResult.imageHeight
        faceExtractCopy.faceCount = faceExtractResult.faceCount
        faceExtractCopy.isAdvancedExtraction = faceExtractResult.isAdvancedExtraction
        faceExtractResult.faceInfos.swap(faceExtractCopy.faceInfos)
        faceExtractResult.faceLandmarks.swap(faceExtractCopy.faceLandmarks)
        faceExtractResult.faceFeatures.swap(faceExtractCopy.faceFeatures)
        faceExtractResult.faceAttributes.swap(faceExtractCopy.faceAttributes)
        return faceExtractCopy

    def DeepCopyVideoFrameInfo(self, videoFrameInfo):
        videoFrameInfoCopy = FaceMe.VideoFrameInfo()
        videoFrameInfoCopy.frameIndex = videoFrameInfo.frameIndex
        videoFrameInfoCopy.timePosition = videoFrameInfo.timePosition
        videoFrameInfoCopy.timeDuration = videoFrameInfo.timeDuration
        return videoFrameInfoCopy

class VideoStreamHandler(FaceMe.IVideoStreamHandler):
    def __init__(self):
        FaceMe.IVideoStreamHandler.__init__(self)
        self.__camera_id = 0
        self.__exit = False

    def IsExit(self):
        return self.__exit

    def OnEndOfStream(self):
        print("camera id(", self.__camera_id, ") get end of stream")
        self.__exit = True

    def OnError(self, result):
        print("camera id(", self.__camera_id, ") get error, failed:", result)
        self.__exit = True

    def SetCameraId(self, cameraId):
        self.__camera_id = cameraId


class AntiSpoofingHandler(FaceMe.I2DAntiSpoofingHandler):
    def __init__(self):
        FaceMe.I2DAntiSpoofingHandler.__init__(self)
        self.__faceMe_sdk = None

    def SetFaceMeSDK(self, faceMe_sdk):
        self.__faceMe_sdk = faceMe_sdk

    def OnEndOfStream(self):
        print("2D antispoofing on end of stream")

    def OnError(self, errorCode):
        print("2D antispoofing On error: " << errorCode)

    def OnSpoofingResult(self, antispoofingResult):
        result_dict = {}
        result_dict['data'] = antispoofingResult.faceMeImage.data
        result_dict['width'] = antispoofingResult.faceMeImage.width
        result_dict['stride'] = antispoofingResult.faceMeImage.stride
        result_dict['height'] = antispoofingResult.faceMeImage.height
        result_dict['pixelFormat'] = antispoofingResult.faceMeImage.pixelFormat
        result_dict['channel'] = antispoofingResult.faceMeImage.channel
        result_dict['faceMeImage'] = antispoofingResult.faceMeImage
        result_dict['spoofingResults'] = antispoofingResult.spoofingResults
        for spoofing_result_info in result_dict['spoofingResults']:
            if spoofing_result_info.progress == 100:
                name = self.search_similar_face(spoofing_result_info.faceFeature)
                spoofing_answer = "unknwon"
                if spoofing_result_info.spoofingResult == FaceMe.FR_ANTISPOOFING_RESULT_SPOOFING:
                    spoofing_answer = "spoofing"
                elif spoofing_result_info.spoofingResult == FaceMe.FR_ANTISPOOFING_RESULT_LIVENESS:
                    spoofing_answer = "liveness"
                print("spoofing result: ", spoofing_answer, ", name: ", name)

    def search_similar_face(self, face_feature):
        search_config = {'maxNumOfCandidates':1}
        name = ""
        ret, similar_faces = self.__faceMe_sdk.search_similar_faces(face_feature, search_config)
        if FR_FAILED(ret):
            if ret == FaceMe.FR_RETURN_NOT_FOUND:
                name = "visitor"
            else:
                print("search_similar_faces failed, return: ", ret)
                name = "visitor"
        else:
            if similar_faces:
                name = similar_faces[0]['name']
        return name

class ParseArg():
    command = ""
    chipset = "NV"
    far = "1E-5"
    fdm = "DEFAULT"
    frm = "DEFAULT"

    image_path = ""
    image_path2 = ""
    license_key = ""
    user_id = -1
    user_name = ""
    face_id = -1
    face_attribute_option = FaceMe.FR_FEATURE_OPTION_ALL
    candidate_count = 1
    min_face_width_ratio = 0.05

    device_id = -1
    device_path = ""
    width = 1280
    height = 720
    fps = 0
    detect_thread = 0
    extract_thread = 0
    detect_batchsize = 0   # GPU: 4, 8, 16, VPU: VPU count, CPU: 1
    extract_batchsize = 0   # GPU: 4, 8, 16, VPU: VPU count, CPU: 1
    searchPreference = 2   # 0 : faste search, 2 : Disable fast search

    advfrm = "DEFAULT"
    adv_etract_image_only = True
    need_image_quality_check = False
    need_extract_face_images = False
    need_extract_frame_image = False
    is_adv_extraction_enabled = False
    adv_extraction_interval = 500
    device_paths = []
    device_ids = []

    precision_mode = "STANDARD"

FRM_model_list = ["H1", "H2", "H3", "H5", "H6", "VH", "VH_M", "VH5", "VH5_M", "VH6", "VH6_M", \
     "UH", "UH3", "UH3_M", "UH5", "UH5_M", "UH6", "UH6_M", "DEFAULT"]
FDM_model_list = ["DNN", "DEFAULT"]
FAR_list = ["1E-6", "1E-5", "1E-4", "1E-3", "1E-2"]
Chipset_list = ["CPU", "NV", "MOVIDIUS", "DEFAULT"]

def parse_command_line(args, command_args):
    arglen = len(args)
    if arglen > 0:
        for i in range(len(args)):
            if i == 0:
                command_args.command = args[i].lower()
            else:
                if args[i].upper() == "-LICENSEKEY":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        command_args.license_key = args[i+1]
                        i += 1
                if args[i].upper() == "-FRM":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        FRM = args[i+1].upper()
                        if FRM in FRM_model_list:
                            command_args.frm = FRM
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-ADVFRM":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        adv_FRM = args[i+1].upper()
                        if adv_FRM in FRM_model_list:
                            command_args.advfrm = adv_FRM
                            if command_args.advfrm != "DEFAULT":
                                command_args.is_adv_extraction_enabled = True
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-FDM":
                    if i + 1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        FDM = args[i+1].upper()
                        if FDM in FDM_model_list:
                            command_args.fdm = FDM
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-MINFACE":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.min_face_width_ratio = float(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-FAR":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        FAR = args[i+1].upper()
                        if FAR in FAR_list:
                            command_args.far = FAR
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-CHIPSET":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, arglen[i])
                    else:
                        chipset = args[i+1].upper()
                        if chipset in Chipset_list:
                            command_args.chipset = chipset
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-FACEATTROPTION":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        face_attribute_option = args[i+1]
                        i += 1
                        extract_option = 0
                        if face_attribute_option == "":
                            extract_option = FaceMe.FR_FEATURE_OPTION_ALL
                        else:
                            options = face_attribute_option.lower()
                            for opt in options:
                                extract_option |= FaceMe.FR_FEATURE_OPTION_EMOTION if opt == 'e'\
                                    else FaceMe.FR_FEATURE_OPTION_GENDER if opt == 'g'\
                                    else FaceMe.FR_FEATURE_OPTION_AGE if opt == 'a'\
                                    else FaceMe.FR_FEATURE_OPTION_BOUNDING_BOX if opt == 'b'\
                                    else FaceMe.FR_FEATURE_OPTION_FEATURE_LANDMARK if opt == 'l'\
                                    else FaceMe.FR_FEATURE_OPTION_POSE if opt == 'p'\
                                    else FaceMe.FR_FEATURE_OPTION_OCCLUSION if opt == 'o'\
                                    else FaceMe.FR_FEATURE_OPTION_MASKED_FEATURE if opt == 'm' \
                                    else FaceMe.FR_FEATURE_OPTION_FULL_FEATURE if opt == 'f'\
                                    else FaceMe.FR_FEATURE_OPTION_NONE
                        command_args.face_attribute_option = extract_option
                if args[i].upper() == "-PHOTO":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        if command_args.image_path != "":
                            command_args.image_path2 = args[i+1]
                            i += 1
                        else:
                            command_args.image_path = args[i+1]
                            i += 1
                if args[i].upper() == "-ID":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.user_id = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-NAME":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        command_args.user_name = args[i+1]
                        i += 1
                if args[i].upper() == "-FACEID":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.face_id = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-CANDIDATECOUNT":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.candidate_count = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-DEVICEID":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.device_id = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-DEVICEPATH":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        command_args.device_path = args[i+1]
                        i += 1
                if args[i].upper() == "-WIDTH":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.width = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-HEIGHT":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.height = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-FPS":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.fps = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-DETECT_THREAD":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.detect_thread = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-EXTRACT_THREAD":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.extract_thread = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-DETECT_BATCHSIZE":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.detect_batchsize = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-EXTRACT_BATCHSIZE":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.extract_batchsize = int(args[i+1])
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                if args[i].upper() == "-ADVEXTRACTIMAGEONLY":
                    command_args.adv_etract_image_only = True
                if args[i].upper() == "-NEEDIMAGEQUALITYCHECK":
                    command_args.need_image_quality_check = True
                if args[i].upper() == "-NEEDEXTRACTFACEIMAGES":
                    command_args.need_extract_face_images = True
                if args[i].upper() == "-NEEDEXTRACTFRAMEIMAGE":
                    command_args.need_extract_frame_image = True
                if args[i].upper() == "-ADV_INTERVAL":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        command_args.adv_extraction_interval = args[i+1]
                        i += 1
                if args[i].upper() == "-DEVICEPATHS":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        command_args.device_paths.append(args[i+1])
                        i += 1
                        while i+1 < arglen:
                            if args[i+1][0] != "-":
                                command_args.device_paths.append(args[i+1])
                                i += 1
                            else:
                                break
                if args[i].upper() == "-DEVICEIDS":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        try:
                            command_args.device_ids.append(int(args[i+1]))
                        except ValueError:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1
                        parameter = "-DEVICEIDS"
                        while i+1 < arglen:
                            if args[i+1][0] != "-":
                                try:
                                    command_args.device_ids.append(int(args[i+1]))
                                except ValueError:
                                    return show_help_and_exit(command_args.command, parameter, args[i])
                                i += 1
                            else:
                                break
                if args[i].upper() == "-PRECISIONMODE":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, args[i])
                    else:
                        if i+1 >= arglen:
                            show_help_and_exit(command_args.command, args[i])
                        else:
                            precision_mode = args[i+1].upper()
                            if precision_mode == "HIGH" or precision_mode == "FAST" or precision_mode == "STANDARD":
                                command_args.precision_mode = precision_mode
                            else:
                                return show_help_and_exit(command_args.command, args[i], args[i+1])
                            i += 1
                if args[i].upper() == "-FAST_SEARCH":
                    if i+1 >= arglen:
                        return show_help_and_exit(command_args.command, arglen[i])
                    else:
                        strsearchpreference = args[i+1].upper()
                        if strsearchpreference == "OFF":
                            command_args.searchPreference = 2
                        elif strsearchpreference == "ON":
                            command_args.searchPreference = 0
                        else:
                            return show_help_and_exit(command_args.command, args[i], args[i+1])
                        i += 1

    if command_args.command.lower() == "help":
        return show_help_and_exit("", "", "", False, True)
    return FaceMe.FR_RETURN_OK


def show_help_and_exit(command, parameter, value="", show_error=True, show_all=False):
    python_version = str(sys.version_info[0]) + "." + str(sys.version_info[1])
    if platform.system() == "Windows":
        pycommand = "py" +" -" + python_version + " FaceMePythonSample.py"
    else:
        pycommand = "python" + python_version + " FaceMePythonSample.py"
    if show_error:
        print(f"Parser parameter error.\nKey: {parameter}, Value: {value}")
    print("Usage:")
    if show_all:
        print("============ License ============")

    if command.lower() == "activate" or show_all:
        print("- Activate FaceMe:")
        print(f"\t{pycommand} activate -licensekey licensekey")

    if command.lower() == "deactivate" or show_all:
        print("- Deactivate FaceMe:")
        print(f"\t{pycommand} deactivate")

    if command.lower() == "renewlicense" or show_all:
        print("- Refresh FaceMe license")
        print(f"\t{pycommand} renewlicense")

    if show_all:
        print("\n======== Face Recognition =======")

    if command.lower() == "recognizeface" or show_all:
        print("- Extract face information from image")
        print(f"\t{pycommand} recognizeface -photo imagePath [-faceAttrOption Options]")

    if command.lower() == "detectface" or show_all:
        print("- detect face information from image")
        print(f"\t{pycommand} detectface -photo imagePath [-faceAttrOption Options]")

    if show_all:
        print("\n====== Database Management ======")

    if command.lower() == "registeruser" or show_all:
        print("- Register user into database")
        print(f"\t{pycommand} registeruser -photo imagePath -name userName")

    if command.lower() == "unregisteruser" or show_all:
        print("- Unregister user from database")
        print(f"\t{pycommand} unregisteruser -name userName")

    if command.lower() == "addface" or show_all:
        print("- Add face to user")
        print(f"\t{pycommand} addface -photo imagePath -name userName")

    if command.lower() == "removeface" or show_all:
        print("- Remove user from database")
        print(f"\t{pycommand} removeface -faceid facdId")

    if command.lower() == "listusers" or show_all:
        print("- List all users in database")
        print(f"\t{pycommand} listusers [-name userName] [-id userId]")

    if command.lower() == "getfacefeature" or show_all:
        print("- Get face feature from specific face id")
        print(f"\t{pycommand} getfacefeature -faceId faceId")

    if command.lower() == "getfacethumbnail" or show_all:
        print("- Get face thumbnail from specific face id ")
        print(f"\t{pycommand} getfacethumbnail -faceId faceId")

    if show_all:
        print("\n====== Face Compare/Search ======")

    if command.lower() == "compare" or show_all:
        print("- Compare face in image to specific user in database")
        print(f"\t{pycommand} compare -photo imagePath -name userName [-far far]")

    if command.lower() == "compareimages" or show_all:
        print("- Compare faces from images")
        print(f"\t{pycommand} compareimages -photo imagePath -photo imagePath")

    if command.lower() == "searchsimilarface" or show_all:
        print("- Search user in database")
        print(f"\t{pycommand} searchsimilarface -photo imagePath [-far far] [-candidateCount count]")

    if show_all:
        print("\n============= Misc. =============")

    if command.lower() == "detectimagequality" or show_all:
        print("- Analyze quality from image file")
        print(f"\t{pycommand} detectimagequality -photo imagePath")

    if show_all:
        print("\n======== Video Recognize =========")

    if command.lower() == "listcameras" or show_all:
        print("- list all available webcam")
        print(f"\t{pycommand} listcameras")

    if command.lower() == "recognizevideos" or show_all:
        print("- recognize face in webcam/rtsp")
        print(f"\t{pycommand} recognizevideos -deviceids webcamId1 webcamId2 ... -devicepaths rtspURL1 rtspURL2 ...")

    if show_all:
        print("\n======== 2D AntiSpoofing =========")

    if command.lower() == "list2dantispoofingcameras" or show_all:
        print("- list all available webcam for 2D antispoofing")
        print(f"\t{pycommand} list2dantispoofingcameras")

    if command.lower() == "antispoofing2d" or show_all:
        print("- 2D antispoofing without screen with webcam or rtsp")
        print(f"\t{pycommand} antispoofing2d [-deviceid deviceId] [-devicePath path]")

    return FaceMe.FR_RETURN_INVALID_ARGUMENT


def wrong_command():
    result = 0
    show_help_and_exit("", "", "", False, True)


def get_license_file_path():
    if platform.system() == "Windows":
        app_data_path = os.getenv('PROGRAMDATA')
    elif platform.system() == "Linux":
        app_data_path = os.path.join(os.getenv('HOME'), ".local", "share")
    license_file_path = os.path.join(app_data_path, 'CyberLink', 'FaceMeSDK', 'DemoTool', 'License.key')
    return license_file_path


def read_license_key_cache():
    license_file_path = get_license_file_path()
    try:
        license_file = open(license_file_path, 'r')
    except OSError:
        return ""
    license_key = license_file.read()
    return license_key


def remove_license_key_cache():
    license_file_path = get_license_file_path()
    os.remove(license_file_path)


def save_license_key_cache(license_key):
    license_file_path = get_license_file_path()
    license_file = open(license_file_path, 'w+')
    license_file.write(license_key)


def initialize_SDK(command_args):
    global faceMe_sdk
    faceMe_sdk = FaceMeSDK()
    options = {}
    if platform.system() == "Windows":
        app_bundle_path = os.path.dirname(os.path.realpath(__file__))
        app_cache_path = os.getenv('LOCALAPPDATA')
        app_data_path = os.getenv('PROGRAMDATA')
    elif platform.system() == "Linux":
        app_bundle_path = os.path.dirname(os.path.realpath(__file__))
        app_cache_path = os.path.join(os.getenv('HOME'), ".cache")
        app_data_path = os.path.join(os.getenv('HOME'), ".local", "share")
    else:
        print("FaceMe Python SDK only support in Windows and Linux")
        return FaceMe.FR_RETURN_FAIL

    license_key_cache = read_license_key_cache()
    if command_args.command == "activate":
        if command_args.license_key == "":
            return show_help_and_exit(command_args.command, "-licensekey")
        if license_key_cache != "" and command_args.license_key != license_key_cache:
            print("Need to deacitvate license first before acitvate the new license")
            return FaceMe.FR_RETURN_FAIL
        license_key = command_args.license_key
    else:
        license_key = license_key_cache
        if license_key == "":
            print("license file doesn't found")
            return FaceMe.FR_RETURN_NOT_FOUND
    options['chipset'] = command_args.chipset
    options['fdm'] = command_args.fdm
    options['frm'] = command_args.frm
    options['minFaceWidthRatio'] = command_args.min_face_width_ratio
    options['advfrm'] = command_args.advfrm
    options['maxDetectionThreads'] = command_args.detect_thread
    options['maxExtractionThreads'] = command_args.extract_thread
    options['maxAdvancedExtractionThreads'] = command_args.extract_thread
    options['preferredDetectionBatchSize'] = command_args.detect_batchsize
    options['preferredExtractionBatchSize'] = command_args.extract_batchsize
    options['searchPreference'] = command_args.searchPreference

    if command_args.command == "list2dantispoofingcameras" or command_args.command == "antispoofing2d":
        global spoofing_result_handler
        spoofing_result_handler = AntiSpoofingHandler()
        spoofing_result_handler.SetFaceMeSDK(faceMe_sdk)
        options['spoofingResultHandler'] = spoofing_result_handler

    if command_args.command == "listcameras" or command_args.command == "recognizevideos":
        global camera_recognizer_handler
        camera_recognizer_handler = CameraRecognizerCallbackHandler()
        camera_recognizer_handler.Initialize(faceMe_sdk, command_args.adv_etract_image_only \
                , command_args.need_image_quality_check, command_args.need_extract_face_images \
                , command_args.need_extract_frame_image, command_args.is_adv_extraction_enabled)
        camera_recognizer_handler.SetAdvExtractionInterval(command_args.adv_extraction_interval)
        options['cameraRecognizerHandler'] = camera_recognizer_handler
        if command_args.command  == "listcameras":
            options['maxNumberOfCameras'] = 1
        if command_args.command  == "recognizevideos":
            camera_count = len(command_args.device_ids) + len(command_args.device_paths)
            options['maxNumberOfCameras'] = camera_count

    ret = faceMe_sdk.initialize(license_key, app_bundle_path, app_cache_path, app_data_path, options)
    if FR_FAILED(ret):
        print("Register licesne failed, return:", ret)
        return ret
    return ret

def activate_license(command_args):
    global faceMe_sdk
    print("Register license succ")
    save_license_key_cache(command_args.license_key)

def deactivate_license(command_args):
    global faceMe_sdk
    ret = faceMe_sdk.deactivate_license()
    if FR_FAILED(ret):
        print("Deactivate licesne failed, return:", ret)
        return

    print("Deactivate license succ")
    remove_license_key_cache()

def renew_license(command_args):
    global faceMe_sdk
    ret = faceMe_sdk.renew_license()
    if FR_FAILED(ret):
        print("Renew licesne failed, return:", ret)
        return

    print("Renew license succ")

def list_users(command_args):
    global faceMe_sdk
    options={}

    if command_args.user_name != "":
        options['userName'] = command_args.user_name
    if command_args.user_id != -1:
        options['userId'] = command_args.user_id
    # options['userNames'] = ['Albert Chang', 'Carol']
    # options['userIds'] = [1,5]
    ret, result = faceMe_sdk.list_users(options)
    if FR_FAILED(ret):
        print("List user failed, return:", ret)
        return

    print(result)

def register_user(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    if command_args.user_name == "":
        return show_help_and_exit(command_args.command, "-name")
    global faceMe_sdk
    options={}
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.register_user(command_args.user_name, image, register_config)
    if FR_FAILED(ret):
        print("Register user failed")
        return

    print("Register user succ, faceId: ", face_id)

def add_face(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    if command_args.user_name == "":
        return show_help_and_exit(command_args.command, "-name")
    global faceMe_sdk
    options={}
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to Faceme Image failed")
        return

    register_config = {'enableMaskFeatureEnroll':False}
    ret, face_id = faceMe_sdk.add_face(command_args.user_name, image, register_config)
    if FR_FAILED(ret):
        print("Add Face failed")
        return

    print("Add face succ, faceId: ", face_id)

def unregister_user(command_args):
    if command_args.user_name == "":
        return show_help_and_exit(command_args.command, "-name")
    global faceMe_sdk

    ret = faceMe_sdk.unregister_user(command_args.user_name)
    if FR_FAILED(ret):
        print("Unregister user failed, reutrn:", ret)
        return

    print("Unregister user succ")

def remove_face(command_args):
    if command_args.face_id == -1:
        return show_help_and_exit(command_args.command, "-faceid")

    global faceMe_sdk
    ret = faceMe_sdk.remove_face(command_args.face_id)
    if FR_FAILED(ret):
        print("Remove face failed, return:", ret)

    print("Remove face succ")

def compare(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    if command_args.user_name == "":
        return show_help_and_exit(command_args.command, "-name")

    global faceMe_sdk
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    images = [image]
    ret, recognizeResults = faceMe_sdk.recognize_faces(images)
    if FR_FAILED(ret):
        print("Recognize face failed, return: ", ret)
        return
    if len(recognizeResults) != 1:
        print("No face or too many face for searching")
        return

    search_config = {'maxNumOfCandidates': command_args.candidate_count, 'collectionName': command_args.user_name, 'far': command_args.far}
    ret, similar_faces = faceMe_sdk.search_similar_faces(recognizeResults[0]['faceFeatureStruct'], search_config)
    if ret == FaceMe.FR_RETURN_NOT_FOUND:
        print("Similar face not found")
        return
    if FR_FAILED(ret): 
        print("Search similar faces failed, return: ", ret)
        return
    print(similar_faces)

def compare_images(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    if command_args.image_path2 == "":
        return show_help_and_exit(command_args.command, "-photo")

    global faceMe_sdk
    ret, image1 = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Image1 convert to faceme image failed")
        return
    ret, image2 = faceMe_sdk.convert_to_faceMe_image(command_args.image_path2)
    if FR_FAILED(ret):
        print("Image2 convert to faceme image failed")
        return

    ret, recognize_results1 = faceMe_sdk.recognize_faces([image1])
    if FR_FAILED(ret):
        print("Recognize image1 failed, return: ", ret)
        return
    if len(recognize_results1) == 0:
        print("No face found in image1")
        return 
    ret, recognize_results2 = faceMe_sdk.recognize_faces([image2])
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

    print("Compare confidence: ", compare_result['confidence'], ", Same person: ", compare_result['isSamePerson'])

def search_similar_face(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")

    global faceMe_sdk
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    images = [image]
    ret, recognize_results = faceMe_sdk.recognize_faces(images)
    if FR_FAILED(ret):
        print("Recognize face failed, return: ", ret)
        return
    if len(recognize_results) != 1:
        print("No face or too many face for searching")
        return

    search_config = {'maxNumOfCandidates': command_args.candidate_count, 'far': command_args.far}
    ret, similar_faces = faceMe_sdk.search_similar_faces(recognize_results[0]['faceFeatureStruct'], search_config)
    if ret == FaceMe.FR_RETURN_NOT_FOUND or len(similar_faces) == 0:
        print("Similar face not found")
        return
    if FR_FAILED(ret):
        print("Search similar faces failed, return: ", ret)
        return
    print(similar_faces)

def get_face_feature(command_args):
    if command_args.face_id == -1:
        return show_help_and_exit(command_args.command, "-faceId")
    global faceMe_sdk
    ret, feature = faceMe_sdk.get_face_feature(command_args.face_id)
    if ret == FaceMe.FR_RETURN_NOT_FOUND:
        print("Face feature not found")
        return
    if FR_FAILED(ret):
        print("Get face feature failed, ret: ", ret)
        return
    print("Get face feature succ.")
    for key, value in feature.items():
        if key in ['faceFeatureStruct']:
            continue
        print("\t", key, ": ", value)

def get_face_thumbnail(command_args):
    if command_args.face_id == -1:
        return show_help_and_exit(command_args.command, "-faceId")
    global faceMe_sdk
    ret, image = faceMe_sdk.get_face_thumbnail(command_args.face_id)
    if FR_FAILED(ret):
        print("Get face thumbnail failed, return: ", ret)
        return
    image_dict = {}
    image_dict['data'] = image.data
    image_dict['width'] = image.width
    image_dict['stride'] = image.stride
    image_dict['height'] = image.height
    image_dict['pixelFormat'] = image.pixelFormat
    image_dict['channel'] = image.channel
    image_name = str(command_args.face_id) + ".png"
    save_path = SaveFaceMeImage(image_dict, image_name)
    print(f"Get face thumbnail succ, image save in \"{save_path}\"")

def recognize_face(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    global faceMe_sdk
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    images = [image]
    # option 1 : Detect and extract do together
    recognize_config = {'extractOptions':command_args.face_attribute_option}

    ret, recognize_results = faceMe_sdk.recognize_faces(images, recognize_config)
    if FR_FAILED(ret):
        print("Recognize face failed, return: ", ret)
        return
    if len(recognize_results) == 0:
        print("No face found!")
    for i in range(0, len(recognize_results)):
        print("Face ", i, " Recognition Result: ")
        for key, value in recognize_results[i].items():
            if key in ['faceInfoStruct', 'faceLandmarkStruct', 'faceAttributeStruct', 'faceFeatureStruct', 'featureList']:
                continue
            print("\t", key, ": ", value)

    #option 2: Detect and extreact do seperately
    # detect_config = {'detectOptions':FaceMe.FR_FEATURE_OPTION_BOUNDING_BOX | FaceMe.FR_FEATURE_OPTION_FEATURE_LANDMARK | command_args.face_attribute_option }
    # for image in images:
    #     ret, detect_result = faceMe_sdk.detect_face(image, detect_config)
    #     if FR_FAILED(ret):
    #         print("detect_face failed, return: ", ret)
    #         continue
    #     recognize_config = {'extractOptions':command_args.face_attribute_option}
    #     ret, extract_result = faceMe_sdk.extract_face(image, detect_result, recognize_config)
    #     if FR_FAILED(ret):
    #         print("extract_face failed, return: ", ret)
    #         continue
    #     if len(extract_result) == 0:
    #         print("No face found!")
    #     for i in range(0, len(extract_result)):
    #         print("Face ", i, " Recognition Result: ")
    #         for key, value in extract_result[i].items():
    #             if key in ['faceInfoStruct', 'faceLandmarkStruct', 'faceAttributeStruct', 'faceFeatureStruct', 'featureList']:
    #                 continue
    #             print("\t", key, ": ", value)

def detect_face(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    global faceMe_sdk
    options={}
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    detect_config = {'detectOptions':command_args.face_attribute_option}
    ret, detect_results = faceMe_sdk.detect_face(image, detect_config)
    if FR_FAILED(ret):
        print("Detect face failed, return: ", ret)
        return
    if len(detect_results) == 0:
        print("No face found!")
    for i in range(0, len(detect_results)):
        print("Face ", i, " Detection Result: ")
        for key, value in detect_results[i].items():
            if key in ['faceInfoStruct', 'faceLandmarkStruct']:
                continue
            print("\t", key, ": ", value)

def list_cameras(commag_args):
    global faceMe_sdk
    ret, results = faceMe_sdk.list_cameras()
    if FR_FAILED(ret):
        print("List cameras failed, return: ", ret)
        return
    print(results)

def recognize_videos(command_args):
    global faceMe_sdk
    global camera_recognizer_handler
    if not bool(command_args.device_paths) and not bool(command_args.device_ids):
        return show_help_and_exit(command_args.command, "-deviceIds")
    options = {}
    options['preferredWidth'] = command_args.width
    options['preferredHeight'] = command_args.height
    options['preferredFrameRate'] = command_args.fps
    print("paths:", command_args.device_paths)
    print("ids:", command_args.device_ids)
    camera_recognizer_handler.start()
    device_cameraId_map = {}
    handler_cameraId_map = {}
    for device_path in command_args.device_paths:
        handler = VideoStreamHandler()
        ret, camera_id = faceMe_sdk.start_recognize_video(-1, device_path, handler, options)
        if FR_FAILED(ret):
            print("Start recognize video failed, return:", ret, ", device path: ", device_path)
            continue
        print("Start recognize video succ, device path: ", device_path)
        device_cameraId_map[device_path] = camera_id
        handler.SetCameraId(camera_id)
        handler_cameraId_map[str(camera_id)] = handler

    for device_id in command_args.device_ids:
        handler = VideoStreamHandler()
        ret, camera_id = faceMe_sdk.start_recognize_video(device_id, "", handler, options)
        if FR_FAILED(ret):
            print("Start recognize video failed, return:", ret, ", device id: ", device_id)
            continue
        print("Start recognize video succ, device id: ", device_id)
        device_cameraId_map[str(device_id)] = camera_id
        handler.SetCameraId(camera_id)
        handler_cameraId_map[str(camera_id)] = handler
    if bool(device_cameraId_map):
        press = KeyBoardPress()
        while True:
            try: 
                exit_cameras = []
                for camera_id, handler in handler_cameraId_map.items():
                    if handler.IsExit():
                        exit_cameras.append(camera_id)
                for camera_id in exit_cameras:
                    del handler_cameraId_map[camera_id]
                if not bool(handler_cameraId_map):
                    break
                if press.kbhit():
                    if press.getch() == chr(27).encode():
                        break
                time.sleep(0.01)
            except KeyboardInterrupt:
                break

        for device_path in device_cameraId_map:
            ret = faceMe_sdk.stop_recognize_video(device_cameraId_map[device_path])
            if FR_FAILED(ret):
                print("Stop recognize video failed, return:", ret, ", device path: ", device_path)
                continue
    camera_recognizer_handler.Stop()
    camera_recognizer_handler.join()
    return

def list_2d_antispoofing_cameras(command_args):
    global faceMe_sdk
    ret, camera_list = faceMe_sdk.list_2d_antispoofing_cameras()
    if FR_FAILED(ret):
        print("List 2d anti-spoofing cameras failed, return: ", ret)
        return
    print("Camera list: ")
    print(camera_list)

def do_anti_spoofing_2d(command_args):
    if command_args.device_id == -1 and command_args.device_path == "" :
        return show_help_and_exit(command_args.command, "-devicePath")
    global faceMe_sdk
    options = {}
    options['precisionMode'] = command_args.precision_mode

    device_name = ""
    if command_args.device_id != -1:
        ret, camera_list = faceMe_sdk.list_2d_antispoofing_cameras()
        if FR_FAILED(ret):
            print("List 2d anti-spoofing camera failed, return:", ret)
            return
        is_exist = False
        for camera in camera_list:
            if camera['deviceId'] == command_args.device_id:
                is_exist = True
                break
        if not is_exist:
            print("Doesn't find the camera")
            return

    ret = faceMe_sdk.start_2d_antispoofing(command_args.device_id, command_args.device_path, options)
    if FR_FAILED(ret):
        print("Run 2d anti-spoofing failed, return:", ret)
        return
    print("Start 2d anti-spoofing without screen")
    press = KeyBoardPress()
    while True:
        try:
            if press.kbhit():
                if press.getch() == chr(27).encode():
                    break
            time.sleep(0.01)
        except KeyboardInterrupt:
            break

    faceMe_sdk.stop_2d_antispoofing()

def detect_image_quality(command_args):
    if command_args.image_path == "":
        return show_help_and_exit(command_args.command, "-photo")
    global faceMe_sdk
    options={}
    ret, image = faceMe_sdk.convert_to_faceMe_image(command_args.image_path)
    if FR_FAILED(ret):
        print("Convert to FaceMe image failed")
        return

    ret, detect_results = faceMe_sdk.detect_image_quality(image, options)
    if FR_FAILED(ret):
        print("Detect image quality failed, return:", ret)
        return 
    face_index = 0
    for detect_result in detect_results:
        print("{")
        print("faceIndex:", face_index)
        print("Result:")
        print("\t{")
        print(ToQualityIssueString(detect_result['issue'], detect_result['occlusionReason'], "\t"))
        print("\t}")
        print("}")
        face_index += 1
    return

def switch_command(command_args):
    switcher = {
        "activate": activate_license,
        "deactivate": deactivate_license,
        "renewlicense": renew_license,

		"recognizeface": recognize_face,
		"detectface": detect_face,

        "listusers": list_users,
        "registeruser": register_user,
		"unregisteruser": unregister_user,
        "addface": add_face,
        "removeface": remove_face,

        "compare": compare,
		"compareimages": compare_images,
        "searchsimilarface": search_similar_face,
        "getfacefeature": get_face_feature,
        "getfacethumbnail": get_face_thumbnail,

        "listcameras": list_cameras,

        "recognizevideos": recognize_videos,

        "list2dantispoofingcameras": list_2d_antispoofing_cameras,
        "antispoofing2d": do_anti_spoofing_2d,

        "detectimagequality":detect_image_quality
    }
    func = switcher.get(command_args.command, None)
    return func

faceMe_sdk = None
spoofing_result_handler = None
camera_recognizer_handler = None
def main(argv):
    global faceMe_sdk
    if platform.architecture()[0] != '64bit':
        print("[ERROR] FaceMe SDK only supports x64")
        return 

    command_args = ParseArg()
    ret = parse_command_line(argv, command_args)
    if FR_FAILED(ret):
        return
    command_function = switch_command(command_args)
    if command_function is None:
        wrong_command()
        return
    ret = initialize_SDK(command_args)
    if FR_FAILED(ret):
        print("initialize SDK failed")
        return
    command_function(command_args)
    faceMe_sdk.finalize()

if __name__ == "__main__":
    main(sys.argv[1:])

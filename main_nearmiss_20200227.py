import cv2
from datetime import datetime, timedelta
import queue

import time
import socket
import numpy as np
import sys, os
import traceback

from PIL import Image
from collections import deque
import sys
import os
import urllib
import tensorflow.contrib.tensorrt as trt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tensorflow as tf
import scipy
import time
import serial
from tf_trt_models.detection import download_detection_model, build_detection_graph
from utils.od_utils import read_label_map, build_trt_pb, load_trt_pb, \
                           write_graph_tensorboard, detect
from sort import *
from object import object

import threading
import warnings
warnings.filterwarnings("ignore")


def TriggerEvent(eventSystem, eventType, ttcH, ttcW, slope, gps_str):
    
    event_time = datetime.now()
    event_start = (datetime.now() - timedelta(seconds = DELTA_T)).strftime('%Y-%m-%d %H:%M:%S')
    event_end = (datetime.now() + timedelta(seconds = DELTA_T)).strftime('%Y-%m-%d %H:%M:%S')
    event_info = eventSystem + ',' + eventType + ',' + str(round(ttcH,1)) + ',' + str(round(ttcW,1)) + ',' + str(round(slope,2)) + ',' + gps_str + ',' + str(event_start) + ',' + str(event_end)
    sock = socket.socket()
    sock.settimeout(10)
    sock.connect((TCP_IP, TCP_PORT))
    sock.send( str(METADATA_SIZE).ljust(16).encode())
    sock.send( str(event_info).ljust(METADATA_SIZE).encode())
    sock.close()
#     print(event_info)


class UploadThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        print('UploadThread Init!')
        
    def run(self):
        print('UploadThread Start!')
        while True:
            if restart == True:
                break
            #print('UploadFrameQueue length:', len(uploadFrameQueue))
            while len(uploadFrameQueue) != 0:
                if restart == True:
                    break
                try:
                    (frame, frame_ID) = uploadFrameQueue.pop(0)
                    print(frame_ID)
            
                    result, imgencode = cv2.imencode('.jpg', frame, ENCODE_PARAM)
                    data = np.array(imgencode)
                    stringData = data.tostring()
                    sock = socket.socket()
                    sock.settimeout(10)
                    sock.connect((TCP_IP, TCP_PORT))
                    sock.send( str(len(stringData)).ljust(16).encode())
            #             print(str(len(stringData)).ljust(16))
                    sock.send( stringData )
                    #time.sleep(1)
                    sock.close()
                    print('UploadFrameQueue length:', len(uploadFrameQueue))
                    
                except Exception as e:
                    print(str(e))
            time.sleep(1)



uploadFrameQueue = []
restart = False

        
if __name__ == '__main__':
    '''
    Main Process
    '''
    
    TCP_IP = '128.95.29.66'
    TCP_PORT = 80
    
    QUEUE_MAX_SIZE = 180 # size of cached frames
    
    DELTA_T = 12 # seconds
    
    METADATA_SIZE = 100
    
    ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY),90]
    
    uploadThread = UploadThread() 
    uploadThread.start()

    try:		
        pb_path = './data/ssd_inception_v2_coco_trt.pb'
        input_names = ['image_tensor']
        trt_graph = load_trt_pb(pb_path)
        tf_config = tf.ConfigProto()
        tf_config.gpu_options.allow_growth = True
    
        tf_sess = tf.Session(config=tf_config, graph=trt_graph)
    
        tf_input = tf_sess.graph.get_tensor_by_name(input_names[0] + ':0')
        tf_scores = tf_sess.graph.get_tensor_by_name('detection_scores:0')
        tf_boxes = tf_sess.graph.get_tensor_by_name('detection_boxes:0')
        tf_classes = tf_sess.graph.get_tensor_by_name('detection_classes:0')
        tf_num_detections = tf_sess.graph.get_tensor_by_name('num_detections:0')
    
        tracker_ped = Sort()
        tracker_car = Sort()
        objs = deque(maxlen=50)
        frameNum = 0    
        
        
        # upload trigger. When event is detected, uploadTrigger is set as 1 
        # and the main process start to send frames to upload frame queue.
        uploadTrigger = 0
        # ending time of frame upload. When event is detected, the ending time should be updated.
        uploadEndTime = datetime.now() + timedelta(seconds = DELTA_T)
        
        
        # a queue to store videos in the main process
        frameQueue = []
        # a queue to store videos in the main process
        frameIDQueue = []
        
        interface = os.popen('ls /dev/tty* | grep "USB"').read()
        gps = serial.Serial(interface.replace('\n',''), baudrate = 4800)
        # video capturing
        cap = cv2.VideoCapture(0)
    #    cap = cv2.VideoCapture('rtsp://collisionavoid:Pierce2019@192.168.2.129:554/trackID=1')
    #    cap = cv2.VideoCapture('rtsp://starlab:star*lab@192.168.2.129:554/trackID=1')
        
        # sequence number of a frame in one second, used for setting the frame's name (frame_ID)
        frame_num_in_one_second = 0 
        
        pre_second = (datetime.now() - timedelta(seconds = 1)).strftime('%S')
        
        # capturing frames
        while(True):
            ret,frame = cap.read()
			
            #if frame is None:
            #    continue
            timeCur = time.time()
            image_resized = cv2.resize(frame, (300,300))
            now = datetime.now()
            curr_timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
            curr_second = datetime.now().strftime('%S')
            frame = cv2.resize(frame, (640,480))

            ######### Object Detection ##########
            scores, boxes, classes, num_detections = tf_sess.run([tf_scores, tf_boxes, tf_classes, tf_num_detections], feed_dict={
                tf_input: image_resized[None, ...]
            })
    
            boxes = boxes[0] # index by 0 to remove batch dimension
            scores = scores[0]
            classes = classes[0]
            classes.astype(int)
            num_detections = int(num_detections[0])
            dets_ped = []
            dets_car = []
            for i in range(num_detections):
                if classes[i] == 1 or classes[i] == 2 or classes[i] == 4:
                    if boxes[i][2] > 0.3 and (boxes[i][2]-boxes[i][0])*(boxes[i][3]-boxes[i][1]) > 0.001:
                        box = boxes[i] * np.array([frame.shape[0], frame.shape[1], frame.shape[0], frame.shape[1]])
                        box.astype(int)
                        dets_ped.append([box[1],box[0],box[3],box[2],scores[i]])
                elif classes[i] == 3 or classes[i] == 6 or classes[i] == 8:
                    if boxes[i][2] > 0.3 and (boxes[i][2]-boxes[i][0])*(boxes[i][3]-boxes[i][1]) > 0.003:
                        box = boxes[i] * np.array([frame.shape[0], frame.shape[1], frame.shape[0], frame.shape[1]])
                        box.astype(int)
                        dets_car.append([box[1],box[0],box[3],box[2],scores[i]])
            dets_ped = np.asarray(dets_ped)
            dets_car = np.asarray(dets_car)
    
            ######### Object Tracking ##########
            tracks_ped = tracker_ped.update(dets_ped)
            tracks_car = tracker_car.update(dets_car)
            boxesTk = []
            indexIDs = []
    
            for track in tracks_ped:
                boxesTk.append([track[0], track[1], track[2], track[3], 'ped'])
                indexIDs.append(int(track[4]))
            for track in tracks_car:
                boxesTk.append([track[0], track[1], track[2], track[3], 'veh'])
                indexIDs.append(int(track[4]))
            for obj in objs:
                obj.setUpdated(False)
            if len(boxesTk) > 0:
                i = int(0)
                for box in boxesTk:
                    (x1,y1) = (int(box[0]), int(box[1]))
                    (x2,y2) = (int(box[2]), int(box[3]))
                    category = box[4]
                    ID = indexIDs[i]
                    found = False
    
                    # update objs list
                    for obj in objs:
                        if obj.getID() == ID:
                            obj.add2tracks([x1,y1,x2,y2])
                            obj.setUpdated(True)
                            found = True
                            break
                    if found == False:
                        objNew = object(ID,category)
                        objs.append(objNew)
    
                    # Draw
                    color = [0,255,0]
                    cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                    #text = "{}".format(indexIDs[i])
                    cv2.putText(frame, category, (x1+5,y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    i += 1
    
            ######### Near miss detection #########
            nearmiss = False
            eventSystem = 'N/A'
            eventType = 'N/A'
            ttcH = -1
            ttcW = -1
            slope = -1
            gps_str = 'N/A'
            for obj in objs:
                trajs = obj.getTracks()
                framesToUse = 12
                if len(trajs) >= framesToUse:
                    if obj.getUpdated() == True:
    
                        left = [tr[0] for tr in trajs][-framesToUse:]
                        right = [tr[2] for tr in trajs][-framesToUse:]
                        top = [tr[1] for tr in trajs][-framesToUse:]
                        bottom = [tr[3] for tr in trajs][-framesToUse:]
                        centers = [[int((tr[1]+tr[3])/2),int((tr[0]+tr[2])/2)] for tr in trajs][-framesToUse:]
                        heights = [tr[3]-tr[1] for tr in trajs][-framesToUse:]
                        widths = [tr[2]-tr[0] for tr in trajs][-framesToUse:]
                        times = [tr[4] for tr in trajs][-framesToUse:]
                        slopeH, intercept, r_value, p_value, std_err = scipy.stats.linregress(times, heights)
                        slopeW, intercept, r_value, p_value, std_err = scipy.stats.linregress(times, widths)
    
                        if slopeH == 0 or slopeW == 0:
                            continue
                        ttcH = np.mean(heights)/slopeH
                        ttcW = np.mean(heights)/slopeW
                        thMin = 6
                        thMax = 12
    #                     if ttcH > 0  and ttcW > 0 and min(ttcH,ttcW) < ttcThre:
                        if max(ttcH,ttcW)<thMax and min(ttcH,ttcW)<thMin and min(ttcH,ttcW)>0:
                            c = np.transpose(centers)
                            slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(c[0],c[1])
    #                         if c[0][-1] > np.mean(c[0]) and c[0][0] < np.mean(c[0]):
                            (h,w) = (frame.shape[0],frame.shape[1])
                            cx = np.mean(c[0])
                            cy = np.mean(c[1])
                            k1 = 0.1 #weight
                            k2 = 0.1 #weight
                            botThre = 0.6
                            slopeThre = k1*cx/h + k2*np.abs(cy/w*2-1)
                            if (np.abs(cy/w*2-1) < 0.08) or (slope * (cy/w*2-1) <= -0.1 and np.abs(slope) < slopeThre):
                                if bottom[-1] > botThre*h:
                                    nearmiss = True
                                    eventSystem = 'TELS'
                                    eventType = obj.getCategory()
                                    color = [0,0,255]
                                    tr = trajs[-1]
                                    cv2.rectangle(frame, (tr[0],tr[1]), (tr[2],tr[3]), color, 3)
                                    text = "{}".format(obj.getID())+':ttc='+str(round(ttcH,1))+','+str(round(ttcW,1)) +' slope='+str(round(slope,2))
                                    cv2.putText(frame, text, (tr[0],tr[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    				# show gps on frame
                                    while(True):
                                        line = gps.readline()
                                        line = str(line)
                                        data = line.split(',')
                                        if "GPRMC" in data[0]:
                                            gps_str = data[3]+data[4]+','+data[5]+data[6]
                                            cv2.putText(frame, data[3]+data[4], (520,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0,255,255], 2)
                                            cv2.putText(frame, data[5]+data[6], (520,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0,255,255], 2)
                                            break
                                    #cv2.imwrite('./images/'+str(time.ctime())+'.jpg',frame)
    
    
            cv2.putText(frame, str(round(1/(time.time() - timeCur),2))+' fps', (30,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0,255,255], 2)
            frame = cv2.resize(frame, (320,240))

             # check second of current time
            if curr_second != pre_second:
                # if second is changed, the frame number reset to zero
                frame_num_in_one_second = 0
                pre_second = curr_second
            else: 
                # if second does not change, the frame number add one
                frame_num_in_one_second += 1        
            
            frame_ID = curr_timestamp + '_' + str(frame_num_in_one_second)
            
            
            if len(frameQueue) < QUEUE_MAX_SIZE:
                frameQueue.append(frame)
                frameIDQueue.append(frame_ID)
                
            if len(frameQueue) == QUEUE_MAX_SIZE:
                frameQueue.pop(0)
                frameQueue.append(frame)
                frameIDQueue.pop(0)
                frameIDQueue.append(frame_ID)
            
            # if upload is triggered, 
            if uploadTrigger == 1:
                # if current time has not exceed the upload ending time, keep send frames to the upload frame queue
                if now <= uploadEndTime:  
                    uploadFrameQueue.append((frame, frame_ID))
                else:
                    uploadTrigger = 0          
            
            random_event = (int(datetime.now().strftime('%M'))%15 == 0) & (int(datetime.now().strftime('%S'))<3)
            
            if nearmiss == True:
                # TELS Event
                # trigger an event               
                print('Trigger a TELS event.')
                
                if uploadTrigger == 0:
                    uploadEndTime = datetime.now() + timedelta(seconds = DELTA_T)
                    uploadTrigger = 1
                    uploadFrameQueue += list(zip(frameQueue, frameIDQueue))
                else:
                    uploadEndTime = datetime.now() + timedelta(seconds = DELTA_T)
                
                TriggerEvent(eventSystem, eventType, ttcH, ttcW, slope, gps_str)
            elif random_event:
                # Random Video
                # trigger an event               
                print('Trigger a random event.')
                
                if uploadTrigger == 0:
                    uploadEndTime = datetime.now() + timedelta(seconds = DELTA_T)
                    uploadTrigger = 1
                    uploadFrameQueue += list(zip(frameQueue, frameIDQueue))
                else:
                    uploadEndTime = datetime.now() + timedelta(seconds = DELTA_T)

                eventSystem = 'random'
                while(True):
                    line = gps.readline()
                    line = str(line)
                    data = line.split(',')
                    if "GPRMC" in data[0]:
                        gps_str = data[3]+data[4]+','+data[5]+data[6]
                        break
                TriggerEvent(eventSystem, eventType, ttcH, ttcW, slope, gps_str)
                
                
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                restart = True
                break

        cap.release()
        cv2.destroyAllWindows()
    
    except Exception as e:
        f = open("log.txt","a+")
        f.write(time.ctime() + ": " + str(e) + '\n')
        f.close()
        print(str(e))
        restart = True

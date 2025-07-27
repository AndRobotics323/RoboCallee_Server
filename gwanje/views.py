# from django.http import HttpResponse
from django.shortcuts import render, get_object_or_404, redirect
from django.utils import timezone
from django.http import StreamingHttpResponse , JsonResponse
from django.http import HttpResponseNotAllowed

from .camera import VideoCamera


import posix_ipc
import mmap

import socket
import struct
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

# Create your views here.




def gwanje_cam(request):

    return render(request,  'gwanje/gwanje_cam.html'   )
                     



# Python. 장고 버전
dev_num = 0
# dev_num = 2

# camera_instance = VideoCamera(dev_num)

def gen(camera):
    while True:
        frame = camera.get_frame()
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def video_feed(request):
    return StreamingHttpResponse(gen(camera_instance),
                                 content_type='multipart/x-mixed-replace; boundary=frame')

                                 


# 얘는 지금 당장은
def markers_api(request):
    data = camera_instance.get_markers()
    return JsonResponse(data, safe=False)



                              




# C++ 버전. 

'''
def video_feed(request):
    return StreamingHttpResponse(stream_generator(), content_type='multipart/x-mixed-replace; boundary=frame')
'''


# TCP 버전
'''
def stream_generator():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("127.0.0.1", 9000))
    server_socket.listen(1)
    conn, _ = server_socket.accept()

    while True:
        data = conn.recv(4)
        if not data:
            break
        length = struct.unpack('!I', data)[0]
        buffer = b''
        while len(buffer) < length:
            buffer += conn.recv(length - len(buffer))

        img_array = np.frombuffer(buffer, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        _, jpeg = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

               
'''



# shared memory 버전
'''
SHM_NAME = "/shared_frame"
WIDTH, HEIGHT, CHANNELS = 1920, 1080, 3
FRAME_SIZE = WIDTH * HEIGHT * CHANNELS


def stream_generator():
    shm = posix_ipc.SharedMemory(SHM_NAME)
    mapfile = mmap.mmap(shm.fd, FRAME_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
    shm.close_fd()

    while True:
        mapfile.seek(0)
        frame_bytes = mapfile.read(FRAME_SIZE)
        frame_array = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((HEIGHT, WIDTH, CHANNELS))
        
        ret, jpeg = cv2.imencode('.jpg', frame_array)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

'''








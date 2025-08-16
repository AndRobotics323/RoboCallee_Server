# ocr_from_mjpeg.py
import requests
import cv2
import numpy as np
import easyocr
import base64
from pupil_apriltags import Detector



# streaming_flask_url = "http://192.168.0.168:5000/stream" # arm2 주소임
streaming_flask_url = "http://192.168.35.138:5000/stream"



# === 카메라 내부 파라미터 설정 ===
camera_matrix = np.array([[1018.8890899848071, 0., 372.64373648977255],
                          [0., 1016.7247236426332, 229.30521863962326],
                          [0., 0., 1.]], dtype=np.float32)
dist_coeffs = np.array([-0.4664, 2.0392, 0.00035, -0.00077, -16.977], dtype=np.float64)

tag_size = 0.02  # 단위: meter

# === Detector 객체는 한 번만 생성 ===
_apriltag_detector = Detector(families="tag36h11")


# === OCR ===
reader = easyocr.Reader(['ko', 'en'], gpu=False)




def run_ocr_from_flask():
    stream = requests.get(, stream=True)
    byte_data = b''

    for chunk in stream.iter_content(chunk_size=1024):
        byte_data += chunk
        start = byte_data.find(b'\xff\xd8')
        end = byte_data.find(b'\xff\xd9')
        if start != -1 and end != -1:
            jpg = byte_data[start:end+2]
            npimg = np.frombuffer(jpg, np.uint8)
            frame = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
            break

    result = reader.readtext(frame)

    word_coords = []

    for (bbox, text, conf) in result:
        # (tl, tr, br, bl) = bbox
        # tl = tuple(map(int, tl))
        # br = tuple(map(int, br))
        # cv2.rectangle(img, tl, br, (0, 255, 0), 2)
        # cv2.putText(img, text, tl, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        pts = np.array(bbox).astype(int)
        
        coord_list = []
        for (x, y) in pts:
            x, y = int(x), int(y)
            coord_list.append((x, y))
            cv2.circle(frame, (x, y), 3, (255, 255, 0), -1)  # 초록 점 찍기

        word_coords.append({'text': text, 'coords': coord_list})


    # 이미지 base64 인코딩
    _, buf = cv2.imencode('.jpg', frame)
    img_b64 = base64.b64encode(buf).decode('utf-8')

    # return img_b64, [r[1] for r in result]
    return img_b64, word_coords







### 여긴 임시. 이미 있는 거(로봇팔 안에서 april tag 탐지하는)
### 이용해도 됨

def detect_april_from_flask():
    stream = requests.get(, stream=True)
    byte_data = b''

    for chunk in stream.iter_content(chunk_size=1024):
        byte_data += chunk
        start = byte_data.find(b'\xff\xd8')
        end = byte_data.find(b'\xff\xd9')
        if start != -1 and end != -1:
            jpg = byte_data[start:end+2]
            npimg = np.frombuffer(jpg, np.uint8)
            frame = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
            break

##################################################

    result = reader.readtext(frame)

    word_coords = []

    for (bbox, text, conf) in result:
        # (tl, tr, br, bl) = bbox
        # tl = tuple(map(int, tl))
        # br = tuple(map(int, br))
        # cv2.rectangle(img, tl, br, (0, 255, 0), 2)
        # cv2.putText(img, text, tl, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        pts = np.array(bbox).astype(int)
        
        coord_list = []
        for (x, y) in pts:
            x, y = int(x), int(y)
            coord_list.append((x, y))
            cv2.circle(frame, (x, y), 3, (255, 255, 0), -1)  # 초록 점 찍기

        word_coords.append({'text': text, 'coords': coord_list})


    # 이미지 base64 인코딩
    _, buf = cv2.imencode('.jpg', frame)
    img_b64 = base64.b64encode(buf).decode('utf-8')

    # return img_b64, [r[1] for r in result]
    return img_b64, word_coords
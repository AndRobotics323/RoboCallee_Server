# ocr_from_mjpeg.py
import requests
import cv2
import numpy as np
import easyocr
import base64

reader = easyocr.Reader(['ko', 'en'], gpu=False)

def run_ocr_from_flask():
    stream = requests.get("http://192.168.0.167:5000/stream", stream=True)
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

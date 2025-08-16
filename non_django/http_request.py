
import requests
import base64
import cv2
import numpy as np
import re

# CJ 192.168.0.189

django_url = 'http://192.168.5.17:8000/gwanje/ocr_from_flask_stream/'


defined_models =['나이키', '아디다스', '뉴발란스', '반스', '컨버스','푸마']
defined_colors =['white', 'black', 'red', 'blue', 'green', 'yellow', 'gray', 'brown', 'pink', 'purple']
defined_sizes = ['230', '235', '240', '245', '250', '255', '260', '265', '270', '275', '280', '285', '290']



def ask_django_ocr(url , mode):
    # data = {'name': 'CJ'}
    response = requests.post(url)

    data = response.json()

    print(response.status_code)

    if len(data) == 0:
        print("탐지된 텍스트 없음! ")
        return None
    
    word_coords = data['results']
    
    # 예시 데이터 구조
    print( word_coords )

    # [    {        'text': 'Hello',        'coords': [(100, 200), (150, 200), (150, 230), (100, 230)]    },
    #     {        'text': '안녕',        'coords': [(300, 400), (350, 400), (350, 430), (300, 430)]    }]

    if mode == 'get_coords':
        return word_coords


    elif mode == 'get_shoe_info':
        tmp_json = {'model': '아직', 'color' : 'yet', 'size': -1}
        
        for item in word_coords:
            text = item['text']

            if re.fullmatch(r'[A-Za-z]+', text) and text in defined_colors:
                tmp_json['color'] = text 
            elif re.fullmatch(r'[가-힣]+', text) and text in defined_models:
                tmp_json['model'] = text
            elif re.fullmatch(r'[0-9]+', text) and text in defined_sizes:
                tmp_json['size'] = int(text)

        # 위의 예시 데이터 구조와 통일성 갖추기 위함
        return [tmp_json]


    else : # mode == 'show_image'
        # Base64 디코딩
        img_b64 = data['image']
        img_bytes = base64.b64decode(img_b64)

        # NumPy 배열로 변환
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        # 화면에 표시
        cv2.imshow('Received Image', img)

        while True:
            key = cv2.waitKey(1) & 0xFF  # 1ms 대기 후 키 입력 확인
            
            # 예: 'q' 키를 누르면 종료
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        return None



if __name__ == '__main__':
    
    # ask_django_ocr(django_url, 'get_coords')
    ask_django_ocr(django_url, 'get_shoe_info')
    # ask_django_ocr(django_url, 'show_image')




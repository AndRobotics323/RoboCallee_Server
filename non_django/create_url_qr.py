import qrcode
import cv2
import numpy as np

# ==== 직접 입력 부분 ====
# ip = "192.168.146.131"   # 서버 컴퓨터 내부망 IP
# port = 8000              # 포트

image_path = "url_as_qr.jpg"  # 저장할 QR 코드 파일 경로

# url = f"http://{ip}:{port}"
url = 'https://robocallee.jp.ngrok.io'

if __name__ == "__main__":

    # QR 코드 생성 (PIL 이미지)
    qr = qrcode.make(url)
    qr.save(image_path)

    print(f"서버 주소: {url}")
    print("QR코드가 {image_path} 로 저장됨. 핸드폰으로 찍으면 접속 가능! 창을 닫으려면 q ")

    # PIL 이미지를 OpenCV 형식(numpy array)으로 변환
    qr_cv = np.array(qr.convert("RGB"))
    qr_cv = cv2.cvtColor(qr_cv, cv2.COLOR_RGB2BGR)

    # OpenCV 창에 QR코드 띄우기
    cv2.imshow("QR Code", qr_cv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

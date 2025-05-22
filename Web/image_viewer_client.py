import requests
import cv2
import numpy as np
import time

def main():
    url = 'http://localhost:8000/camera/image'  # 서버 주소 (IP 바꿔도 됨)

    while True:
        start_time = time.time()
        try:
            response = requests.get(url, timeout=1.0)
            if response.status_code == 200:
                image_array = np.frombuffer(response.content, np.uint8)
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow('RealSense Stream from Flask Server', frame)
            else:
                print("Failed to get image:", response.status_code)
        except Exception as e:
            print("Error:", e)

        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

        elapsed = time.time() - start_time
        sleep_time = max(0, (1.0 / 30.0) - elapsed)
        time.sleep(sleep_time)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

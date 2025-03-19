import requests
import time

API_URL = "http://192.168.1.188:20000/osc/state"

def poll_camera_state():
    try:
        response = requests.post(API_URL, timeout=5)
        if response.status_code == 200:
            state = response.json()
            print("Camera State:", state)
        else:
            print("Failed to get state:", response.status_code)
    except requests.exceptions.RequestException as e:
        print("Error:", e)

if __name__ == "__main__":
    while True:
        poll_camera_state()
        time.sleep(1)

import requests

FIREBASE_URL = "https://ros2-project-2a2df-default-rtdb.firebaseio.com"

data = {
    "latitude": 24.123456,
    "longitude": 120.654321
}

r = requests.patch(f"{FIREBASE_URL}/gps.json", json=data)
print(r.status_code, r.text)
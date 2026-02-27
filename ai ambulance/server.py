from flask import Flask, render_template, jsonify
import random
import time
import joblib
import numpy as np

app = Flask(__name__)

# Load trained AI model
model = joblib.load("eta_model.pkl")

# Store history
speed_history = []
eta_history = []

# Starting location (Udupi example)
lat = 13.3409
lon = 74.7421

@app.route("/")
def home():
    return render_template("dashboard.html")

@app.route("/data")
def data():

    global lat, lon

    # Simulate movement
    lat += random.uniform(0.0001, 0.0005)
    lon += random.uniform(0.0001, 0.0005)

    distance = random.randint(100, 1000)   # meters
    speed = random.randint(40, 90)         # km/h

    # AI ETA Prediction
    eta = model.predict(np.array([[distance, speed]]))[0]

    # Store history
    speed_history.append(speed)
    eta_history.append(float(eta))

    if len(speed_history) > 10:
        speed_history.pop(0)
        eta_history.pop(0)

    signal = "GREEN" if distance < 300 else "RED"

    return jsonify({
        "latest": {
            "distance": distance,
            "speed": speed,
            "eta": round(float(eta),2),
            "signal": signal,
            "lat": lat,
            "lon": lon
        },
        "speed_history": speed_history,
        "eta_history": eta_history
    })

if __name__ == "__main__":
    app.run(debug=True)
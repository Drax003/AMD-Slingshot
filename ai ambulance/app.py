import streamlit as st
import random
import time
import joblib
import numpy as np
import pandas as pd

# Load AI model
model = joblib.load("eta_model.pkl")

st.set_page_config(page_title="Smart Ambulance AI", layout="wide")

st.title("🚑 Smart Ambulance AI Control Dashboard")

# Layout
col1, col2, col3, col4 = st.columns(4)

# Initialize session state
if "lat" not in st.session_state:
    st.session_state.lat = 13.3409
    st.session_state.lon = 74.7421
    st.session_state.speed_data = []
    st.session_state.eta_data = []

# Simulate movement
st.session_state.lat += random.uniform(0.0001, 0.0005)
st.session_state.lon += random.uniform(0.0001, 0.0005)

distance = random.randint(100, 1000)
speed = random.randint(40, 100)

eta = model.predict(np.array([[distance, speed]]))[0]

st.session_state.speed_data.append(speed)
st.session_state.eta_data.append(float(eta))

if len(st.session_state.speed_data) > 20:
    st.session_state.speed_data.pop(0)
    st.session_state.eta_data.pop(0)

signal = "🟢 GREEN" if distance < 300 else "🔴 RED"

# Metrics
col1.metric("Distance (m)", distance)
col2.metric("Speed (km/h)", speed)
col3.metric("ETA (mins)", round(float(eta),2))
col4.metric("Signal Status", signal)

st.divider()

# Charts
chart_col1, chart_col2 = st.columns(2)

with chart_col1:
    st.subheader("Speed History")
    st.line_chart(st.session_state.speed_data)

with chart_col2:
    st.subheader("ETA Prediction History")
    st.line_chart(st.session_state.eta_data)

st.divider()

# Map
st.subheader("Live Ambulance Location")

map_data = pd.DataFrame({
    'lat': [st.session_state.lat],
    'lon': [st.session_state.lon]
})

st.map(map_data)

time.sleep(2)
st.rerun()
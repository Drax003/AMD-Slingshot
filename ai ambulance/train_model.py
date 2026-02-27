import pandas as pd
from sklearn.ensemble import RandomForestRegressor
import pickle

# Sample dataset (replace with real later)
data = pd.DataFrame({
    "distance": [100,200,300,400,500,600,700],
    "speed":    [40,50,60,45,55,65,70],
    "eta":      [8,15,18,22,25,28,32]
})

X = data[["distance","speed"]]
y = data["eta"]

model = RandomForestRegressor()
model.fit(X,y)

with open("eta_model.pkl","wb") as f:
    pickle.dump(model,f)

print("Model trained successfully.")
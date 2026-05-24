import joblib
import pandas as pd
import numpy as np
from pathlib import Path

from ml import FEATURE_COLS, TARGET_COLS, MODEL_FILE

# Load model once at module level with error handling
if not MODEL_FILE.exists():
    raise FileNotFoundError(f"Model file not found at {MODEL_FILE}. Please run ml.py first.")
model = joblib.load(MODEL_FILE)


def predict(x, y, orientation, velocityx, velocityy, steer_angle, dist_to_target, yaw_err, obs1_dist, obs2_dist):
    # Wrap inputs in a DataFrame with named columns to match training expectations and avoid UserWarning
    input_data = pd.DataFrame(
        [[x, y, orientation, velocityx, velocityy, steer_angle, dist_to_target, yaw_err, obs1_dist, obs2_dist]],
        columns=FEATURE_COLS
    )

    result = model.predict(input_data)[0]
    return result[0], result[1]


if __name__ == "__main__":
    # live...
    new_sensor_data = [-3.48, -4.12, -1.38, 0.12, -0.01, 0.08, 5.31, -2.96, 5.14, 4.67]

    steer, speed = predict(*new_sensor_data)

    print("=== Prediction from Sensor Input ===")
    print(f"  Input Features:")
    for name, val in zip(FEATURE_COLS, new_sensor_data):
        print(f"    {name:<20}: {val}")
    print(f"\n  Predicted Outputs:")
    print(f"    {'steer':<20}: {steer:.4f} rad  (steering angle)")
    print(f"    {'speed':<20}: {speed:.4f} units (target speed)")
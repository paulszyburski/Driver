import joblib
import pandas as pd
from pathlib import Path

from ml import FEATURE_COLS, TARGET_COLS, MODEL_FILE

model = joblib.load(MODEL_FILE)


def predict(x, y, orientation, velocityx, velocityy,
            steer_angle, dist_to_target, yaw_err, obs1_dist, obs2_dist) -> dict:
    """
    Predict the car's steering angle and speed from live sensor data.

    Returns a dict with keys: 'steer' (radians) and 'speed' (simulation units)
    """
    input_data = pd.DataFrame(
        [[x, y, orientation, velocityx, velocityy,
          steer_angle, dist_to_target, yaw_err, obs1_dist, obs2_dist]],
        columns=FEATURE_COLS
    )

    result = model.predict(input_data)[0]  # [steer, speed]
    return {col: round(float(val), 6) for col, val in zip(TARGET_COLS, result)}


if __name__ == "__main__":
    # live...
    new_sensor_data = [-3.48, -4.12, -1.38, 0.12, -0.01, 0.08, 5.31, -2.96, 5.14, 4.67]

    prediction = predict(*new_sensor_data)

    print("=== Prediction from Sensor Input ===")
    print(f"  Input Features:")
    for name, val in zip(FEATURE_COLS, new_sensor_data):
        print(f"    {name:<20}: {val}")
    print(f"\n  Predicted Outputs:")
    print(f"    {'steer':<20}: {prediction['steer']:.4f} rad  (steering angle)")
    print(f"    {'speed':<20}: {prediction['speed']:.4f} units (target speed)")
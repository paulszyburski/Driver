import pandas as pd
from pathlib import Path
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_squared_error, r2_score
import joblib

# Full column structure matching the CSV exactly
ALL_COLS = [
    "episode", "step",
    "x", "y", "orientation", "velocityx", "velocityy", "steer_angle",
    "dist_to_target", "yaw_err", "obs1_dist", "obs2_dist",
    "steer", "speed"
]

# Input features the model learns from
FEATURE_COLS = [
    "x", "y", "orientation", "velocityx", "velocityy",
    "steer_angle", "dist_to_target", "yaw_err", "obs1_dist", "obs2_dist"
]

# Outputs the model predicts
TARGET_COLS = ["steer", "speed"]

# Path to training data relative to the project root
CSV_FILE = Path(__file__).resolve().parent.parent.parent / "data" / "training_data_V1_2026-05-16_09-50-04.csv"

MODEL_FILE = Path(__file__).resolve().parent / "trained_robot_model.pkl"


def train_and_evaluate_model(csv_path: Path):
    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path, header=None, names=ALL_COLS)

    X = df[FEATURE_COLS]
    y = df[TARGET_COLS]

    print(f"Samples: {len(df):,}  |  Features: {len(FEATURE_COLS)}  |  Targets: {TARGET_COLS}")

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    # Train a Random Forest for multi-output regression
    print("\nTraining Random Forest Regressor...")
    model = RandomForestRegressor(n_estimators=100, random_state=42, n_jobs=-1)
    model.fit(X_train, y_train)
    print("Training complete.")

    # Evaluate per target
    y_pred = model.predict(X_test)
    y_pred_df = pd.DataFrame(y_pred, columns=TARGET_COLS)
    y_test_df = y_test.reset_index(drop=True)

    print("\n=== Model Performance ===")
    for col in TARGET_COLS:
        mse = mean_squared_error(y_test_df[col], y_pred_df[col])
        r2  = r2_score(y_test_df[col], y_pred_df[col])
        print(f"  [{col}]  MSE: {mse:.6f}   R²: {r2:.4f}")

    joblib.dump(model, MODEL_FILE)
    print(f"\nModel saved to: {MODEL_FILE}")

    return model


if __name__ == "__main__":
    try:
        train_and_evaluate_model(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Training data not found at '{CSV_FILE}'")

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pandas as pd

from core.ML.env import Env
from core.ML.utils import compute_power
from core.ML.controller import MLController
from core.ML.utils import log
import time
import joblib

ALL_COLS = [
    "episode", "step",
    "x", "y", "orientation", "velocityx", "velocityy", "steer_angle",
    "dist_to_target", "yaw_err", "obs1_dist", "obs2_dist",
    "steer", "speed"
]

CSV_FILE = Path(__file__).resolve().parent.parent.parent / "data" / "training_data_V1_2026-05-05_21-48-18.csv"

MODEL_FILE = Path(__file__).resolve().parent.parent.parent / "models" / "trained_robot_model.pkl"
MAX_TRAIN_SAMPLES = 4_500_000
EPOCHS = 200
CHECKPOINT_EVERY_EPOCHS = 10
BATCH_SIZE = 512
LEARNING_RATE = 8e-4
HIDDEN_LAYERS = (128, 64)

def setup():
    controller = MLController()

    df = pd.read_csv(CSV_FILE, header=None, names=ALL_COLS)
    sample_n = min(MAX_TRAIN_SAMPLES, len(df))
    df = df.sample(n=sample_n, random_state=42).reset_index(drop=True)

    X_train, _, y_train, _ = controller.split_data(df, test_size=0.2, random_state=42)
    print("Starting model training...")
    controller.train(
        X_train,
        y_train,
        checkpoint_path=MODEL_FILE,
        checkpoint_every=CHECKPOINT_EVERY_EPOCHS,
        epochs=EPOCHS,
        batch_size=BATCH_SIZE,
        learning_rate_init=LEARNING_RATE,
        hidden_layer_sizes=HIDDEN_LAYERS,
    )
    print("Training finished. Saving model...")
    controller.save_model(MODEL_FILE)
    return controller


def main():
    env = Env()
    MLcontroller = MLController()
    MLcontroller.load_model(MODEL_FILE)

    step = 0
    phase = 0

    while True:
        state = env.get_state(env.car)
        steer, speed = MLcontroller.predict(state)

        env.apply_control(steer, speed)
        env.step()

        time.sleep(1 / 240)

        log(state, step)
        step += 1

if __name__ == "__main__":
    #setup()
    main()

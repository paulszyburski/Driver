import math

import pybullet as p
import random
import pickle
import joblib
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

from core.utils import approach_target, steering_to_target, track_held_keys, is_facing_target, adjust_yaw_in_place


class Controller:
    def __init__(self):
        self.keys_held = set()

    def generate_random_action(self):
        action = []

        action_move = random.randint(-10, 10) # forward, backward, or no movement
        action_steer = random.randint(-60, 60)/100  # left, right, or no steering
        action_hold = random.randint(1, 480)  # hold the current action or not

        action.append(action_move) # forward or backward
        action.append(action_steer)  # left or right
        action.append(action_hold)

        return action

        
    def control(self, action, mode):# first position for drive second for steer third for hold
        speed = action[0]
        steer = action[1]
        hold = action[2]

        if mode == "manual":
            events = p.getKeyboardEvents()
            track_held_keys(events, self.keys_held)

            steer, speed = 0, 0
            if p.B3G_UP_ARROW in self.keys_held:
                speed = 20
            if p.B3G_DOWN_ARROW in self.keys_held:
                speed = -6
            if p.B3G_LEFT_ARROW in self.keys_held:
                steer = 0.4
            if p.B3G_RIGHT_ARROW in self.keys_held:
                steer = -0.4

        if mode == "random":
            pass
        
        elif mode == "approach_target":
            pass

        return steer, speed, hold
    
    def redirect(self):
        pass

class EndOn(Controller):
    def __init__(self, ):
        pass

    def scripted_controller(self, state, phase):
        yaw_target = -math.pi/2 if state["y"] >= 0 else math.pi/2
        yaw_err = state["orientation"] - yaw_target
        
        if phase == 0:
            
            y_pos = state["y"]
            if y_pos >= 0:
                target_pos = (0, 0.95)
            elif y_pos <= 0:
                target_pos = (0, -0.95)
            action = approach_target(state, target_pos)
            if action == [0,0,0]:
                phase = 1.0
            return action, phase
        
        if phase in [1.0, 1.1]:
            action, phase = adjust_yaw_in_place(yaw_err, phase)
            if action == (0,0,0):
                phase = 2
            return action, phase
        
        if phase == 2:
            action = approach_target(state, (0, 0.21))
            if action == [0, 0, 0]:
                phase = 3
            return action, phase
        
        if phase == 3:
            return [0, 0, 0], phase

class Paraller(Controller):
    def __init__(self, ):
        pass

    def scripted_controller(self, state, phase):
        pass


class MLController(Controller):
    FEATURE_COLS = [
        "x",
        "y",
        "orientation",
        "velocityx",
        "velocityy",
        "steer_angle",
        "dist_to_target",
        "yaw_err",
        "obs1_dist",
        "obs2_dist",
    ]

    def __init__(self, model=None):
        super().__init__()
        self.model = model
    # use sklearn

    def save_model(self, filename):
        if self.model is None:
            raise ValueError("No model available to save.")
        joblib.dump(self.model, filename)

    def load_model(self, filename):
        self.model = joblib.load(filename)
        return self.model

    def split_data(self, data, test_size=0.2, random_state=42):
        X = data[self.FEATURE_COLS]
        y = data[["steer", "speed"]]
        return train_test_split(X, y, test_size=test_size, random_state=random_state)

    def train(
        self,
        X_train,
        y_train,
        checkpoint_path=None,
        checkpoint_every=10,
        epochs=200,
        batch_size=512,
        learning_rate_init=8e-4,
        hidden_layer_sizes=(128, 64),
        patience=20,
        min_delta=1e-5,
        validation_split=0.1,
    ):
        X_subtrain, X_val, y_subtrain, y_val = train_test_split(
            X_train, y_train, test_size=validation_split, random_state=42
        )

        scaler = StandardScaler()
        X_subtrain_scaled = scaler.fit_transform(X_subtrain)
        X_val_scaled = scaler.transform(X_val)

        mlp = MLPRegressor(
            hidden_layer_sizes=hidden_layer_sizes,
            activation="relu",
            solver="adam",
            learning_rate_init=learning_rate_init,
            batch_size=batch_size,
            max_iter=1,
            warm_start=True,
            random_state=42,
            early_stopping=False,
            shuffle=True,
        )

        best_val_loss = float("inf")
        epochs_without_improve = 0

        for epoch in range(1, epochs + 1):
            mlp.fit(X_subtrain_scaled, y_subtrain)

            val_pred = mlp.predict(X_val_scaled)
            val_loss = float(np.mean((y_val.to_numpy() - val_pred) ** 2))
            print(f"Epoch {epoch}/{epochs} - train_loss: {mlp.loss_:.6f} - val_mse: {val_loss:.6f}")

            if val_loss + min_delta < best_val_loss:
                best_val_loss = val_loss
                epochs_without_improve = 0
            else:
                epochs_without_improve += 1

            if checkpoint_path and epoch % checkpoint_every == 0:
                self.model = Pipeline([("scaler", scaler), ("mlp", mlp)])
                self.save_model(checkpoint_path)
                print(f"Checkpoint saved at epoch {epoch}: {checkpoint_path}")

            if epochs_without_improve >= patience:
                print(f"Early stopping at epoch {epoch} (no val improvement for {patience} epochs).")
                break

        self.model = Pipeline([("scaler", scaler), ("mlp", mlp)])
        return self.model

    def predict(self, state):
        if self.model is None:
            raise ValueError("No model set for prediction.")

        features = [state[col] for col in self.FEATURE_COLS]
        pred = self.model.predict([features])[0]
        steer, speed = float(pred[0]), float(pred[1])
        return steer, speed

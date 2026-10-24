# Driver

This repository contains my earlier physical-robot parking prototype. It can handle basic parking, but I wasn't satisfied with its reliability. The newer rewrite currently lives only on my Raspberry Pi and will be added here in the future. After taking a break, I'm starting fresh with a better, more stable approach.

My robotics project with an mBot, Python, and PyBullet. It includes keyboard driving, ultrasonic experiments, and simulated parking with a learned controller.

This is still a work in progress.

## Run

Requires Python 3.13+ and [uv](https://docs.astral.sh/uv/).

```sh
uv sync
uv run python src/scripts/main.py
```

This opens the scripted parking simulation in a desktop window.

For the real mBot, connect it over USB and set the serial port in `src/driver/steerCar.py` (default: `/dev/ttyUSB0`):

```sh
uv run python src/driver/steerCar.py
```

Use **W/S** to drive, **A/D** to turn, **Space** to stop, and **Q** to quit.

## Experiments

- `src/core/hardware/` — mBot communication and ultrasonic experiments.
- `src/scripts/calibrate.py` and `src/scripts/map.py` — earlier camera-based physical-robot experiments.
- `src/core/ml/` — simulation, scripted parking, and model training.
- `src/scripts/record_data.py` — record parking episodes to local CSV files.
- `src/scripts/MLcontrol.py --train data/<recording>.csv` — train a model; omit `--train` to run the saved model.
- `tests/` — manual hardware checks; HID examples also need `hidapi`.

Run the scripts with `uv run python`. Recordings and new model files stay local; `models/trained_robot_model.pkl` is the earlier saved model.

from time import sleep

import cv2
import numpy as np
from picamera2 import Picamera2


CALIBRATION_WIDTH = 1944
CALIBRATION_HEIGHT = 2592


PIXEL_POSITION_SAMPLES = {
    (489, 915): (-30, 20),
    (615, 881): (-20, 20),
    (807, 841): (-10, 20),
    (1065, 839): (0, 20),
    (1286, 886): (10, 20),
    (1490, 923): (20, 20),
    (1609, 981): (30, 20),

    (445, 1085): (-30, 10),
    (562, 1085): (-20, 10),
    (744, 1049): (-10, 10),
    (1335, 1070): (10, 10),
    (1517, 1133): (20, 10),
    (1630, 1188): (30, 10),

    (412, 1280): (-30, 0),
    (533, 1307): (-20, 0),
    (736, 1328): (-10, 0),
    (1317, 1357): (10, 0),
    (1509, 1391): (20, 0),
    (1635, 1367): (30, 0),

    (418, 1467): (-30, -10),
    (539, 1528): (-20, -10),
    (720, 1596): (-10, -10),
    (983, 1622): (0, -10),
    (1246, 1635): (10, -10),
    (1449, 1601): (20, -10),
    (1564, 1567): (30, -10),

    (441, 1630): (-30, -20),
    (573, 1691): (-20, -20),
    (752, 1746): (-10, -20),
    (975, 1777): (0, -20),
    (1199, 1820): (10, -20),
    (1501, 1705): (30, -20),
}


class Camera:
    def __init__(self, res=(1296, 972)):
        self.camera = Picamera2()
        self.res = res
        self.configured = False

        # Resolution after rotating the image 90 degrees.
        self.full_width = res[1]
        self.full_height = res[0]

        scale_x = self.full_width / CALIBRATION_WIDTH
        scale_y = self.full_height / CALIBRATION_HEIGHT

        original_pixels = np.array(
            list(PIXEL_POSITION_SAMPLES.keys()),
            dtype=float,
        )

        # Scale calibration pixels to the lower camera resolution.
        pixels = original_pixels.copy()
        pixels[:, 0] *= scale_x
        pixels[:, 1] *= scale_y

        positions = np.array(
            list(PIXEL_POSITION_SAMPLES.values()),
            dtype=float,
        )

        self.crop_x1 = int(np.floor(pixels[:, 0].min()))
        self.crop_y1 = int(np.floor(pixels[:, 1].min()))
        self.crop_x2 = int(np.ceil(pixels[:, 0].max())) + 1
        self.crop_y2 = int(np.ceil(pixels[:, 1].max())) + 1

        features = self._features(
            pixels[:, 0],
            pixels[:, 1],
        )

        self.x_model = np.linalg.lstsq(
            features,
            positions[:, 0],
            rcond=None,
        )[0]

        self.y_model = np.linalg.lstsq(
            features,
            positions[:, 1],
            rcond=None,
        )[0]

    def _features(self, x, y):
        x = (
            np.asarray(x, dtype=float)
            - self.full_width / 2
        ) / (self.full_width / 2)

        y = (
            np.asarray(y, dtype=float)
            - self.full_height / 2
        ) / (self.full_height / 2)

        return np.column_stack([
            np.ones_like(x),
            x,
            y,
            x ** 2,
            x * y,
            y ** 2,
            x ** 3,
            x ** 2 * y,
            x * y ** 2,
            y ** 3,
        ])

    def configure_camera(self):
        config = self.camera.create_still_configuration(
            main={
                "size": self.res,
                "format": "RGB888",
            },
            buffer_count=1,
        )

        self.camera.configure(config)
        self.configured = True

    def take_picture(self, warmup_time=0):
        if not self.configured:
            raise RuntimeError(
                "Run configure_camera() first."
            )

        self.camera.start()

        try:
            sleep(warmup_time)
            frame = self.camera.capture_array()
        finally:
            self.camera.stop()

        frame = cv2.rotate(
            frame,
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        )

        return frame[
            self.crop_y1:self.crop_y2,
            self.crop_x1:self.crop_x2,
        ]

    def pixel_to_position(self, pixel_x, pixel_y):
        features = self._features(
            [pixel_x + self.crop_x1],
            [pixel_y + self.crop_y1],
        )

        return (
            float(features @ self.x_model),
            float(features @ self.y_model),
        )

    def free_positions_from_mask(
        self,
        mask,
        pixel_step=4,
    ):
        """
        Convert non-boundary image pixels into unique ground
        positions in centimetres.
        """

        sampled_mask = mask[::pixel_step, ::pixel_step]

        sampled_y, sampled_x = np.where(
            sampled_mask == 0
        )

        pixel_x = sampled_x * pixel_step
        pixel_y = sampled_y * pixel_step

        features = self._features(
            pixel_x + self.crop_x1,
            pixel_y + self.crop_y1,
        )

        ground_x = features @ self.x_model
        ground_y = features @ self.y_model

        # Many pixels correspond to the same centimetre position.
        # A set removes those duplicates immediately.
        return {
            (round(float(x)), round(float(y)))
            for x, y in zip(ground_x, ground_y)
        }

    def detect_color(self, frame, lower, upper):
        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )

        mask = cv2.inRange(
            hsv,
            np.asarray(lower, dtype=np.uint8),
            np.asarray(upper, dtype=np.uint8),
        )

        pixel_y, pixel_x = np.where(mask > 0)

        if len(pixel_x) == 0:
            return {}, mask

        features = self._features(
            pixel_x + self.crop_x1,
            pixel_y + self.crop_y1,
        )

        ground_x = features @ self.x_model
        ground_y = features @ self.y_model

        positions = {
            (int(px), int(py)): (int(gx), int(gy))
            for px, py, gx, gy in zip(
                pixel_x,
                pixel_y,
                ground_x,
                ground_y,
            )
        }

        return positions, mask

    def close(self):
        self.camera.close()


if __name__ == "__main__":
    camera = Camera()

    try:
        camera.configure_camera()
        frame = camera.take_picture(2)

        positions, mask = camera.detect_color(
            frame,
            lower=[77, 164, 93],
            upper=[119, 255, 222],
        )

        cv2.imwrite("camera_picture.jpg", frame)
        cv2.imwrite("color_mask.png", mask)

        print(f"Detected pixels: {len(positions)}")

        for pixel, position in list(positions.items())[:20]:
            print(f"{pixel} -> {position}")

    finally:
        camera.close()
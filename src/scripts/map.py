import sys
from pathlib import Path

import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from core.hardware.controller import BotController
from core.hardware.mapping import Mapping
from core.hardware.camera import Camera
from core.hardware.MBotlib.mBot import mBot


CAMERA_RESOLUTION = (1296, 972)

# Higher value = faster, but slightly less detailed.
PIXEL_STEP = 8


plt.ion()


def visualize_mapping(boundaries, free_space, current_position):
    plt.clf()

    if free_space:
        x, y = zip(*free_space)
        plt.scatter(x, y, s=5, alpha=0.3, label="Free space")

    if boundaries:
        x, y = zip(*boundaries)
        plt.scatter(x, y, s=10, label="Boundaries")

    plt.scatter(
        current_position[0],
        current_position[1],
        s=100,
        marker="x",
        label="Robot",
    )

    plt.axhline(0)
    plt.axvline(0)
    plt.axis("equal")
    plt.grid()
    plt.legend()

    plt.draw()
    plt.pause(0.01)


def map_out(bot, controller, camera, mapping):
    frame = camera.take_picture(warmup_time=0.3)
    positions, mask = camera.detect_color(
        frame,
        lower=[77, 164, 93],
        upper=[119, 255, 222],
    )

    robot_x, robot_y = controller.position

    for relative_x, relative_y in positions.values():
        mapping.add_boundary(robot_x + relative_x, robot_y + relative_y)

    frame_height, frame_width = mask.shape

    for pixel_y in range(0, frame_height, PIXEL_STEP):
        for pixel_x in range(0, frame_width, PIXEL_STEP):
            # Colored pixel means boundary.
            if mask[pixel_y, pixel_x] != 0:
                continue

            relative_x, relative_y = camera.pixel_to_position(
                pixel_x,
                pixel_y,
            )

            mapping.add_free_space(
                robot_x + relative_x,
                robot_y + relative_y,
            )


def main():
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")
    controller = BotController(bot)
    camera = Camera(res=CAMERA_RESOLUTION)
    mapping = Mapping()

    camera.configure_camera()

    for _ in range(3):
        map_out(bot, controller, camera, mapping)

        visualize_mapping(
            mapping.boundaries,
            mapping.free_space,
            controller.position,
        )

        controller.drive_distance(20)

    camera.close()
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()

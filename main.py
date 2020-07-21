import numpy as np

from sortingrobot import SortingRobot


def main(path, n_objects, spawn_special=False, save_video=False, seed=None):
    if seed:
        np.random.seed(seed)

    robot = SortingRobot(path, n_objects, spawn_special, save_video)

    robot.wait(3000)
    robot.run_simulation(5000)

    if spawn_special:
        robot.sort(list(range(n_objects + 1)))
    else:
        robot.sort(list(range(n_objects)))

    robot.wait(5000)
    robot.destroy()


if __name__ == '__main__':
    scene_path = './scenarios/scene_special.g'

    main(scene_path, 6, spawn_special=True, seed=987654)

import asyncio
import time
from pathlib import Path
import os
import psutil
import subprocess

import matplotlib.pyplot as plt
import numpy as np


def retrieve_stats():
    with open("log.txt") as f:
        travelled = float(f.readline().split(" ")[1])
        mean_distance_to_wall = float(f.readline().split(" ")[1])
        elapsed = float(f.readline().split(" ")[1])
        return {"travelled": travelled, "mean_distance_to_wall": mean_distance_to_wall,
                "elapsed": elapsed}


def kill(process):
    # kill subprocesses
    children = psutil.Process(process.pid).children()
    for child in children:
        child.kill()
    process.kill()
    time.sleep(1)

    # kill rviz
    subprocess.run(['pkill', '-f', 'rviz'])
    time.sleep(1)


async def run_simulation_and_read_log(pos_x=None, pos_y=None,
                                      target_velocity=None, ideal_distance=None,
                                      k_ang=None, k_lin=None, invert_direction=None):
    # delete log file
    file = Path("log.txt")
    if file.exists():
        os.remove(file)

    cmd = ""
    if pos_x is not None:
        cmd += f"export ROBOT_POS_X={pos_x};"
    if pos_y is not None:
        cmd += f"export ROBOT_POS_Y={pos_y};"
    cmd += "ros2 launch wall_following_diff_robot wall_following_diff_robot.launch.py"
    if target_velocity is not None:
        cmd += f" target_velocity:={target_velocity}"
    if ideal_distance is not None:
        cmd += f" ideal_distance:={ideal_distance}"
    if k_ang is not None:
        cmd += f" k_ang:={k_ang}"
    if k_lin is not None:
        cmd += f" k_lin:={k_lin}"
    if invert_direction is not None:
        cmd += f" invert_direction:={invert_direction}"

    process = await asyncio.create_subprocess_shell(cmd)
    time.sleep(1)

    while True:
        file = Path("log.txt")
        if file.exists() and file.stat().st_size > 10:
            kill(process)
            return retrieve_stats()


async def main():
    INITIAL_POSITION = [5.0, 4.0]
    stats = []

    # independent variables
    for offset_x in range(-2, 0):
        for offset_y in range(-3, -1):
            for invert_direction in [True, False]:
                for target_velocity in [0.3, 0.5]:
                    for ideal_distance in [0.5, 0.8, 1.0]:
                        for k_ang in [8.0, 16.0]:
                            for k_lin in [1.0, 2.0]:
                                stat = await run_simulation_and_read_log(INITIAL_POSITION[0] + offset_x/2,
                                                                         INITIAL_POSITION[1] + offset_y/2, target_velocity,
                                                                         ideal_distance, k_ang, k_lin, invert_direction)
                                stat["offset_x"] = offset_x
                                stat["offset_y"] = offset_y
                                stat["target_velocity"] = target_velocity
                                stat["ideal_distance"] = ideal_distance
                                stat["invert_direction"] = invert_direction
                                stat["k_ang"] = k_ang
                                stat["k_lin"] = k_lin
                                stats.append(stat)
                                break
                            break
                        break
                    break
                break
            break
        break

if __name__ == "__main__":
    asyncio.run(main())

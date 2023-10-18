import asyncio
import time
from pathlib import Path
import os
import psutil
import subprocess

import matplotlib.pyplot as plt
import numpy as np

INITIAL_POSITION = [5.0, 4.0]

def retrieve_stats():
    with open("log.txt") as f:
        travelled = float(f.readline().split(" ")[1])
        elapsed = float(f.readline().split(" ")[1])
        return {"travelled": travelled, "elapsed": elapsed}


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

async def run_simulation_and_read_log(pos_x=None, pos_y=None, target_velocity=None):
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

    process = await asyncio.create_subprocess_shell(cmd)
    time.sleep(1)

    while True:
        file = Path("log.txt")
        if file.exists() and file.stat().st_size > 10:
            kill(process)
            return retrieve_stats()

def dump_graphics(full_stats):
    def variable_graphic(filtered_stats, variable_name):
        values = set([stat[variable_name] for stat in filtered_stats])
        for value in values:
            filtered_stats = [stat for stat in filtered_stats if stat[variable_name] == value]
            print(filtered_stats)
            positions = [str((stat["offset_x"], stat["offset_y"])) for stat in filtered_stats]

            # scatter plot with (offset_x, offset_t) as x axis and elapsed as y axis
            plt.scatter(positions, [stat["elapsed"] for stat in filtered_stats])
            plt.title("Elapsed time from different initial positions")
            plt.savefig(f"elapsed_{variable_name}_{value}.png")
            plt.close()

            # scatter plot with (offset_x, offset_t) as x axis and travelled as y axis
            plt.scatter(positions, [stat["travelled"] for stat in filtered_stats])
            plt.title("Travelled distance from different initial positions")
            plt.savefig(f"travelled_{variable_name}_{value}.png")
            plt.close()

    variable_graphic(full_stats, "target_velocity")


async def main():
    stats = []

    for offset_x in range(-2,0):
        for offset_y in range(-3,-1):
             for target_velocity in range(4, 5):
                stat = await run_simulation_and_read_log(INITIAL_POSITION[0] + offset_x/2,
                                                           INITIAL_POSITION[1] + offset_y/2, target_velocity/10)
                stat["offset_x"] = offset_x
                stat["offset_y"] = offset_y
                stat["target_velocity"] = target_velocity
                stats.append(stat)

    dump_graphics(stats)


if __name__ == "__main__":
    asyncio.run(main())
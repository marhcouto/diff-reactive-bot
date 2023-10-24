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
        mean_distance_to_wall = float(f.readline().split(" ")[1])
        elapsed = float(f.readline().split(" ")[1])
        distances = np.array([float(x) for x in f.readline().split(" ")[1:]])
        return {"travelled": travelled, "mean_distance_to_wall": mean_distance_to_wall,
                "elapsed": elapsed, "distances": distances}


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
                                      k_ang=None, k_lin=None, invert_direction=None,
                                      old_controller=None, detection_by_line=None):
    # delete log file
    file = Path("log.txt")
    if file.exists():
        os.remove(file)

    # start timer
    start = time.time()

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
    if old_controller is not None:
        cmd += f" old_controller:={old_controller}"
    if detection_by_line is not None:
        cmd += f" detection_by_line:={detection_by_line}"

    process = await asyncio.create_subprocess_shell(cmd)
    time.sleep(1)

    while True:
        elapsed = time.time() - start
        if elapsed > 90:
            # process is stale
            kill(process)
            time.sleep(10)
            return {}
        file = Path("log.txt")
        if file.exists() and file.stat().st_size > 10:
            kill(process)
            return retrieve_stats()
        time.sleep(1)

def dump_metrics_graphics(stats, prefix=""):
    # stabilize some parameters
    target_velocity = 0.4
    ideal_distance = 0.5
    k_ang = 8.0
    k_lin = 1.0

    # 2d bar plot with initial position as x
    filtered_stats = [stat for stat in stats if stat["target_velocity"] == target_velocity and stat["ideal_distance"]
                      == ideal_distance and stat["k_ang"] == k_ang and stat["k_lin"] == k_lin]
    for metric in ["travelled", "mean_distance_to_wall", "elapsed"]:
        plt.figure()
        for invert_direction in [True, False]:
            filtered_stats_invert = [stat for stat in filtered_stats if stat["invert_direction"] == invert_direction]
            xs = [str((stat["offset_x"] + INITIAL_POSITION[0], stat["offset_y"] + INITIAL_POSITION[1])) for stat in filtered_stats_invert]
            ys = [stat[metric] for stat in filtered_stats_invert]
            plt.scatter(xs, ys, label=f"{metric} (invert={invert_direction})", c="red" if invert_direction else "blue")
            plt.legend()
        plt.savefig(f"results/{prefix}_initialpos_{metric}.png")
        plt.close()

    # 3d plot with k_ang as x, k_lin as y and mean_distance_to_wall as z
    filtered_stats = [stat for stat in stats if stat["target_velocity"] == target_velocity and stat["ideal_distance"]
                      == ideal_distance and stat["offset_x"] == 0 and stat["offset_y"] == -1]

    plt.figure()
    plt.title("mean_distance_to_wall according to k_ang and k_lin")
    xs = [stat["k_ang"] for stat in filtered_stats]
    ys = [stat["k_lin"] for stat in filtered_stats]
    zs = [stat["mean_distance_to_wall"] for stat in filtered_stats]
    ax = plt.axes(projection='3d')
    ax.scatter(xs, ys, zs)
    ax.set_xlabel("k_ang")
    ax.set_ylabel("k_lin")
    ax.set_zlabel("mean_distance_to_wall")
    plt.savefig(f"results/{prefix}_kangklin.png")
    plt.close()

    # 2d bar plot with ideal_distance as x and mean_distance_to_wall as y
    filtered_stats = [stat for stat in stats if stat["target_velocity"] == target_velocity
                      and stat["offset_x"] == -2 and stat["offset_y"] == -3
                      and stat["k_ang"] == k_ang and stat["k_lin"] == k_lin]
    plt.figure()
    plt.title("mean_distance_to_wall according to ideal_distance")
    xs = [stat["ideal_distance"] for stat in filtered_stats]
    ys = [stat["mean_distance_to_wall"] for stat in filtered_stats]
    plt.bar(xs, ys)
    plt.xlabel("ideal_distance")
    plt.ylabel("mean_distance_to_wall")
    plt.savefig(f"results/{prefix}_idealdistance.png")

    # 2d bar plot with target_velocity as x and mean_distance_to_wall as y
    filtered_stats = [stat for stat in stats if stat["ideal_distance"] == ideal_distance
                        and stat["offset_x"] == -2 and stat["offset_y"] == -3
                        and stat["k_ang"] == k_ang and stat["k_lin"] == k_lin]
    plt.figure()
    plt.title("mean_distance_to_wall according to target_velocity")
    xs = [stat["target_velocity"] for stat in filtered_stats]
    ys = [stat["mean_distance_to_wall"] for stat in filtered_stats]
    plt.bar(xs, ys)
    plt.xlabel("target_velocity")
    plt.ylabel("mean_distance_to_wall")
    plt.savefig(f"results/{prefix}_targetvelocity.png")


def dump_distance_to_wall_graphics(stats, prefix=""):
    for target_velocity in [0.4, 0.6]:
        for ideal_distance in [0.5, 0.7]:
            k_ang = 8.0
            k_lin = 1.0
            name = f"timeseries_target_velocity={target_velocity}_ideal_distance={ideal_distance}_k_ang={k_ang}_k_lin={k_lin}"
            plt.figure()
            plt.title("Distance to wall over time")
            plt.xlabel("time")
            plt.ylabel("distance to wall")

            filtered_stats = [stat for stat in stats if stat["target_velocity"] == target_velocity and stat["ideal_distance"]
                              == ideal_distance and stat["k_ang"] == k_ang and stat["k_lin"] == k_lin]

            if filtered_stats == []:
                print(f"warning: no stats for {prefix}_{name}")
                continue

            stat = filtered_stats[0]
            distances = stat["distances"]
            x = np.arange(0, len(distances), 1)
            plt.plot(x, distances)
            plt.savefig(f"results/{prefix}_{name}.png")
            plt.close()


async def main():
    stats = []

    # remove all pngs inside results folder
    for file in os.listdir("results"):
        if file.endswith(".png"):
            os.remove(os.path.join("results", file))

    # if READ environment variable is set, read stats from file
    if os.environ.get("READ") is not None:
        with open("stats.txt") as f:
            # read entire file and remove \n
            content = f.read()
            objects = content.split("}\n")
            for obj in objects:
                if obj == "":
                    continue
                # replace "array" by "np.array"
                obj = obj.replace("array", "np.array")
                if obj[-1] != "}":
                    obj += "}"
                stats.append(eval(obj))
    else:
        # run simulations
        print("running simulations... stats file will be overwritten")
        time.sleep(5)
        with open("stats.txt", "w") as f:
            for old_controller in [True, False]:
                for offset_x in [0.0,2.0]:
                    for offset_y in [-1.0,0.0]:
                        for invert_direction in [True, False]:
                            for target_velocity in [0.4, 0.6]:
                                for ideal_distance in [0.5, 0.7]:
                                    for k_ang in [8.0, 16.0]:
                                        for k_lin in [1.0, 2.0]:
                                            stat = await run_simulation_and_read_log(INITIAL_POSITION[0] + offset_x/2,
                                                                                    INITIAL_POSITION[1] +
                                                                                    offset_y/2, target_velocity,
                                                                                    ideal_distance, k_ang, k_lin,
                                                                                    invert_direction, old_controller)
                                            stat["offset_x"] = offset_x
                                            stat["offset_y"] = offset_y
                                            stat["target_velocity"] = target_velocity
                                            stat["ideal_distance"] = ideal_distance
                                            stat["invert_direction"] = invert_direction
                                            stat["k_ang"] = k_ang
                                            stat["k_lin"] = k_lin
                                            stat["old_controller"] = old_controller
                                            stats.append(stat)
                                            f.write(str(stat) + "\n")
                                            f.flush()
                if old_controller:
                    break # only run old controller once
        f.close()

    for old_controller in [True, False]:
        prefix = "oldc" if old_controller else "newc"
        filtered_stats = [stat for stat in stats if stat["old_controller"] == old_controller]
        dump_distance_to_wall_graphics(filtered_stats, prefix)
        dump_metrics_graphics(filtered_stats, prefix)

if __name__ == "__main__":
    asyncio.run(main())

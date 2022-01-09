#!/usr/bin/python

import argparse
import os
import subprocess
import sys

parser = argparse.ArgumentParser(description="Wrapper script for running the Gazebo simulator with a biped model.")

parser.add_argument("--laptop", "-l", help="Adjust CPU pinning to fit laptop CPU topology.", action="store_true")
parser.add_argument("--paused", "--pause", "-p", help="Start the simulation paused", action="store_true")
parser.add_argument("--skip-pinning", help="Run gazebo without pinning to specific CPU's. May be useful when running with insufficient permissions for chrt.", action="store_true")

parser.add_argument("--real-time-factor", "--rtf", type=float, default=1.0, help="Run the simulation with the specified real-time factor.")
parser.add_argument('--runtime-limit', '--rtl', type=int, default=-1, help='Set runtime limit. This makes the simulation exit after the specified time has elapsed.')
parser.add_argument("--physics-engine", "--engine", type=str, default="bullet", help="Run the simulation with the specified physics engine.")

args = parser.parse_args()

print("Args:", args, "\n")

# Build gazebo plugin, exit if failed.
if(os.WEXITSTATUS(os.system("./build_plugins")) != 0):
    print("Plugin Build exited with a nonzero error code, not running sim.")
    sys.exit()

# Create temporary world file to patch real-time factor
temp_world_file_path = subprocess.run("mktemp -t biped_adjusted_rtf_XXXX.world", shell=True, capture_output=True, check=True).stdout.decode().replace("\n", "")
print("Temp world file path:", temp_world_file_path, "\n")

# Patch real-time factor
world_file_str = open(os.getcwd() + "/simplified_biped.world", "r").read()
world_file_str.replace("<max_step_size>0.001</max_step_size>", f"<max_step_size>{1.0/1000.0*args.real_time_factor}</max_step_size>")
world_file_str.replace("<real_time_factor>1</real_time_factor>", f"<real_time_factor>{args.real_time_factor}</real_time_factor>")


temp_world_file = open(temp_world_file_path, "w")
temp_world_file.write(world_file_str)
temp_world_file.close()

# Because {} brackets mess up formatting string in python
LD_LIBRARY_PATH_str = "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"

pinning_str = "8-11,20-23"

if args.laptop:
    pinning_str = "4-5,10-11"

command = f"{LD_LIBRARY_PATH_str}:{os.getcwd()}/control_plugin/build/ chrt -r 1 taskset -c {pinning_str} timeout {args.runtime_limit} gazebo -e {args.physics_engine} --verbose -r --record_encoding zlib --record_path {os.getcwd()} {temp_world_file_path} ; killall gzserver ; killall gazebo ; killall gzclient"

if args.runtime_limit == -1:
    command = command.replace(f" timeout {args.runtime_limit}", "")

if args.paused:
    # Add -u
    command = command.replace("gazebo", "gazebo -u")

if args.skip_pinning:
    # Remove pinning commands
    command = command.replace(f"chrt -r 1 taskset -c {pinning_str}", "")

print("Final command:", command, "\n")

os.system(command)

plot_data_dir = os.environ["HOME"] + "/dev/walking_controller/plot_data/"

largest_index = 0

for name in os.listdir(plot_data_dir):
    try:
        index = int(name.split('.')[0].replace('_left', '').replace('_right', ''))
        if index > largest_index:
            largest_index = index
    except:
        pass

print("Largest index:", largest_index)
print(f"Copying recorded state to \"{plot_data_dir}/{largest_index}_state.log\"...\n")
os.system(f"pv state.log > {plot_data_dir}/{largest_index}_state.log")

print("Done.")

# sim_process = subprocess.Popen(f"{LD_LIBRARY_PATH_str}:{os.getcwd()}/control_plugin/build/ chrt -r 1 taskset -c {pinning_str} gazebo -e bullet {temp_world_file_path}", cwd=os.getcwd(), shell=True, stdout=subprocess.PIPE)
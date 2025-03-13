import os
import datetime
import shutil
from SCons.Script import DefaultEnvironment
import subprocess

env = DefaultEnvironment()

def copy_hex_file(source, target, env):
    def get_git_commit():
        try:
            commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).strip().decode("utf-8")
            return commit
        except subprocess.CalledProcessError:
            return "unknown"

    git_commit = get_git_commit()
    build_dir = env.subst("$BUILD_DIR")
    build_env = env.subst("$PIOENV")
    hex_file = os.path.join(build_dir, "firmware.hex")
    dest_dir = os.path.join(env.subst("$PROJECT_DIR"), "hex")
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
    build_date = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
    dest_file = os.path.join(dest_dir, f"{build_env}-{build_date}-{git_commit}.hex")

    if os.path.exists(hex_file):
        shutil.copy(hex_file, dest_file)
        print(f"Copied {hex_file} to {dest_file}")
    else:
        print(f"Hex file {hex_file} not found")

env.AddPostAction("buildprog", copy_hex_file)
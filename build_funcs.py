import subprocess
def get_git_commit():

# Run git describe command to get version string
    try:
        # Use 'git describe --tags --always --dirty' for comprehensive version info
        version_str = subprocess.check_output(["git", "describe", "--tags", "--always", "--dirty", "--match", "v*"], cwd=".").strip().decode("utf-8")
    except (subprocess.CalledProcessError, OSError):
        version_str = "unknown"

    return version_str


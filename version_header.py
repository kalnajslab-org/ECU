import subprocess
import os
from build_funcs import get_git_commit

version_file_name = "ecu_version.h"
header_define = "ECU_VERSION"

# Define the path for the generated header file
header_file_path = os.path.join("src", version_file_name)

version_str = get_git_commit()

# Ensure 'src' directory exists
os.makedirs(os.path.dirname(header_file_path), exist_ok=True)

# Write the version string to the version.h file as a preprocessor macro
with open(header_file_path, "w") as f:
    f.write(f'#ifndef {header_define}_H\n')
    f.write(f'#define {header_define}_H\n')
    f.write(f'#define {header_define} "{version_str}"\n')
    f.write(f'#endif // {header_define}_H\n')

print(f"Generated {header_file_path}: #define {header_define} \"{version_str}\"")

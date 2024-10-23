import os
from pathlib import Path
from glob import glob
import shutil
cwd = Path(os.getcwd())
binary_path = os.path.join(os.getcwd(), "kis.bin")
fw_path = os.path.join(cwd.parents[1],"kis_service", "kis_service", "robot", "fw_binary")
header_path = os.path.join(cwd.parents[0], "Core", "Inc", "version.h")

with open(header_path, 'r') as f:
    lines = f.readlines()

major = None
minor = None
for line in lines:
    if "MAJOR" in line:
        major = line.split(' ')[-1].strip()
    if "MINOR" in line:
        minor = line.split(' ')[-1].strip()

print("Cleaning binary directory" )
bins_in_path = glob("*.bin", root_dir=fw_path)
for bin in bins_in_path:
    os.remove(os.path.join(fw_path, bin))

print("Moving versioned binary")
shutil.copy(binary_path, os.path.join(fw_path, f"kis2_fw_{major}.{minor}.bin"))

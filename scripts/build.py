"""Configure and stage the cFS robot mission on Linux (including Pi arm64)."""
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
if os.name == "nt":
    raise SystemExit("cFS runtime requires Linux: run on the Pi or in WSL/Linux. Portable tests work on Windows.")
vendor = root / "vendor/cFS"
defs = root / "build/robot_defs"
build = root / "build/cfs"
shutil.copytree(vendor / "sample_defs", defs, dirs_exist_ok=True)
for source, target in [("targets.cmake", "targets.cmake"),
                       ("install_custom.cmake", "cpu1/install_custom.cmake"),
                       ("generate_startup.cmake", "generate_startup.cmake")]:
    shutil.copyfile(root / "mission" / source, defs / target)
env = os.environ.copy()
env["CFS_APP_PATH"] = str(root / "apps")
subprocess.run(["cmake", "-S", str(vendor / "cfe"), "-B", str(build),
                "-G", "Unix Makefiles", "-DSIMULATION=native",
                "-DMISSIONCONFIG=robot", "-DMISSION_DEFS=" + str(defs),
                "-DCFE_EDS_ENABLED=OFF", "-DENABLE_UNIT_TESTS=OFF",
                "-DCMAKE_BUILD_TYPE=Debug", "-DCMAKE_INSTALL_PREFIX=" + str(build / "exe")],
               env=env, check=True)
subprocess.run(["cmake", "--build", str(build), "--target", "mission-install",
                "--parallel", os.environ.get("BUILD_JOBS", "2")], env=env, check=True)

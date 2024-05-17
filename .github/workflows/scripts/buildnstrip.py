from pathlib import Path
import os
from subprocess import run
import argparse

blacklist = ["MAX32572"]

"""
Build peripheral drivers for all micros, then strip the debug symbols from the static library files.
"""
def buildnstrip(maxim_path : Path = None):
    env = os.environ.copy()
    if maxim_path is None and "MAXIM_PATH" in env.keys():
        maxim_path = Path(env['MAXIM_PATH']).absolute()
    else:
        print("MAXIM_PATH not set.")
        return
    
    targets = []
    for dir in os.scandir(f"{maxim_path}/Examples"):
        if dir.name not in blacklist:
            targets.append(dir.name)

    # Build
    for t in targets:
        p = maxim_path / "Examples" / t / "Hello_World"
        if p.exists():
            run(f"make -C {p} distclean", env=env, shell=True, check=True)
            for float_abi in ["soft", "softfp", "hard"]:
                run(f"make -C {p} clean", env=env, shell=True, check=True)
                run(f"make -C {p} -r -j --output-sync=target --no-print-directory DEBUG=0 MFLOAT_ABI={float_abi}", env=env, shell=True, check=True)

    # Strip
    for f in (maxim_path / "Libraries" / "PeriphDrivers").rglob("*.a"):
        print(f"Stripping {f}")
        run(f"arm-none-eabi-strip --strip-debug {f}", env=env, shell=True, check=True)

parser = argparse.ArgumentParser("Build and Strip MSDK utility script")
parser.add_argument("--maxim_path", type=str, help="(Optional) Location of the MaximSDK.  If this is not specified then the script will attempt to use the MAXIM_PATH environment variable.")

if __name__ == "__main__":
    args = parser.parse_args()
    buildnstrip(maxim_path=args.maxim_path)
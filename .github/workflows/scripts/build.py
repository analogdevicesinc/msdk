from pathlib import Path
import os
from subprocess import run

blacklist = [
    "MAX32570",
    "MAX32572",
    "BCB", 
    "ROM", 
    "Simulation", 
    "BCB_PBM", 
    "Emulator", 
    "Emulator_NFC", 
    "EvKit_129B", 
    "EvKit_129C"
    ]

def test(maxim_path : Path = None, targets=None, boards=None, projects=None):
    env = os.environ.copy()
    if maxim_path is None and "MAXIM_PATH" in env.keys():
        maxim_path = Path(env['MAXIM_PATH']).absolute()
    else:
        print("MAXIM_PATH not set.")
        return

    # Simulate the VS Code terminal by appending to the Path
    # if curplatform == 'Linux':
    #     env["PATH"] = f"{maxim_path.as_posix()}/Tools/GNUTools/10.3/bin:{maxim_path.as_posix()}/Tools/xPack/riscv-none-embed-gcc/10.2.0-1.2/bin:" + env["PATH"]
    # elif curplatform == 'Windows':
    #     env["PATH"] = f"{maxim_path.as_posix()}/Tools/GNUTools/10.3/bin;{maxim_path.as_posix()}/Tools/xPack/riscv-none-embed-gcc/10.2.0-1.2/bin;{maxim_path.as_posix()}/Tools/MSYS2/usr/bin;" + env["PATH"]

    # Get list of target micros if none is specified
    if targets is None:
        targets = []

        for dir in os.scandir(f"{maxim_path}/Examples"):
            if dir.name not in blacklist:
                targets.append(dir.name) # Append subdirectories of Examples to list of target micros

        print(f"Detected target microcontrollers: {targets}")
    
    else:
        assert(type(targets) is list)
        print(f"Testing {targets}")

    # Enforce alphabetical ordering
    targets = sorted(targets)

    # Track failed projects for end summary
    failed = []
    count = 0

    for target in targets:

        # Get list of supported boards for this target.
        if boards is None:
            boards = []
            for dirpath, subdirs, items in os.walk(maxim_path / "Libraries" / "Boards" / target):
                if "board.mk" in items and Path(dirpath).name not in blacklist:
                    boards.append(Path(dirpath).name)

        else:
            assert(type(boards) is list)
            print(f"Testing {boards}")

        boards = sorted(boards) # Enforce alphabetical ordering
                
        # Get list of examples for this target.  If a Makefile is in the root directory it's an example.
        if projects is None:
            projects = []
            for dirpath, subdirs, items in os.walk(maxim_path / "Examples" / target):
                if 'Makefile' in items and ("main.c" in items or "project.mk" in items):
                    projects.append(Path(dirpath))

        else:
            assert(type(projects) is list)

        print("====================")
        print(f"Testing {len(projects)} projects for {target}")
        print(f"Detected boards: {boards}")
        print("====================")

        projects = sorted(projects) # Enforce alphabetical ordering

        for project in projects:
            project_name = project.name

            for board in boards:
                print(f"TARGET: {target}")
                print(f"BOARD: {board}")
                print(f"PROJECT: {project_name}")

                res = run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

                # Test build (make all)
                build_cmd = f"make -r -j 8 --output-sync=target --no-print-directory TARGET={target} MAXIM_PATH={maxim_path.as_posix()} BOARD={board} FORCE_COLOR=1"
                res = run(build_cmd, cwd=project, shell=True, capture_output=True, encoding="utf-8")

                # Error check build command
                if res.returncode != 0:
                    print("Failed.")
                    print(f"Build command: {build_cmd}")
                    print("Error:")
                    print(res.stderr)
                    if project not in failed:
                        failed.append(project)

                else:
                    print("Success!")

                print("------------------")
                res = run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

                count += 1

        boards = None # Reset boards list
        projects = None # Reset projects list

    print(f"Tested {count} projects.  {count - len(failed)}/{count} succeeded.")
    if (len(failed) > 0):
        print("Failed projects:")
        for p in failed:
            print(p)

        return -1
    else:
        return 0

if __name__ == "__main__":
    exit(test())

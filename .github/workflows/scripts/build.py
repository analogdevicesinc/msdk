from pathlib import Path
import os
from subprocess import run
import argparse

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
    "EvKit_129C",
    "WLP_VAR",
    "WLP_DB",
    "TQFN_DB"
    ]

def test(maxim_path : Path = None, targets=None, boards=None, projects=None):
    env = os.environ.copy()
    if maxim_path is None and "MAXIM_PATH" in env.keys():
        maxim_path = Path(env['MAXIM_PATH']).absolute()
    else:
        print("MAXIM_PATH not set.")
        return

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
                    project_info = {
                        "target":target,
                        "project":project_name,
                        "board":board,
                        "path":project,
                        "stdout":res.stdout,
                        "stderr":res.stderr
                    }
                    if project_info not in failed:
                        failed.append(project_info)

                else:
                    print("Success!")

                print("------------------", flush=True)
                res = run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

                count += 1

        boards = None # Reset boards list
        projects = None # Reset projects list

    print(f"Tested {count} cases.  {count - len(failed)}/{count} succeeded.")
    if (len(failed) > 0):
        print("Failed projects:")
        for p in failed:
            print(f"{p['target']}: {p['project']} failed for {p['board']}")

        return -1
    else:
        return 0

parser = argparse.ArgumentParser("MSDK Build Test Script")
parser.add_argument("--maxim_path", type=str, help="(Optional) Location of the MaximSDK.  If this is not specified then the script will attempt to use the MAXIM_PATH environment variable.")
parser.add_argument("--targets", type=str, nargs="+", required=False, help="Target microcontrollers to test.")
parser.add_argument("--boards", type=str, nargs="+", required=False, help="Boards to test.  Should match the BSP folder-name exactly.")
parser.add_argument("--projects", type=str, nargs="+", required=False, help="Examples to populate.  Should match the example's folder name.")

if __name__ == "__main__":
    args = parser.parse_args()
    exit(
        test(
            maxim_path=args.maxim_path,
            targets=args.targets,
            boards=args.boards,
            projects=args.projects
        )
    )

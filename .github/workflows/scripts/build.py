from pathlib import Path
import os
from subprocess import run
import argparse
from rich.progress import Progress
from rich.console import Console
import time

blacklist = [
    "MAX32570",
    "MAX32572",
    "MAXREFDES178",
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
    "TQFN_DB",
    "WLP_V1"
]

known_errors = [
    "ERR_NOTSUPPORTED",
    "ERR_LIBNOTFOUND"
]

def test(maxim_path : Path = None, targets=None, boards=None, projects=None):
    env = os.environ.copy()
    if maxim_path is None and "MAXIM_PATH" in env.keys():
        maxim_path = Path(env['MAXIM_PATH']).absolute()
    else:
        print("MAXIM_PATH not set.")
        return

    env["FORCE_COLOR"] = 1

    console = Console(emoji=False)

    # Get list of target micros if none is specified
    if targets is None:
        targets = []

        for dir in os.scandir(f"{maxim_path}/Examples"):
            if dir.name not in blacklist:
                targets.append(dir.name) # Append subdirectories of Examples to list of target micros

        console.print(f"Detected target microcontrollers: {targets}")
    
    else:
        assert(type(targets) is list)
        console.print(f"Testing {targets}")

    # Enforce alphabetical ordering
    targets = sorted(targets)

    # Track failed projects for end summary
    failed = []
    count = 0

    for target in sorted(targets):

        target_fails = 0

        # Get list of supported boards for this target.
        if boards is None:
            boards = []
            for dirpath, subdirs, items in os.walk(maxim_path / "Libraries" / "Boards" / target):
                if "board.mk" in items and Path(dirpath).name not in blacklist:
                    boards.append(Path(dirpath).name)

        else:
            assert(type(boards) is list)
            console.print(f"Testing {boards}")

        boards = sorted(boards) # Enforce alphabetical ordering
                
        # Get list of examples for this target.  If a Makefile is in the root directory it's an example.
        if projects is None:
            projects = []
            for dirpath, subdirs, items in os.walk(maxim_path / "Examples" / target):
                if 'Makefile' in items and ("main.c" in items or "project.mk" in items):
                    projects.append(Path(dirpath))

        else:
            assert(type(projects) is list)

        console.print("====================")
        console.print(f"Found {len(projects)} projects for {target}")
        console.print(f"Detected boards: {boards}")

        projects = sorted(projects) # Enforce alphabetical ordering

        with Progress(console=console) as progress:
            task_build = progress.add_task(description=f"{target}: {projects[0].name}", total=(len(projects) * len(boards)))

            for project in projects:
                project_name = project.name

                for board in boards:
                    res = run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

                    # Test build (make all)
                    build_cmd = f"make -r -j 8 --output-sync=target --no-print-directory TARGET={target} MAXIM_PATH={maxim_path.as_posix()} BOARD={board}"
                    res = run(build_cmd, cwd=project, shell=True, capture_output=True, encoding="utf-8")

                    # Error check build command
                    if res.returncode != 0:
                        fail = True
                        for err in known_errors:
                            if err in res.stderr:
                                fail = False
                                console.print(f"[yellow]{target} {project_name}: Known error for {board}[/yellow]")
                                console.print(res.stderr, markup=False)
                                print("--------------------")
                                break
                        if fail:
                            console.print(f"[red]{target} {project_name}: Failed for {board}[/red]")
                            print(f"Build command: {build_cmd}")
                            print("Error:")
                            console.print(res.stderr, markup=False)
                            project_info = {
                                "target":target,
                                "project":project_name,
                                "board":board,
                                "path":project,
                                "stdout":res.stdout,
                                "stderr":res.stderr
                            }
                            console.print("--------------------")
                            if project_info not in failed:
                                failed.append(project_info)
                                target_fails += 1

                    res = run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

                    count += 1
                    progress.update(task_build, advance=1, description=f"{target}: {project_name}", refresh=True)

            if target_fails == 0:
                progress.update(task_build, description=f"{target}: [green]Pass.[/green]", refresh=True)
            else:
                progress.update(task_build, description=f"{target}: [red]Failed {target_fails}/{len(projects)} test cases.[/red]", refresh=True)

        boards = None # Reset boards list
        projects = None # Reset projects list

    console.print(f"Tested {count} cases.  {count - len(failed)}/{count} succeeded.")
    if (len(failed) > 0):
        print("Failed projects:")
        for p in failed:
            console.print(f"{p['target']}: {p['project']} failed for {p['board']}")

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

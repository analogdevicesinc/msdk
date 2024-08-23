
from pathlib import Path, PurePath
import os
from subprocess import run
import argparse
from typing import Tuple
from rich.progress import Progress
from rich.console import Console
from rich.text import Text
from rich import inspect
import time
import shutil

blacklist = [
    "MAX32570",
    "MAX32572",
    "MAX32657",
    "MAX32665/BLE_LR_Central",
    "MAX32665/BLE_LR_Peripheral",
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
project_blacklist = {
    "BLE_LR_Central",
    "BLE_LR_Peripheral",
}

known_errors = [
    "ERR_NOTSUPPORTED",
    "ERR_LIBNOTFOUND",
    "[WARNING] - This tool does not handle keys in a PCI-PTS compliant way, only for test"
]

hardfp_test_list = [
    "Hello_World",
    "BLE_periph",
    "BLE_datc",
    "BLE_dats",
    "BLE_FreeRTOS"
]

console = Console(emoji=False, color_system="standard")

def build_project(project:Path, target, board, maxim_path:Path, distclean=False, extra_args=None) -> Tuple[int, tuple]:
    clean_cmd = "make clean" if not distclean else "make distclean"
    # NOTE: Disabled Cordio re-builds as of 8/23/2024 now that the library files should
    # account for basic changes across projects like trace levels and hard/softfp
    # if "Bluetooth" in project.as_posix() or "BLE" in project.as_posix():
    #     # Clean cordio lib for BLE projects
    #     clean_cmd += "&& make clean.cordio"
    res = run(clean_cmd, cwd=project, shell=True, capture_output=True, encoding="utf-8")

    # Test build
    build_cmd = f"make -r -j 8 TARGET={target} MAXIM_PATH={maxim_path.as_posix()} BOARD={board} FORCE_COLOR=1"
    if extra_args:
        build_cmd += f" {str(extra_args)}"
    res = run(build_cmd, cwd=project, shell=True, capture_output=True, encoding="utf-8")

    project_info = {
        "target":target,
        "project":project.name,
        "board":board,
        "path":project,
        "build_cmd":build_cmd,
        "stdout":res.stdout,
        "stderr":res.stderr
    }

    # Error check build command
    fail = (res.returncode != 0)
    warning = False
    known_error = False
    if res.stderr != None:
        for err in known_errors:
            if err in res.stderr:
                known_error = True
            elif err in res.stdout:
                # This case catches the output of the SBTs, which will print a warning
                # to stdout.  For these warnings, stderr is non-null but empty
                if res.stderr == '':
                    known_error = True
                    

    if fail and known_error: # build error
        fail = False
    elif res.stderr != None and res.stderr != '' and not known_error: # Build passed but with warnings
        warning = True

    # Clean before returning
    run("make clean", cwd=project, shell=True, capture_output=True, encoding="utf-8")

    return_code = 0
    if fail:
        return_code = 1
    elif warning:
        return_code = 2

    return (return_code, project_info)

def query_build_variable(project:Path, variable:str) -> list:
    result = run(f"make query QUERY_VAR=\"{variable}\"", cwd=project, shell=True, capture_output=True, encoding="utf-8")
    if result.returncode != 0:
        return []

    output = []
    for v in variable.split(" "):
        for line in result.stdout.splitlines():
            if v in line:
                # query output string will be "{variable}={item1, item2, ..., itemN}"
                output += str(line).split("=")[1].split(" ")

    return output

"""
Create a dictionary mapping each target micro to its dependencies in the MSDK.
The dependency paths contain IPATH, VPATH, SRCS, and LIBS from all of the target's examples.

It should be noted that any paths relative to the Libraries folder will be walked back to a
1-level deep sub-folder.

Ex: Libraries/PeriphDrivers/Source/ADC/adc_me14.c   --->   Libraries/PeriphDrivers

This is so that any changes to source files will be caught by the dependency tracker.
TODO: Improve this so the dependency checks are more granular.  (i.e. ME14 source change only 
    affects ME14).  This is difficult because example projects do not have direct visibility 
    into library SRCS/VPATH for libraries that build a static file.

"""
def create_dependency_map(maxim_path:Path, targets:list) -> dict:
    dependency_map = dict()

    with Progress(console=console) as progress:
        task_dependency_map = progress.add_task("Creating dependency map...", total=len(targets))
        for target in targets:
            progress.update(task_dependency_map, description=f"Creating dependency map for {target}...")
            dependency_map[target] = []
            examples_dir = Path(maxim_path / "Examples" / target)
            if examples_dir.exists():
                projects = [Path(i).parent for i in examples_dir.rglob("**/project.mk")]
                for project in projects:
                    console.print(f"\t- Checking dependencies: {project}")
                    dependencies = query_build_variable(project, "IPATH VPATH SRCS LIBS")
                    dependencies = list(set(dependencies))
                    for i in dependencies:
                        if i == ".":
                            dependencies.remove(i)
                            i = project

                        # Convert to absolute paths
                        if not Path(i).is_absolute():
                            dependencies.remove(i)
                            corrected = Path(Path(project) / i).absolute()
                            i = corrected

                        # Walk back library paths to their root library folder.
                        # This is so that any src changes will get caught, since
                        # usually the project does not have the src folders as direct
                        # dependencies on VPATH/SRCS.  IPATH will be exposed to the project.
                        if "Libraries" in str(i):
                            path = Path(i)
                            while path.parent.stem != "Libraries" and path.exists():
                                path = path.parent
                            i = str(path)

                        if i not in dependency_map[target]:
                            dependency_map[target].append(str(i))


            if "." in dependency_map[target]: 
                dependency_map[target].remove(".") # Root project dir
            if str(maxim_path) in dependency_map[target]:
                dependency_map[target].remove(str(maxim_path)) # maxim_path gets added for "mxc_version.h"

            dependency_map[target] = sorted(list(set(dependency_map[target])))
            # console.print(f"\t- {target} dependencies:\n{dependency_map[target]}")
            progress.update(task_dependency_map, advance=1)

    return dependency_map

def get_affected_targets(dependency_map: dict, file: Path) -> list:
    file = Path(file)
    affected = []
    for target in dependency_map.keys():
        add = False
        if target in str(file).upper(): add = True

        for dependency in dependency_map[target]:
            if file.is_relative_to(dependency): add = True

        if add and target not in affected: affected.append(target)

    return affected

def test(maxim_path : Path = None, targets=None, boards=None, projects=None, change_file=None):
    env = os.environ.copy()
    if maxim_path is None and "MAXIM_PATH" in env.keys():
        maxim_path = Path(env['MAXIM_PATH']).absolute()
        console.print(f"[green]Detected MAXIM_PATH[/green] = {maxim_path}")
    else:
        console.print("MAXIM_PATH not set.")
        return
    
    env["FORCE_COLOR"] = 1

    console.print(f"Blacklist: {blacklist}")
    console.print(f"Project blacklist: {project_blacklist}")
    console.print(f"Known errors: {known_errors}")
    console.print(f"Hard FP whitelist: {hardfp_test_list}")

    # Remove the periphdrivers build directory
    console.print("Cleaning PeriphDrivers build directories...")
    shutil.rmtree(Path(maxim_path) / "Libraries" / "PeriphDrivers" / "bin", ignore_errors=True)

    # Get list of target micros if none is specified
    if targets is None:
        console.print("[yellow]Auto-searching for targets...[/yellow]")
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

    if (args.change_file is not None):
        console.print(f"Reading '{args.change_file}'")
        targets_to_skip = []
        for i in targets: targets_to_skip.append(i)
        files:list = []
        with open(args.change_file, "r") as change_file:
            files = change_file.read().strip().replace(" ", "\n").splitlines()
            files = [maxim_path / file for file in files]

        if not files:
            console.print("[red]Changed files is empty.  Skipping dependency checks.[/red]")
        else:
            console.print("Creating dependency map...")
            dependency_map = create_dependency_map(maxim_path, targets)
            console.print(f"Checking {len(files)} changed files...")

            for f in files:
                affected = get_affected_targets(dependency_map, f)
                if affected:
                    for i in affected:
                        if i in targets_to_skip:
                            targets_to_skip.remove(i)
                            console.print(f"\t- Testing {i} from change to {f}")
                else:
                    console.print(f"\t- Unknown effects from change to {f}, testing everything")
                    targets_to_skip.clear()

                if len(targets_to_skip) == 0: break

            targets = [i for i in targets if i not in targets_to_skip]

    if targets is not None:
        console.print(f"Testing: {targets}")
    else:
        console.print("Nothing to be tested.")

    # Track failed projects for end summary
    failed = []
    warnings = []
    count = 0

    for target in sorted(targets):

        console.print("====================")
        console.print(f"Testing {target}...")

        target_fails = 0
        target_warnings = 0

        # Get list of supported boards for this target.
        if boards is None:
            console.print(f"[yellow]Auto-searching for {target} BSPs...[/yellow]")
            boards = []
            for dirpath, subdirs, items in os.walk(maxim_path / "Libraries" / "Boards" / target):
                if "board.mk" in items and Path(dirpath).name not in blacklist:
                    boards.append(Path(dirpath).name)

        else:
            assert(type(boards) is list)
            console.print(f"Testing {boards}")

        boards = sorted(boards) # Enforce alphabetical ordering
                
        # Get list of examples for this target.
        _projects = set()
        if projects is None:
            console.print(f"[yellow]Auto-searching for {target} examples...[/yellow]")
            for dirpath, subdirs, items in os.walk(maxim_path / "Examples" / target):
                if 'Makefile' in items and ("main.c" in items or "project.mk" in items) and PurePath(dirpath).name not in project_blacklist:
                    _projects.add(Path(dirpath))

        else:
            assert(type(projects) is list)
            for dirpath, subdirs, items in os.walk(maxim_path / "Examples" / target):
                dirpath = Path(dirpath)
                if dirpath.name in projects:
                    _projects.add(dirpath)
        
        
        
        console.print(f"Found {len(_projects)} projects for [bold cyan]{target}[/bold cyan]")
        console.print(f"Detected boards: {boards}")

        _projects = sorted(_projects) # Enforce alphabetical ordering
                

        with Progress(console=console) as progress:
            task_build = progress.add_task(description=f"{target}: PeriphDrivers", total=(len(_projects) * len(boards)) + len(boards))

            periph_success = True

            # Find Hello_World and do a PeriphDriver build test first.
            hello_world = None
            for p in _projects:
                if p.name == "Hello_World":
                    hello_world = p
            
            if hello_world is None:
                console.print(f"[red]Failed to locate Hello_World for {target}[/red]")
            else:
                for board in boards:
                    progress.update(task_build, description=f"[bold cyan]{target}[/bold cyan] ({board}) PeriphDriver", refresh=True)
                    (return_code, project_info) = build_project(hello_world, target, board, maxim_path, distclean=True)
                    count += 1

                    # Error check build command
                    if return_code == 0:
                        progress.update(task_build, advance=1, description=f"[bold cyan]{target}[/bold cyan] ({board}): [green]PeriphDriver build pass.[/green]", refresh=True)
                    elif return_code == 1:
                        console.print(f"\n[red]{target} ({board}): PeriphDriver build failed.[/red]")
                        print(f"Build command: {project_info['build_cmd']}")
                        console.print("[bold]Errors:[/bold]")
                        console.print("[red]----------------------------------------[/red]")
                        console.print(Text.from_ansi(project_info['stderr']), markup=False)
                        console.print("[red]----------------------------------------[/red]\n")

                        if project_info not in failed:
                            failed.append(project_info)
                            target_fails += 1

                        periph_success = False
                        progress.update(task_build, advance=1, description=f"[bold cyan]{target}[/bold cyan] ({board}): [red]PeriphDriver build fail.[/red]", refresh=True)
                    elif return_code == 2:
                        console.print(f"\n[yellow]{target} ({board}): PeriphDriver built with warnings.[/yellow]")
                        print(f"Build command: {project_info['build_cmd']}")
                        console.print("[bold]Warnings:[/bold]")
                        console.print("[yellow]----------------------------------------[/yellow]")
                        console.print(Text.from_ansi(project_info['stderr']), markup=False)
                        console.print("[yellow]----------------------------------------[/yellow]\n")

                        if project_info not in warnings:
                            warnings.append(project_info)
                            target_warnings += 1

            if periph_success:
                # Iteratively across and test example projects
                for project in _projects:
                    project_name = project.name

                    for board in boards:
                        progress.update(task_build, advance=1, description=f"{target} ({board}): {project_name}", refresh=True)

                        (return_code, project_info) = build_project(project, target, board, maxim_path, distclean=False)

                        # Error check build command
                        if return_code == 1:                            
                            console.print(f"\n[red]{target} ({board}): {project_name} failed.[/red]")
                            print(f"Build command: {project_info['build_cmd']}")
                            console.print("[bold]Errors:[/bold]")
                            console.print("[red]----------------------------------------[/red]")
                            console.print(Text.from_ansi(project_info['stderr']), markup=False)
                            console.print("[red]----------------------------------------[/red]\n")

                            if project_info not in failed:
                                failed.append(project_info)
                                target_fails += 1

                        elif return_code == 2:
                            console.print(f"\n[yellow]{target} ({board}): {project_name} built with warnings.[/yellow]")
                            print(f"Build command: {project_info['build_cmd']}")
                            console.print("[bold]Warnings:[/bold]")
                            console.print("[yellow]----------------------------------------[/yellow]")
                            console.print(Text.from_ansi(project_info['stderr']), markup=False)
                            console.print("[yellow]----------------------------------------[/yellow]\n")

                            if project_info not in warnings:
                                warnings.append(project_info)
                                target_warnings += 1

                        count += 1

                    if project_name in hardfp_test_list:
                        console.print(f"[yellow]{target}: {project_name} found in hardfp test whitelist.[/yellow]")
                        progress.update(task_build, advance=1, description=f"{target} (hardfp): {project_name}", refresh=True)
                        (return_code, project_info) = build_project(project, target, board, maxim_path, distclean=False, extra_args="MFLOAT_ABI=hard")
                        project_info['project'] += " [italic](hardfp)[/italic]" # Add a string to differentiate this test

                        # Error check hardfp builds
                        if return_code == 1:
                            console.print(f"\n[red]{target}: {project_name} failed to build with hardware floating-point acceleration enabled.[/red]")
                            print(f"Build command: {project_info['build_cmd']}")
                            console.print("[bold]Errors:[/bold]")
                            console.print("[red]----------------------------------------[/red]")
                            console.print(Text.from_ansi(project_info['stderr']), markup=False)
                            console.print("[red]----------------------------------------[/red]\n")

                            if project_info not in failed:
                                failed.append(project_info)
                                target_fails += 1

                        elif return_code == 2:
                            console.print(f"\n[yellow]{target}: {project_name} built for hardware floating point acceleration, but with warnings.[/yellow]")
                            print(f"Build command: {project_info['build_cmd']}")
                            console.print("[bold]Warnings:[/bold]")
                            console.print("[yellow]----------------------------------------[/yellow]")
                            console.print(Text.from_ansi(project_info['stderr']), markup=False)
                            console.print("[yellow]----------------------------------------[/yellow]\n")

                            if project_info not in warnings:
                                warnings.append(project_info)
                                target_warnings += 1



            if target_warnings != 0:
                console.print(f"[bold cyan]{target}[/bold cyan]: [yellow]{target_warnings} projects built with warnings.[/yellow]")

            if target_fails == 0:
                progress.update(task_build, description=f"[bold cyan]{target}[/bold cyan]: [green]Pass.[/green]", refresh=True)
            elif not periph_success:
                progress.update(task_build, description=f"[bold cyan]{target}[/bold cyan]: [red]PeriphDriver build failed.[/red]", refresh=True)
            else:
                progress.update(task_build, description=f"[bold cyan]{target}[/bold cyan]: [red]Failed for {target_fails}/{len(_projects)} projects[/red]", refresh=True)            

        boards = None # Reset boards list
        _projects = None # Reset projects list

    console.print(f"Tested {count} cases.  {count - len(failed)}/{count} succeeded.")
    if (len(warnings) > 0):
        print(f"{len(warnings)} projects with warnings:")
        for p in warnings:
            console.print(f"[bold cyan]{p['target']}[/bold cyan]: [bold]{p['project']}[/bold] [yellow]warnings[/yellow] for [yellow]{p['board']}[/yellow]")
    
    if (len(failed) > 0):
        print("Failed projects:")
        for p in failed:
            console.print(f"[bold cyan]{p['target']}[/bold cyan]: [bold]{p['project']}[/bold] [red]failed[/red] for [yellow]{p['board']}[/yellow]")

        return -1
    else:
        console.print("[bold][green]Test pass.[/bold][/green]")
        return 0

parser = argparse.ArgumentParser("MSDK Build Test Script")
parser.add_argument("--maxim_path", type=str, help="(Optional) Location of the MaximSDK.  If this is not specified then the script will attempt to use the MAXIM_PATH environment variable.")
parser.add_argument("--targets", type=str, nargs="+", required=False, help="Target microcontrollers to test.")
parser.add_argument("--boards", type=str, nargs="+", required=False, help="Boards to test.  Should match the BSP folder-name exactly.")
parser.add_argument("--projects", type=str, nargs="+", required=False, help="Examples to populate.  Should match the example's folder name.")
parser.add_argument("--change_file", type=str, required=False, help="(Optional) Pass a text file containing a list of space-separated or new-line separated changed files.  The build script will intelligently adjust which parts it tests based on this list.")

if __name__ == "__main__":
    args = parser.parse_args()
    inspect(args, title="Script arguments:", )

    exit(
        test(
            maxim_path=args.maxim_path,
            targets=args.targets,
            boards=args.boards,
            projects=args.projects,
            change_file=args.change_file
        )
    )

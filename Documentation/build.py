from pathlib import Path
import shutil
from subprocess import run
from mkdocs.commands.build import build
from mkdocs.config import load_config

# Locate some directories relative to this file
here = Path(__file__).parent
repo = here.parent
periph_docs_dir = repo / "Libraries" / "PeriphDrivers" / "Documentation"

# Run Doxygen builds
for f in periph_docs_dir.glob("*_Doxyfile"):
    run(f"doxygen {f.name}", cwd=f.parent)
    micro = f.name.split("_")[0].upper()  # max32xxx_Doxyfile -> MAX32XXX
    dest = here / "Libraries" / "PeriphDrivers" / "Documentation" / micro
    # ^ Recreate directory structure so built links work
    shutil.copytree(periph_docs_dir / micro, dest, dirs_exist_ok=True)

# Pre-populate markdown files
for f in repo.glob("*.md"):
    shutil.copy(f, here)

# Run mkdocs build
# A subprocess is used because the Mkdocs Python API does not print any logging info
print("Building docs (this could take a few minutes)...")
run("mkdocs build")  # Build with CLI
# build(load_config())  # Build with Python API
from pathlib import Path
import shutil
from subprocess import run
from rich.progress import track

# Locate some directories relative to this file
here = Path(__file__).parent
repo = here.parent
periph_docs_dir = repo / "Libraries" / "PeriphDrivers" / "Documentation"

for f in periph_docs_dir.glob("*_Doxyfile"):
    run(f"doxygen {f.name}", cwd=f.parent)
    micro = f.name.split("_")[0].upper()  # max32xxx_Doxyfile -> MAX32XXX
    dest = here / "Libraries" / "PeriphDrivers" / "Documentation" / micro
    # ^ Recreate directory structure so built links work
    shutil.copytree(periph_docs_dir / micro, dest, dirs_exist_ok=True)

# Copy root markdown files and resources into Documentation folder
shutil.copy(repo / "USERGUIDE.md", here)
shutil.copy(repo / "CONTRIBUTING.md", here)
shutil.copytree(repo / "res", here / "res", dirs_exist_ok=True)

# Run mkdocs build
# A subprocess is used because the Mkdocs Python API does not print any logging info
print("Building docs (this could take a few minutes)...")
run("mkdocs build")
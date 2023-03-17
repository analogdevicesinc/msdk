from pathlib import Path
import shutil
from subprocess import run
from mkdocs.commands.build import build
from mkdocs.config import load_config
from os import listdir
import os

# Locate some directories relative to this file
here = Path(__file__).parent.absolute()
repo = here.parent
periph_docs_dir = repo / "Libraries" / "PeriphDrivers" / "Documentation"

# Run Doxygen builds
for f in periph_docs_dir.glob("*_Doxyfile"):
    micro = f.name.split("_")[0].upper()  # max32xxx_Doxyfile -> MAX32XXX
    if not (periph_docs_dir / micro).exists():
        run(f"doxygen {f.name}", cwd=f.parent, shell=True)
    else:
        print(f"{micro} Doxygen exists, skipping")
    dest = here / "Libraries" / "PeriphDrivers" / "Documentation" / micro
    # ^ Recreate directory structure so built links work
    shutil.copytree(periph_docs_dir / micro, dest, dirs_exist_ok=True)

# Copy res (resources) folder
print("Copying res folder")
shutil.copytree(repo / "res", repo / "Documentation" / "res", dirs_exist_ok=True)

# Pre-populate markdown files
print("Copying markdown files")
for f in repo.glob("*.md"):
    print(f.name)
    shutil.copy(f, here)

for f in (repo / "Tools" / "Bluetooth").glob("*.md"):
    print(f.name)
    dest = here / "Tools" / "Bluetooth"
    if not dest.exists():
        dest.mkdir(parents=True, exist_ok=True)
    shutil.copy(f, dest)

############################## UPDATE USER GUIDE FOR CORDIO ###################
# TEMPLATE = "- [%s](%s/%s)\n"
cordio_docs_dir =  repo / "Libraries" / "Cordio" / "docs"
platform_docs_dir = repo / "Libraries" / "Cordio" / "platform" / "Documentation"

# cordio_doc_files = [f for f in listdir(cordio_docs_dir) if f.endswith('.md')]
platform_doc_files = [f for f in listdir(platform_docs_dir) if f.endswith('.pdf')]

# with open(repo / "USERGUIDE.md", 'r') as f:
#     lines = f.readlines()

# idx = 0
# foundStart = False

# for line in lines:
#     if foundStart:
#         if line[0] == '#':
#             end_idx = idx
#             break
    
#     if '#### Cordio Documentation' in line:
#         start_idx = idx
#         foundStart = True
        
#     idx += 1

# comp_lines = lines[start_idx+1:end_idx]

# for cfile in cordio_doc_files:
#     for line in lines:
#         if cfile in line:
#             cordio_doc_files.remove(cfile)
        
# for pfile in platform_doc_files:
#     for line in lines:
#         if pfile in line:
#             platform_doc_files.remove(pfile)

# if len(cordio_doc_files) > 0:
#     entries = []
#     for f in cordio_doc_files:
#         if 'LICENSE' in f:
#             continue
#         nav_name = f.split('.')[0].replace("_", " ").title()
#         entries.append(TEMPLATE % (nav_name, cordio_docs_dir, f))

# if len(platform_doc_files) > 0:
#     for f in platform_doc_files:
#         if 'LICENSE' in f:
#             continue
#         nav_name = f.split('.')[0].replace("-", " ").title()
#         entries.append(TEMPLATE % (nav_name, platform_docs_dir, f))

#     while True:
#         if lines[end_idx][0] == '-':
#             end_idx -= 1
#             break
#         end_idx -= 1

#     idx = 0
#     for entry in entries:
#         lines.insert(end_idx, entry)
#         end_idx += 1

#     content = "".join(lines)

#     with open(repo / "USERGUIDE.md", 'w') as f:
#         f.write(content)
###############################################################################

shutil.copytree(cordio_docs_dir, here / "Libraries" / "Cordio" / "docs", dirs_exist_ok=True)
shutil.copytree(platform_docs_dir, here / "Libraries" / "Cordio" / "platform" / "Documentation", dirs_exist_ok=True)

# Run mkdocs build
# A subprocess is used because the Mkdocs Python API does not print any logging info
print("Building docs (this could take a few minutes)...")
run(f"mkdocs build", cwd=repo, shell=True)  # Build with CLI
# build(load_config())  # Build with Python API

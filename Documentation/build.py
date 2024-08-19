from dataclasses import dataclass
from pathlib import Path
import shutil
from subprocess import run
from mkdocs.commands.build import build
from mkdocs.config import load_config
from os import listdir
import os
import re
from operator import attrgetter

# Locate some directories relative to this file
here = Path(__file__).parent.absolute()
repo = here.parent
periph_docs_dir = repo / "Libraries" / "PeriphDrivers" / "Documentation"
examples_dir = repo / "Examples"

target_blacklist = ["MAX32572"]

@dataclass
class ExampleInfo():
    folder: Path
    name: str
    description: str
    details: str

# Given a path to a directory, look for a main.c file and attempt to parse
# information from a @file-tagged Doxygen block (if it exists)
def parse_example_info(f: Path) -> ExampleInfo:
    example_folder = f.parent
    example_name = f.parent.name
    example_description = "Empty Description"
    example_details = "No details"
    with open(f, "r", encoding="utf-8") as file:
        file_content = file.read()
        lines = file_content.splitlines()
        for i in range(len(lines)):
            if "/*" in lines[i]:
                comment_block = [ lines[i] ]
                while "*/" not in lines[i] and i < len(lines) - 1:
                    i += 1
                    comment_block.append(lines[i])
                if "@file" in "\n".join(comment_block):
                    for l in comment_block:
                        if "@brief" in l:
                            regex = re.compile(r"@brief[\s]*[^\n]*")
                            # Match @brief, 
                            # then any whitespace [\s]*
                            # then any character except newline [^\n]*
                            match = regex.findall(l)
                            if match:
                                example_description = str(match[0]).partition(" ")[2].strip()
                                break

                        # TODO: @details (this is more difficult because it spans multiple lines)

        return ExampleInfo(example_folder, example_name, example_description, example_details)

@dataclass
class CommonExampleInfo(ExampleInfo):
    supported_parts: list

markdown_content = ""
common_examples = []
print("Searching for shared examples...")
# Start a table
markdown_content += f"### Common Examples\n\n"
markdown_content += "The following common examples are supported across multiple microcontrollers.\n\n"
markdown_content += "| Example | Description | Supported Parts |\n"
markdown_content += "| --- | --- | --- |\n"
for i in sorted(Path(dir) for dir in os.scandir(examples_dir)):
    target_micro = Path(i).name
    if target_micro not in target_blacklist:
        for main_file in Path(i).rglob("**/main.c"):
            example_info = parse_example_info(main_file)
            if example_info:
                if example_info.name not in [i.name for i in common_examples]:
                    # Example does not exist in the table yet.
                    common_examples.append(
                        CommonExampleInfo(
                            example_info.folder,
                            example_info.name,
                            example_info.description,
                            example_info.details,
                            [target_micro]
                        )
                    )
                else:
                    idx = [example_info.name == i.name for i in common_examples].index(True)
                    if target_micro not in common_examples[idx].supported_parts:
                        # Example exists in the table, but the current micro has not been added
                        # to the list of suppored parts yet.
                        common_examples[idx].supported_parts.append(target_micro)

# Remove entries with only 1 micro
common_examples = [i for i in common_examples if len(i.supported_parts) > 1]
common_examples.sort(key=attrgetter("name")) # Sort using example name attribute

for i in common_examples:
    _list = '<br>'.join(i.supported_parts)
    markdown_content += f"| **{i.name}** | {i.description} | {_list} |\n"

markdown_content += "\n\n"

common_example_names = [i.name for i in common_examples] # We'll use this as a filter for the part-specific tables below

# Create an auto-generated table of examples for each micro.
example_md_files_list = []
for i in sorted(Path(dir) for dir in os.scandir(examples_dir)):    
    target_micro = Path(i).name
    if target_micro in target_blacklist:
        print(f"Skipping {target_micro}...  (in blacklist)")
    else:
        print(f"Generating examples table for {target_micro}")
        markdown_content += f"### {target_micro.upper()} Examples\n\n" # Add header
        markdown_content += f"In addition to the [Common Examples](#common-examples), the following examples are available specifically for the {target_micro.upper()}.\n\n"
        markdown_content += "| Example | Description | MSDK Location |\n" # Start a table
        markdown_content += "| --- | --- | --- |\n"
        table_entries = []
        for main_file in Path(i).rglob("**/main.c"):
            example_info = parse_example_info(main_file)
            if example_info and example_info.name not in common_example_names:
                link = f"https://github.com/analogdevicesinc/msdk/tree/release/{Path(example_info.folder).relative_to(repo).as_posix()}"
                table_entries.append(f"| **{example_info.name}** | {example_info.description} | _Local:_`{Path(example_info.folder).relative_to(repo)}`<br>_Github:_ [link]({link})")
        markdown_content += "\n".join(sorted(table_entries))
        markdown_content += "\n\n"

# Pre-populate markdown files
print("Copying markdown files")
for f in repo.glob("*.md"):
    print(f.name)
    if (f.name != "README.md"): shutil.copy(f, here) # Workaround for https://github.com/mkdocs/mkdocs/issues/3313

(here / "Libraries" / "CLI").mkdir(exist_ok=True, parents=True)
shutil.copy(Path("Libraries") / "CLI" / "README.md", here / "Libraries" / "CLI" / "README.md") # TODO: Less hard-coded way of pulling these in

# String replace "##__EXAMPLES_LIST__##" with autogenerated tables
# Note: We use the copy for the site build in the Documentation folder so that the 
# tracked "source" doc in the root of the repo is unmodified.
ug_content = ""
with open(repo / "Documentation" / "USERGUIDE.md", "r", encoding="utf-8") as userguide:
    ug_content = userguide.read()
with open(repo / "Documentation" / "USERGUIDE.md", "w", encoding="utf-8") as userguide:
    userguide.write(ug_content.replace("##__EXAMPLES_LIST__##", markdown_content))

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

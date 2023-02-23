from pathlib import Path
from os import listdir


TEMPLATE = "- [%s](%s/%s)\n"
repo = Path.cwd().parent.parent
cordio_docs_dir =  repo / "Libraries" / "Cordio" / "docs"
platform_docs_dir = repo / "Libraries" / "Cordio" / "platform" / "Documentation"

cordio_doc_files = [f for f in listdir(cordio_docs_dir) if f.endswith('.md')]
platform_doc_files = [f for f in listdir(platform_docs_dir) if f.endswith('.pdf')]

with open(repo / "USERGUIDE.md", 'r') as f:
    lines = f.readlines()

idx = 0
foundStart = False

for line in lines:
    if foundStart:
        if line[0] == '#':
            end_idx = idx
            break
    
    if '#### Cordio Documentation' in line:
        start_idx = idx
        foundStart = True
        
    idx += 1

comp_lines = lines[start_idx+1:end_idx]

for cfile in cordio_doc_files:
    for line in lines:
        if cfile in line:
            cordio_doc_files.remove(cfile)
        
for pfile in platform_doc_files:
    for line in lines:
        if pfile in line:
            platform_doc_files.remove(pfile)

if len(cordio_doc_files) > 0:
    entries = []
    for f in cordio_doc_files:
        if 'LICENSE' in f:
            continue
        nav_name = f.split('.')[0].replace("_", " ").title()
        entries.append(TEMPLATE % (nav_name, cordio_docs_dir, f))

if len(platform_doc_files) > 0:
    for f in platform_doc_files:
        if 'LICENSE' in f:
            continue
        nav_name = f.split('.')[0].replace("-", " ").title()
        entries.append(TEMPLATE % (nav_name, platform_docs_dir, f))

    while True:
        if lines[end_idx][0] == '-':
            end_idx -= 1
            break
        end_idx -= 1

    idx = 0
    for entry in entries:
        lines.insert(end_idx, entry)
        end_idx += 1

    content = "".join(lines)

    with open(repo / "USERGUIDE.md", 'w') as f:
        f.write(content)

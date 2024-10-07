# Copyright (c) 2024 Analog Devices, Inc. All Rights Reserved
# pylint: disable=C0103,C0114,C0115,C0116,C0301

import re
import shutil
import textwrap
from operator import attrgetter
from pathlib import Path
from subprocess import run

# Locate some directories relative to this file
here = Path(__file__).parent.absolute()
repo = here.parent

target_excludelist = ["MAX32572"]


@dataclass
class ExampleInfo:
    folder: Path
    name: str
    description: str
    details: str


# Given a path to a main.c file, attempt to parse information from a
# @file-tagged Doxygen block if it exists
def parse_example_info(source_file: Path) -> ExampleInfo:
    example_details = "No details"
    example_description = "No description"
    expr = re.compile(r"(@brief|@details)\s+([\s\S]*?)(?=\n\s*\*\s*@|\n\s*\*)")
    for match in expr.finditer(source_file.read_text()):
        tag, value = match.groups()
        match tag:
            case "@brief":
                example_description = value
            case "@details":
                example_details = value
    return ExampleInfo(
        folder=source_file.parent,
        name=source_file.parent.name,
        description=example_description,
        details=example_details,
    )


@dataclass
class CommonExampleInfo(ExampleInfo):
    supported_parts: list


def generate_examples_md():
    all_examples = []
    all_target_micros = set()
    examples_dir = repo / "Examples"
    print("Searching for shared examples...")
    for directory in examples_dir.glob("*"):
        if directory.name in target_excludelist:
            print(f"Skipping {directory.name}...  (in exclude-list)")
            continue
        target_micro = directory.name
        all_target_micros.add(target_micro)
        for main_file in directory.rglob("**/main.c"):
            info = parse_example_info(main_file)
            existing_example = [e for e in all_examples if e.name == info.name]
            if not existing_example:
                # Example does not exist in the table yet.
                all_examples.append(
                    CommonExampleInfo(
                        info.folder,
                        info.name,
                        info.description,
                        info.details,
                        [target_micro],
                    )
                )
            elif target_micro not in existing_example[0].supported_parts:
                # Example exists, but not for this micro.
                existing_example[0].supported_parts.append(target_micro)

    all_examples.sort(key=attrgetter("name"))  # Sort using example name attribute

    # Generate markdown content for common examples
    common_entries = ""
    for e in all_examples:
        if len(e.supported_parts) > 1:
            common_entries += f"| **{e.name}** | {e.description} | {'<br/>'.join(e.supported_parts)} |\n"

<<<<<<< HEAD
    markdown_content = textwrap.dedent(
        """
        ### Common Examples

        The following common examples are supported across multiple microcontrollers.

        | Example | Description | Supported Parts |
        | ------- | ----------- | --------------- |
        """
    )
    markdown_content += common_entries

    # Generate markdown content for part-specific examples
    all_target_micros = sorted(all_target_micros)
    specific_examples = {}
    for micro in all_target_micros:
        specific_examples[micro] = [
            e
            for e in all_examples
            if micro in e.supported_parts and len(e.supported_parts) == 1
        ]

    for target, examples in specific_examples.items():
        markdown_content += textwrap.dedent(
            f"""
            ### {target.upper()} Examples

            In addition to the [Common Examples](#common-examples), the following examples 
            are available specifically for the {target.upper()}.

            | Example | Description | MSDK Location |
            | ------- | ----------- | ------------- |
            """
        )
        for e in examples:
            relative_path = e.folder.relative_to(repo).as_posix()
            github = (
                f"https://github.com/analogdevicesinc/msdk/tree/release/{relative_path}"
            )
            markdown_content += f"| **{e.name}** | {e.description} | _Local:_`{relative_path}`<br>_Github:_ [link]({github})\n"

    # generate examples.md from the template
    src_path = repo / "Documentation/user-guide/examples.md.tpl"
    dst_path = repo / "Documentation/user-guide/examples.md"
    dst_path.write_text(
        src_path.read_text(encoding="utf-8").replace(
            "{{EXAMPLES_LIST}}", markdown_content
        ),
        encoding="utf-8",
    )


def copy_sources():

    files_to_copy = [
        (repo / "LICENSE.md", here),
        (repo / "CONTRIBUTING.md", here),
        (repo / "Libraries/CLI/README.md", here / "Libraries/CLI"),
        (repo / "Tools/Bluetooth/*.md", here / "Tools/Bluetooth"),
        (repo / "Libraries/Cordio/docs", here / "Libraries/Cordio/docs"),
        (
            repo / "Libraries/Cordio/platform/Documentation",
            here / "Libraries/Cordio/platform/Documentation",
        ),
    ]

    print("Copying source documentation")
    for src, dst in files_to_copy:
        if not dst.exists():
            dst.mkdir(parents=True)
        if "*" in str(src):
            for f in src.parent.glob(src.name):
                print(f"-> {f}")
                shutil.copy(f, dst)
        elif src.is_file():
            print(f"-> {src}")
            shutil.copy(src, dst)
        elif src.is_dir():
            print(f"-> {src}")
            shutil.copytree(src, dst, dirs_exist_ok=True)


def build_doxygen_docs():
    # Run Doxygen builds
    print("Building Doxygen docs...")
    periph_docs_dir = repo / "Libraries/PeriphDrivers/Documentation"
    for f in periph_docs_dir.glob("*_Doxyfile"):
        micro = f.name.split("_")[0].upper()  # max32xxx_Doxyfile -> MAX32XXX
        if not (periph_docs_dir / micro).exists():
            run(f"doxygen {f.name}", cwd=f.parent, shell=True, check=False)
        else:
            print(f"-> {micro} Doxygen exists, skipping")
        dest = here / "Libraries/PeriphDrivers/Documentation" / micro
        # ^ Recreate directory structure so built links work
        shutil.copytree(periph_docs_dir / micro, dest, dirs_exist_ok=True)


def build_mkdocs():
    # Run mkdocs build
    # A subprocess is used because the Mkdocs Python API does not print any logging info
    print("Building docs (this could take a few minutes)...")
    run("mkdocs build", cwd=repo, shell=True, check=False)  # Build with CLI


=======
    markdown_content = f"""
    ### Common Examples

    The following common examples are supported across multiple microcontrollers.

    | Example | Description | Supported Parts |
    | ------- | ----------- | --------------- |
    {common_entries}

    """

    # Generate markdown content for part-specific examples
    all_target_micros = sorted(all_target_micros)
    specific_examples = {}
    for micro in all_target_micros:
        specific_examples[micro] = [
            e
            for e in all_examples
            if micro in e.supported_parts and len(e.supported_parts) == 1
        ]

    for target, examples in specific_examples.items():
        markdown_content += f"""
        ### {target.upper()} Examples

        In addition to the [Common Examples](#common-examples), the following examples 
        are available specifically for the {target.upper()}.

        | Example | Description | MSDK Location |
        | ------- | ----------- | ------------- |
        """
        for e in examples:
            relative_path = e.folder.relative_to(repo).as_posix()
            github = (
                f"https://github.com/analogdevicesinc/msdk/tree/release/{relative_path}"
            )
            markdown_content += f"| **{e.name}** | {e.description} | _Local:_`{relative_path}`<br>_Github:_ [link]({github})\n"

    # generate examples.md from the template
    src_path = repo / "Documentation/user-guide/examples.md.tpl"
    dst_path = repo / "Documentation/user-guide/examples.md"
    dst_path.write_text(
        src_path.read_text(encoding="utf-8").replace(
            "{{EXAMPLES_LIST}}", markdown_content
        ),
        encoding="utf-8",
    )


def copy_sources():
    print("Copying root markdown files")
    for f in repo.glob("*.md"):
        print(f"-> {f.name}")
        if f.name != "README.md":
            shutil.copy(
                f, here
            )  # Workaround for https://github.com/mkdocs/mkdocs/issues/3313

    print("Copying CLI markdown files")
    cli_path = repo / "Libraries/CLI"
    cli_path.mkdir(exist_ok=True, parents=True)
    shutil.copy(
        cli_path / "README.md",
        here / "Libraries/CLI/README.md",
    )

    print("Copying bluetooth markdown files")
    src = repo / "Tools/Bluetooth"
    dst = here / "Tools/Bluetooth"
    for f in src.glob("*.md"):
        print(f"-> {f.name}")
        if not dst.exists():
            dst.mkdir(parents=True, exist_ok=True)
        shutil.copy(f, dst)

    cordio_docs_dir = repo / "Libraries/Cordio/docs"
    platform_docs_dir = repo / "Libraries/Cordio/platform/Documentation"

    print("Copying Cordio markdown files")
    shutil.copytree(cordio_docs_dir, here / "Libraries/Cordio/docs", dirs_exist_ok=True)

    print("Copying platform markdown files")
    shutil.copytree(
        platform_docs_dir,
        here / "Libraries/Cordio/platform/Documentation",
        dirs_exist_ok=True,
    )


def build_doxygen_docs():
    # Run Doxygen builds
    periph_docs_dir = repo / "Libraries/PeriphDrivers/Documentation"
    for f in periph_docs_dir.glob("*_Doxyfile"):
        micro = f.name.split("_")[0].upper()  # max32xxx_Doxyfile -> MAX32XXX
        if not (periph_docs_dir / micro).exists():
            run(f"doxygen {f.name}", cwd=f.parent, shell=True, check=False)
        else:
            print(f"{micro} Doxygen exists, skipping")
        dest = here / "Libraries/PeriphDrivers/Documentation" / micro
        # ^ Recreate directory structure so built links work
        shutil.copytree(periph_docs_dir / micro, dest, dirs_exist_ok=True)


def build_mkdocs():
    # Run mkdocs build
    # A subprocess is used because the Mkdocs Python API does not print any logging info
    print("Building docs (this could take a few minutes)...")
    run("mkdocs build", cwd=repo, shell=True, check=False)  # Build with CLI


if __name__ == "__main__":
    generate_examples_md()
    copy_sources()
    build_doxygen_docs()
    build_mkdocs()

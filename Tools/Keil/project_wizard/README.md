## Keil Project Wizard Application

This folder contains the Keil Project Wizard executable and supporting files for creating new Keil projects which points to a user-selectable MSDK location - allowing users to keep their projects up to date with the latest development in the MSDK Git repo.

## Notice

This app is in pre-release (v0.1.0) and only supported for Windows OS.

Unlike creating examples through Keil's Menus and CMSIS Pack files, there are still setup steps that the user must follow to properly create a new project through this app.

## Devices Currently Supported

* MAX32675

## Setup Steps

To properly setup your project using the Project Wizard App, follow these steps:

1. Install and setup [Keil MDK-Arm](https://www.keil.com/download/product/)
2. Clone the [MSDK repository](https://github.com/analogdevicesinc/msdk)
3. Open Keil uVision 5
4. Open the `Pack Installer` and find the `Devices` Tab (usually on the left half column).
    - `Pack Installer` menu location: ![Pack Installer Location](https://github.com/analogdevicesinc/msdk/blob/feat/keil_project_wizard/Tools/Keil/project_wizard/docs/pack_installer_location.png)
5. Find `Maxim` -> `MAX32675 Family` and select the `MAX32675`.
6. Open the `Packs` Tab (usually on the right half column) and expand `Device Specific`.
7. Install the `Maxim::MAX32675` v1.3.1 pack file.
8. Close Keil.
9. Run the Project Wizard App located in the cloned MSDK repository - located in `msdk/Tools/Keil/project_wizard/project_wizard.exe`.
10. Fill in desired project settings in the opened GUI.
    - Project name - Do not use spaces in project name.
    - Project location - Default location: `C:\Keil_Projects`
    - Keil Install Location - this is the directory where Keil is installed (default location: `C:\Keil_v5`).
    - MSDK Repo Location - should be auto-filled if Wizard app was opened within the cloned MSDK repo.
    - Device selection.
    - Board selection.
    - Example selection - reference an MSDK example for new project.
    - Fill in all prompts before pressing `Finish`
11. Press the `Finish` button once ready and wait for project to be generated.
12. Locate where the project was generated and run `{INSERT_PROJECT_NAME}.uvprojx`
13. Go to `Project` -> `Options for Target 'MAX32675:Cortex-M4'...`.
    - Press `ALT+F7` for macro shortcut to `Options for Target...`. 
14. In the `Target` Tab -> `Code Generation` section -> `ARM Compiler` options, select `Use default compiler version 6`.
15. In the `C/C++ (AC6)` Tab -> `Language / COde Generation` section -> `Language C:` options, select `c99`.
16. In the `Linker` Tab, uncheck `Use Memory Layout from Target Dialog`.
17. Use the browse button (`...`) next to the `Scatter File` text box to select the `MAX32675.sct` file located in the `Tools/Keil/` folder of the MSDK repository you cloned in step #2.
18. In the `Debug` Tab -> Top right corner, select the `CMSIS-DAP Debugger` or the board's supported debugger adapter.
19. In the `Utilities` Tab, press the `Settings` button in the `Configure Flash Menu Command` section. A `Target Driver Setup` window should open.
20. In the `Flash Download` Tab, click the `Erase Full Chip` option in the `Download Function` section.
21. In the `RAM for Algorithm`, set the `Size` field to `0x3000`.
22. If using the `CMSIS-DAP Debugger` from step **18**: In the `Debug` Tab, within the sections `Debug` -> `Connect & Reset Options`, select the `Reset:` option to `VECTRESET`.
23. Press `OK` to save everything.
24. Build and load the project. Enjoy!

### Support

Contact ADI MSDK developers for more support.

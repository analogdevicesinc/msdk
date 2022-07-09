# ARM-DSP
Porting ARM's DSP examples to Maxim's M4-series microcontrollers.

# Introduction

The projects in this repo are ports of ARM's DSP example projects that can be found in the <a href="https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html" >CMSIS-DSP library</a> that have been made compatible with the build system in Maxim's Low-Power ARM Micro Toolchain.  Some slight modifications have been made to the source code to eliminate compiler errors/warnings, but the examples have been left mostly "as-is".  The major changes here have been made to the build system.

Debug launch configurations are included in each project, and should appear in the "Favorites" menu in Eclipse.


# Required Components
- Maxim ARM Cortex Toolchain
  - <a href="https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0001500A">Windows</a>
  - <a href="https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0001660A">MAC</a>

# Software Download/Install
1. Install Maxim's ARM LP Microcontroller Toolchain for your OS from one of the above links (If not already installed).
2. Clone or download+extract this repo to an accessible location.
3. Launch the Eclipse CDT IDE and open a workspace.  (For help getting started with Eclipse, see the Quick Start guide found in the README.pdf of the toolchain, or Maxim's <a href="https://pdfserv.maximintegrated.com/en/an/TUT6245.pdf">Getting Started with Eclipse</a> User Guide.)
4. Open the Eclipse Import wizard with File -> Import...
5. Under General, select the "Existing Projects into Workspace" option.  Hit Next.  <img src="img/import_general.png">
6. Browse to root directory of the repo from step 2.  Select the projects you'd like to import into the workspace.  **If the "Copy projects into workspace" option is checked, the "CMSIS" project must be selected too**. The CMSIS "project" is simply a folder that holds the CMSIS-DSP library that these projects link against.  Hit Finish. 
<img src="img/import.png">
7. You should now see the projects imported into your Eclipse workspace.  These are ready to build, debug, and copy.  
<img src="img/eclipse_projects_imported.PNG">
8. See the Setup section below for information on configuring these projects further.

# Setup
## Changing the Target Microcontroller
These example projects come pre-configured for the MAX32665 target (which covers the MAX32665-MAX32668), but if you'd like to target a different microcontroller there are just two modifications that you will need to make.

1. Change the target processor for the makefile of the project, as shown below.  For example, if targeting the MAX32660, change to TARGET=MAX32660.  The makefile will handle the rest. 
<img src="img/target.png">
2. Change the debug configuration to use the correct .cfg file for your target, as shown below.  Navigate to Run -> Debug Configurations and then to the correct GDB OpenOCD Debugging configuration for the project.  Navigate to the "Debugger" tab, and see the "Config options:" box.  Change the portion shown in red below to match your the target processor in the makefile.  For example, to target the MAX32660 modify config options to use -f target/max32660.cfg
<img src="img/debug_config.png">

## Copying an Example to a New Project
These example projects are great starting points for your own projects.  When copying a project to a new one, there are a few modifications to the copied project that you'll need to make, shown with an example below.  For this example, I've copied the "arm_bayes_example" to a new project called "my_bayes_project".


1. Right click -> Copy on the example project. 
<img src="img/copy_example_1.PNG">
2. Right click on the workspace -> Paste. Rename the project to something new. 
<img src="img/copy_example_2.PNG">
3. Right click on the new project, and select Properties.
4. Navigate to the "C/C++ Build tab", and then the "Refresh Policy" sub-tab.  Change the entry to match the new project folder.  This ensures that the Project Explorer updates automatically when changes are made. 
<img src="img/copy_example_3.PNG">
5. Rename the debug configuration file for the project to match the project name. 
<img src="img/copy_example_4.PNG">
6. Rename the PROJECT variable in the Makefile to match the project name. 
<img src="img/copy_example_5.PNG">
7. Navigate to the debug configuration for the project via Run -> Debug Configurations.
8. Edit the debug configuration to be associated with the new project, and the newly named build output file.  Hit "Apply". 
<img src="img/copy_example_6.PNG">
9. Your new project is now ready to be built & debugged, and is ready for modifications!

# Notes
- ** The filepath to the project directory must not contain spaces **
- All of the example projects retain support for Maxim's peripheral drivers and compile all of the drivers by default on first build.  This can be disabled in the makefile to reduce build time if the peripheral drivers aren't going to be used.


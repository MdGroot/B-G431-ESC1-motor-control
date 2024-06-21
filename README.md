# B-G431-ESC1 Motor Control

The directory generated controller contains generated code for controlling a Maytech MTO5065_HA on a B-G431-ESC1 STM32 based inverter board. The project can be built and flashed to the inverter board following the steps below.

## Contents
### Generated_controller.stwb6
ST Motor Control Workbench project file. 
Would normally be located in C/Users/$USER/.st_workbench/projects/

### Generated_controller
Contains generated code from the Generated_controller.stwb6 project file. 
Would normally be located in C/Users/$USER/.st_workbench/projects/

### Maytech-MTO5065-HA-C.xml
Motor information file for ST Motor Control Workbench. 
Would normally be located in C/Users/$USER/.st_motor_control/user_motors/

## Software used
ST Motor Control Workbench 6.3.0 
ST Motor Profiler 6.3.0 
STM32CubeIDE 1.15.1

## Building and flashing
- Clone the repository
- Open the directory named **STM32CubeIDE** in **Generated_controller**
- Right click on the file **.project** and open with STM32CubeIDE (This should import the project into the Cube IDE)
- Right click the project in the **Project Explorer** in STM32CubeIDE and press **Build Project**
- Right click the project in the **Project Explorer** in STM32CubeIDE and press **Run as** and click on **STM32 C/C++ Application**
The controller should now be flashed

## Running Motor Pilot 
- Build and flash software to the board
- Connect board with USB
- (Don't know if necessary) Place the file **Maytech-MTO5065-HA-C.xml** in the correct directory
- Open the **Motor Pilot** (tested with Motor Pilot 6.3.0)
- (If screen does not show large buttons labelled **Discover Board** **Launch Profiler** and **Load UI**) Click on **GUI** in top left corner and select **Reset GUI to Defualt**
- Select **Discover Board** 
- The motor pilot should now detect the board automatically and the board should now be able to be controlled from the GUI. Control gains can be altered in **Advanced Configuration**

## Running Motor Profiler 
- Build and flash software to the board
- Connect board with USB
- (Don't know if necessary) Place the file **Maytech-MTO5065-HA-C.xml** in the correct directory
- Open the **Motor Pilot** (tested with Motor Pilot 6.3.0)
- (If screen does not show large buttons labelled **Discover Board** **Launch Profiler** and **Load UI**) Click on **GUI** in top left corner and select **Reset GUI to Defualt**
- Select **Launch Profiler** 
- The **Motor Pilot** should now detect the board automatically and the board should now be able to be controlled from the GUI. Control gains can be altered in **Advanced Configuration**

## Profiling a new motor 
- In my experience it is easiest to first profile an unknown motor in **Motor Pilot** 5.4.8
- After this it is possible to start a new project in **ST Motor Control Workbench** and select the generated user motor from the **Motor Pilot 5.4.8**
- Set up the project in the **ST Motor Control Workbench** and make sure **Motor Profiler** is ticked in one of the final project steps
- Continue with instructions below

## Making changes to the controller from ST Motor Control Workbench
- Open the Generated_controller.stwb6 file in the **ST Motor Control workbench** (tested with Motor Control Workbench 6.3.0)
- After making changes save the project as a new project (ran into building issues if not done)
- Click on **Generate the project** 
- Leave all settings as they are and click on **Generate** 
- Click on **Open folder** and build and flash as described in Section *Building and flashing* but with the new project name instead of **Generated_controller**


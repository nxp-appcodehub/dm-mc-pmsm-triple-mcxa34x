# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## 3-Phase PMSM Control and PFC Solution using MCX A34x


This demo describes the implementation of sensorless triple motor FOC on NXP MCXA346 MCU. Please refer to [AN14805](https://docs.nxp.com/bundle/AN14717/page/topics/introduction.html) for complete instructions on how to use this software. 


#### Boards: X-MCXA346-3MC, dual-motor adapter board
#### Categories: Motor Control
#### Peripherals: PWM, ADC
#### Toolchains: MCUXpresso IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
- Download and install [MCUXpresso IDE V25.06 or later](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).
- Download and install the latest version of [FreeMASTER](https://www.nxp.com/design/software/development-software/freemaster-run-time-debugging-tool:FREEMASTER)(3.2.2.2).
- Download the code from Git repository dm-mc-pmsm-triple-mcxa34x.  
- MCUXpresso for Visual Studio Code: This example supports MCUXpresso for Visual Studio Code, for more information about how to use Visual Studio Code please refer [here](https://www.nxp.com/design/training/getting-started-with-mcuxpresso-for-visual-studio-code:TIP-GETTING-STARTED-WITH-MCUXPRESSO-FOR-VS-CODE).

## 2. Hardware<a name="step2"></a>
- X-MCXA346-3MC
- 2 pcs [FRDM-MC-LVPMSM](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/nxp-freedom-development-platform-for-low-voltage-3-phase-pmsm-motor-control:FRDM-MC-LVPMSM).
- 2 [LINIX 45ZWN24 Motor](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/low-voltage-3-phase-motor-for-frdm-platform:FRDM-MC-LVMTR).
- 1 pcs dual-motor adapter board
- 1 DL71-PS11M2-1.5K PMSM High-Voltage motor

- Personal Computer
- USB type-C cable and 24V adaptors × 2.

## 3. Setup<a name="step3"></a>
### 3.1 Import project
#### 3.1.A Import project from Application Code Hub
1. Open MCUXpresso IDE, in the Quick Start Panel, choose **Import from Application Code Hub** 
2. Found demo you need by searching the name directly or selecting tags you interested in. Open the project, click the **GitHub link** to then **Next**. 
3. Select **main** branch and then click **Next**.
4. Select your local path for the repo in **Destination->Directory:** window. The MCUXpresso IDE will clone the repo to the path that your selected, **Next** after the clone process.
5. Select **Import existing Eclipse projects** in **Wizard for project import** window then **Next**. 
6. Select the project in this repo(only one project in this repo) then **Finish**. 
#### 3.1.B Import project after clone git repo to your local path
1. Clone repo **dm-mc-pmsm-triple-mcxa34x ** from **GitHub**. Open MCUXpresso IDE, in the Quick Start Panel, choose **Import project(s) from file system** .
2. Select your local path of the repo in **Project directory(unpacked)** item then **Next**. If you download a compressed pack from GitHub, Select your local path of the package in **Project archive(zip)** item.
3. Select the project in this repo(show in **Projects:** window, only one project in this repo), then **Finish**. . The project will be pasted to your IDE workspace if you click **Copy projects into workspace** in **Options** item, the change you made will be saved in the copy in workspace.
#### 3.1.2 Then you will see the project in **Project Explorer** window.
### 3.2 Compiler the project

This project contains 3 configurations, **Debug**, **Release** and **Debug_SRAMX**. **Release** configuration has a standalone link script to allocate frequently called code in SRAMX to improve performance. **Right click** the project name in **Project Explorer** window, and select **Properties** to open properties window. Open **Manage Configurations**, select configuration you want to use then **Set Active**.  
 
Use build botton in toolbar to compiler the project.  

### 3.3 Hardware setup
#### 3.3.1 Connect the motor and power shield board.
 Connect the 3-phase wire of the **first motor** to the **J7** connector on the **first FRDM-MC-LVPMSM** according to phase sequence (**White** wide--phase**A**; **Bule** wide--phase**B**; **Green** wide--phase**C**). Repeat the same steps for the **second motor** and **second FRDM-MC-LVPMSM**.

#### 3.3.2 Combine the motor kit with adaptor board.
Connect the **first FRDM-MC-LVPMSM** board to the **J1_M1~J4_M1** arduino connector of the **NXP Dual Motor Adaptor Board**. And connect the **second FRDM-MC-LVPMSM** board to the **J1_M2~J4_M2** arduino connector of the adaptor board. The **X-MCXA346-3MC** daughter board **J9 and J8** is connected to the **2 IDC-20Pin socket** on the adaptor board. The GERBER file and schematic of the adaptor are in path **dm-mc-pmsm-triple-mcxa34x/schematic_pcb**.

#### 3.3.3 Rework
Rework HVP board resistor **R128** to **680Ω**

#### 3.3.4 Connect daughter board and High-Voltage motor to HVP
Connect X-MCXA346-3MC daughter board to HVP board PCI socket, connect High-Voltage motor to HVP J13.

#### 3.3.5 Download and Power up
Power 2 FRDM-MC-LVPMSM boards with two 24V adaptor.  
Use a USB cable to connect to the X-MCXA346-3MC board via **J2** Download the code using debug button in tool bar after compiler. Select **CMSIS-DAP** in **Debug As** according firmware in your on-board debugger. Then unplug the USB cable and power up the HVP board with AC 220V power line.

### 3.4 Run the demo
Connect HVP board with PC through USB plugged on  **J4** on **HVP** to control the **Motor M1~M3** to run or stop, use FreeMASTER project **"pmsm_float.pmpx"** in the package path **"dm-mc-pmsm-triple-mcxa34x\motor_control\freemaster"** to control the motor, change rotor speed and obverse the speed or other value. 
## 4. Results<a name="step4"></a>

Memory Usage:
|Debug Configuration|Flash Usage|SRAM usage|SRAMX usage|
|:----:|:----:|:----:|:----:|
|All code in Flash|65.9 KB|30.3 KB|0 KB|
|Partial code in SRAM/SRAMX |66.3 KB |35.2 KB | 4.4 KB |

CPU loading:
|Debug Configuration|Flash Usage|SRAM usage|SRAMX usage|
|:----:|:----:|:----:|:----:|
|All code in Flash|65.38 %|2.98 %|68.36 %|
|Partial code in SRAM/SRAMX |30.09 %|1.14 %|31.23 %|

## 5. Support<a name="step6"></a>


#### Project Metadata

<!----- Boards ----->

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-MOTOR%20CONTROL-yellowgreen)](https://mcuxpresso.nxp.com/appcodehub?category=motor_control)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=pwm)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=adc)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://mcuxpresso.nxp.com/appcodehub?toolchain=mcux)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected functionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/X-Follow%20us%20on%20X-black.svg)](https://x.com/NXP)

## 7. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        | November 11<sup>th</sup> 2025 |

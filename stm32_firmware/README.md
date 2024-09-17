# STM32 Firmware

This folder contains the firmware for the STM32 microcontroller located on the Shift Register PCB. The microcontroller has the job of configuring the [8V79S680NLGI programmable clock divider and fanout](https://www.renesas.com/en/document/dst/8v79s680-datasheet) via SPI. It also handles the calibration procedure (initiated with a button press) and controls an indicator LED that tells the user when calibration is done.

Compile instructions:
1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (tested on version 1.12.0)
2. Open STM32CubeIDE
3. Import the project using `File > Import...`. Select the `General > Existing Projects into Workspace` import wizard and choose this folder as a root directory.
4. Open the project
5. Build the project using `Build > Build project`
6. 
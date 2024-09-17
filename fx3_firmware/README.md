# FX3 Firmware

This folder contains the firmware code for the *CYUSB3KIT-003 EZ-USB™ FX3 SuperSpeed explorer kit*

It implements a 32-bit wide GPIF II interface controlled by an external clock. Data can be continuously streamed from the GPIF II interface to the PC, with a maximum speed of 3050 Mbit/s. It is based on the cyfxbulksrcsink example from the EZ-USB FX3 SDK.

Unfortunately, the Cypress license agreement does not allow to distribute their source code (e.g. code examples from the SDK), even though their SDK is freely available. It is permitted, however, to distribute *changes* to the SDK example in the form of a patch as long as it does only contain changes I made.

Thus all changes exist in the form of a set of patches in em format (as unified format includes context lines of the original code). I included a makefile that makes it easy to apply these patches.

---

Instructions:

1. Clone this repository
2. Install the EZ-USB FX3 SDK (version 1.3.5), available under https://www.infineon.com/cms/en/design-support/tools/sdk/usb-controllers-sdk/ez-usb-fx3-software-development-kit/
3. After installing, locate the "cyfxbulksrcsink" folder located in the SDK's installation directory at /Cypress/EZ-USB FX3 SDK/1.3/firmware/basic_examples/cyfxbulksrcsink
4. Copy this folder into this directory (./fx3_firmware).
5. Run the makefile by opening a terminal and running `make`. If you are on a Windows system you can use [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install)
6. You should now have a new folder called **gpiftousb** in this directory. This contains the firmware code.

For compilation instructions refer to the [SuperSpeed Explorer Kit User Guide](https://www.infineon.com/cms/en/product/evaluation-boards/cyusb3kit-003), specifically section "2.6 EZ USB Suite (Eclipse) IDE

To actually use the firmware to stream data to a PC you also need a small Windows application called *CollectData.exe*. It is part of the examples of the *SuperSpeed Device Design By Example* book by John Hyde

---

### File structure after applying patches

```c
gpiftousb
├── .settings 
│   └── language.settings.xml // eclipse IDE project configuration
├── continuous_read.cydsn // GPIF II designer project files
│   ├── continuous_read.cyfx 
│   └── projectfiles 
│       ├── gpif2model.xml
│       ├── gpif2timingsimulation.xml
│       └── gpif2view.xml
├── .cproject // eclipse IDE project configuration
├── .project // eclipse IDE project configuration
├── cyfx_gcc_startup.S // Startup code to set up stacks and interrupts
├── cyfxbulkdscr.c // USB enumeration descriptors
├── cyfxgpif2config.h // Header file produced by GPIF II designer
├── cyfxgpiftousb.c // Main application
├── cyfxgpiftousb.h // Constants, endpoint and socket definitions
├── cyfxtx.c // Exception handlers & memory allocation routines
├── makefile
├── makefile.init
└── readme.txt
```
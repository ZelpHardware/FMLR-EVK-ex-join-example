# Getting `ex-join` example to build
(Notes by Oli Bailey, Zelp)
From commit 722cc6ddf90770cc864c8f988aa4662d998e82d9 of "ex-join" from https://gitlab.com/fmlr/fmlr-basicmac/ex-join the following steps 
were used to get a working build on the MiroMico "DEV-FMLR-STEVK1" FMLR Evaluation Kit boards that we have:

### 1. FOllow the installation instructions for the build environment
Please follow the instructions here: https://docs.miromico.ch/devkits/evk_stm_sx1272.html

This uses "Linux within Windows" WSL tool to allow Linux compilation using make, gcc etc. on a Windows PC. You might also find it useful to install the "WSL" extension in Microsoft Visual Code if you are using that as your editor/IDE.

(See ms-vscode-remote.remote-wsl in Visual Code)

### 2. Get the credentials for the MiroMico modules
Each FMLR module comes with a UUID number on it's box, and the DevEUI is labelled on the rear of the EVK PCB. Using the link  https://deveui.miromico.ch/uuid you can retrieve the JoinEUI (formerly AppEUI) and AppKey. For our two EVKs these numbers are:

|DevEUI|JoinEUI|AppKey|
|------|-------|------|
|10CE45FFFE007EA4|62BCD6ECF2C0F69B|21E23A0C9523BF9FBE8116CC1E05F2EF|
|10CE45FFFE007EA5|62BCD6ECF2C0F69B|94A9FCB9F031E8BF9ADF9DF11F55B775|

### 3. Make python scripts executable
There are some python scripts used during the `make` process. These need to be set to executable. Using a terminal from within the `ex-join` top-level directory:

`find . -name *.py -exec chmod 777 {} +`

### 4. Convert DOS newlines to Unix newlines in Python files
The Python scripts have a "shebang" (`#!/usr/bin/env python3`) at the start fo the file that contains a DOS newline that stops the scripts running properly. These can be converted using the `dos2unix` command:

`sudo apt-get install dos2unix`  
`find . -name *.py | dos2unix -v`  

### 5. Change the TARGET from SX1261 to SX1272 to match the EVK
We are using the Semtech SX1272 along with the STM32L071 MCU. We need to modify the TARGET variable in the main `Makefile`:

In /application/Makefile, comment out `TARGET := fmlr_61_x_ma625`  and uncomment `TARGET := fmlr_72_x_stl0`

### 6. Enter the correct credientials in `persodata.c`
In the file `/basicmac/lmic/persodata.c`, you will need to enter the DevEUI, JoinEUI and Appkey for the module you are using:

For example, for one of the EVKs we have, this is:

*uint64_t eui;  
eui = 0x11CE45FFFE007EA4ULL;  
memcpy(pd.deveui, &eui, 8);  
eui = 0x62BCD6ECF2C0F69BULL;  
memcpy(pd.joineui, &eui, 8);  
uint8_t nwkkey[16] = {0x21, 0xE2, 0x3A, 0x0C, 0x95, 0x23, 0xBF, 0x9F,  
                    0xBE, 0x81, 0x16, 0xCC, 0x1E, 0x05, 0xF2, 0xEF};  
memcpy(pd.nwkkey, nwkkey, 16);   
memcpy(pd.appkey, nwkkey, 16);*


### 7. Make the `bootloader.hex` file
You need to build the bootloader file and flash this before the main application.

Run `make` in the directory `basicmac/basicloader/build/boards/FMLR-72-X-STL0/`

This will build the file `bootloader.hex` (along with other object and linker files).

### 8. Make the main application
Change directory to `application` and run:
`make clean`  
`'make`  

This should create two sub-directories, `build` and `build-eu868' containing the main application and a script for the Segger Jlink programming tool.


### 9. Program the bootloader & application
Plug in the EVK to USB, along with the Segger J-Link USB programming dongle.

Remaining in the `application` directory, run:
`make jloadbl` to flash the bootlader to the EVK   
`make jload` to flash the main application






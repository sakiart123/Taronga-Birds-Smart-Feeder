# Jacaranda Frame Consulting Group 12: Free Flight Bird

------------------------------------------------------------------------------------------------------------

## Table of Contents
- [Overview](#overview)
- [Key Components](#key-components)
- [Getting Started](#getting-started)
- [Basic Setup](#basic-setup)
- [Hardware Setup](#hardware-setup)
- [Standard library](#Standard_library)
- [Usage](#usage)

------------------------------------------------------------------------------------------------------------

## Overview
Our project involves creating an autonomous feeder system. In this document, we will present detailed instructions on the assembly of both hardware and software, along with a standard development library for each module utilized.
  
------------------------------------------------------------------------------------------------------------

## Getting Started
### Arduino IDE

To upload code to the Arduino board, the **Arduino IDE** is the simplest and most beginner-friendly tool.

Visit the official Arduino website: `https://www.arduino.cc/en/software/`
Download the latest version compatible with your operating system (Windows, macOS, or Linux), and follow the installation instructions.

Once installed, plug in your Arduino board via USB. Open the Arduino IDE and:

1. Use the **board selection menu** (highlighted in blue) to choose the correct board type (e.g., Arduino Uno, Mega, Nano).
2. Select the correct **COM port** associated with your board.
3. Press the **Upload button** (highlighted in red) to compile and upload your code to the board.

![Adruino Bar](image/AdruinoBar.jpg)


------------------------------------------------------------------------------------------------------------

## Basic Setup 
### 1. Install Git
If you don't already have Git installed, you will need it to download (clone) the project files from the command line.

-   Download Git from the official website: `https://git-scm.com/downloads`
-   Follow the installation instructions for your operating system.

### 2. Clone the Project Repository

Open a terminal (on macOS/Linux) or Command Prompt/Git Bash (on Windows) and run the following command. This will create a folder named `Taronga-Birds` on your computer containing all the project files.

```bash
git clone https://github.com/Leo-Tron-usyd/Taronga-Birds.git
```

### 3.  Set Up the Arduino Sketch  

The Arduino IDE expects a project’s sketch folder to live inside your personal **Arduino** directory.

1. **Download & locate the project**  
   - After downloading, open the `Taronga-Birds` folder.

2. **Move the sketch folder**  
   - Inside `Taronga-Birds`, find:  
     ```
     final version/Adruino
     ```  
   - Move **the entire `Adruino` folder** into your Arduino library directory (defaults shown below).  
     - **Windows**  `C:\Users\<Your_Username>\Documents\Arduino`  
     - **macOS**  `/Users/<Your_Username>/Documents/Arduino`

3. **Confirm the structure**  
   Your Adruino directories should now resemble:  
   ```text
    Documents
     └─ Arduino
        ├─ libraries          ← existing libraries live here
        │  ├─ Adafruit_BusIO
        │  ├─ Adafruit_GFX_Library
        │  ├─ Adafruit_INA219
        │  ├─ Adafruit_NeoPixel
        │  ├─ Adafruit_SSD1306
        │  ├─ INA219
        │  ├─ INA219_WE
        │  ├─ RTClib
        │  └─ SdFat
### 4.  Open the Project in Arduino IDE  

1. Launch **Arduino IDE**.  
2. Choose **File → Open…**.  
3. Navigate to project folder: **Taronga-Birds → final version → final**.  
4. Select **`final.ino`** inside that folder.  

The project will now open in the IDE. You can now verify and upload the code to your board as described in the Getting Started section.

------------------------------------------------------------------------------------------------------------
## Hardware setup

------------------------------------------------------------------------------------------------------------


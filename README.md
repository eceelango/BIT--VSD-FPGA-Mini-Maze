# BIT--VSD-FPGA-Mini-Maze
VSD FPGA Based Maze Solving Robot Design and Development. 
# 🚀 VSDSquadron FPGA Mini (FM) - FSM FPGA Design Flow

This document provides an overview of the **VSDSquadron FPGA Mini (FM)** board and details the usage of essential commands in the FSM (Finite State Machine) FPGA design flow.

---

## 📋 Table of Contents

- [Board Overview](#bBoard-Overview)
- [Software Installation](#Software-Installaion).
- [Specifications](#specifications)
- [Command Breakdown](#command-breakdown)
  - [`make`](#1️⃣-make)
  - [`make build`](#2️⃣-make-build)
  - [`sudo make flash`](#3️⃣-sudo-make-flash)

- [Example Makefile Snippet](#example-makefile-snippet)
- [Full Workflow Example](#full-workflow-example)
- [Tools Typically Used](#tools-typically-used)
- [Summary](#summary)
---

## Board Overview

The **VSDSquadron FPGA Mini (FM)** is a compact and cost-effective development board designed for FPGA prototyping and embedded system projects. It offers a seamless hardware development experience with an integrated programmer, versatile GPIO access, and onboard memory, making it ideal for students, hobbyists, and developers exploring FPGA-based designs.

![VSD SQUADRON FPGA](VSD.jpg)
---

## Specifications and Pinouts

- **FPGA Chip:** Lattice UltraPlus ICE40UP5K
  - Logic Cells: 5,280
  - SPRAM: 1Mb
  - DPRAM: 120Kb
  - Multipliers: 8


- **GPIO:** 32 accessible FPGA GPIOs

- **Memory:** 4MB SPI flash for data storage and configuration

- **LED Indicators:** RGB LED for status indication

- **Power Regulation:** Onboard 3.3V regulators with external supply option

- **Dimensions:** 57mm x 29mm

| Feature                 | Specification                             |
|-------------------------|-------------------------------------------|
| Technology Node         | 40 nm                                     |
| Logic Cells             | 5,280                                     |
| Flip-Flops              | 4,960                                     |
| SRAM Blocks             | 120 Kbits                                 |
| DSP Blocks              | None                                      |
| Package Type            | SG48                                      |
| I/O Pins                | 39                                        |
| I/O Standards           | LVCMOS, LVDS                              |
| Max Operating Frequency | 133 MHz                                   |
| Clock Sources           | Internal oscillator, external clock       |
| Core Voltage            | 1.2V                                      |
| I/O Voltage             | 3.3V, 2.5V, 1.8V                          |
| Operating Temp Range    | -40°C to 85°C                             |
| Development Tools       | Project IceStorm, Yosys, NextPNR          |

---

# Prerequisites
Install the following tools before proceeding:

# General dependencies
```bash
sudo apt-get install git vim autoconf automake autotools-dev curl libmpc-dev \
libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool \
patchutils bc zlib1g-dev libexpat1-dev gtkwave picocom -y
```

# FPGA toolchain (Yosys/NextPNR/IceStorm)
```bash
sudo apt-get install yosys nextpnr-ice40 icestorm iverilog -y
```
# Setup
Clone the repository:
```bash
cd ~
git clone https://github.com/gowthamnow/VSD-MAZE-ROBOT
```

---

![VSD SQUADRON FPGA](vsd_image_6_0.png)

---
## 📌 Additional Features
- **4MB SPI Flash**
- **RGB LED indicators**
- **Onboard 3.3V**
- **32 GPIO accessible for prototyping**
- **Form Factor:** 57mm x 29mm, Height: Top 8mm, Bottom 1mm

---

---
# ⚙️ VSDSquadron FM FPGA - Software Installation Guide

This guide helps you set up the **VSDSquadron FPGA Mini (FM)** board on your system and run your first project.

---

## 📥 Required Software and Resources
- **VirtualBox** (Download: https://www.virtualbox.org/wiki/Downloads)
- **VSDSquadron FPGA Mini (FM) Software Package**
  - Download Link: https://forgefunder.com/~kunal/vsdsquadron_fpga_mini.zip
- Minimum **100GB free disk space** on `C:` or `D:` drive
- **4GB RAM** and **4 CPU cores** recommended
- **VDI file** provided inside the software package

---

## 💻 Installation Instructions (Windows Users)

### 1️⃣ Check Disk Space
Ensure you have at least **100GB free**.

### 2️⃣ Download and Extract Software
- Download the **VSDSquadron software zip** package.
- Extract it to a known location.

### 3️⃣ Install VirtualBox
- Download and install **Oracle VirtualBox**.

### 4️⃣ Create a Virtual Machine
- Open VirtualBox → **New** → Enter details:
  - Name: *VSDSquadron_FPGA*
  - Type: *Linux*
  - Version: *Xubuntu (64-bit)*
- Allocate:
  - **RAM:** 4096 MB
  - **CPU:** 4 cores

### 5️⃣ Select the VDI File
- In hard disk settings, select: **Use an existing virtual hard disk file**
- Browse to the extracted **.VDI file**

### 6️⃣ Start the Virtual Machine
- Boot the VM and login with:
  - **Username:** vsdiat
  - **Password:** vsdiat

---

## 📂 Running the Example Project (Blink LED)

### 1️⃣ Open Terminal in VM
- Right-click on desktop → Open Terminal

### 2️⃣ Navigate to Project Folder
```bash
cd VSDSquadron_FM
cd blink_led
```

### 3️⃣ Connect the Board to VM
- **Connect FPGA board via USB**
- In VirtualBox → **Devices → USB → FTDI Single RS232-HS**
- Verify connection:
```bash
lsusb
```
- Look for **"Future Technology Devices International"**

---

## 🛠 Programming the Board

### Clean previous builds:
```bash
make clean
```

### Build binaries:
```bash
make build
```

### Flash to FPGA:
```bash
sudo make flash
```

✅ **If successful:** RGB LEDs on the board will blink.

---

## 📝 Troubleshooting:
- If flashing fails, reconnect the board and select **Devices → USB → FTDI Single RS232-HS** again.
- Retry `sudo make flash`.

---

## Command Breakdown

### 1️⃣ `make`

**Purpose:**  
Compiles the Verilog design files.

**What Happens:**  
- Runs synthesis and simulation tasks.
- Generates intermediate files like `.json`, `.vvp`, or `.bin`.

**Usage:**
```bash
make
```

---

### 2️⃣ `make build`

**Purpose:**  
Builds the design and prepares the FPGA bitstream.

**What Happens:**  
- Maps the synthesized design to FPGA constraints.
- Generates the final FPGA-ready binary or bitstream (e.g., `top.bin` or `top.bit`).

**Usage:**
```bash
make build
```

---

### 3️⃣ `sudo make flash`

**Purpose:**  
Flashes (uploads) the generated bitstream or binary to the FPGA hardware.

**Why `sudo`?**  
- Flashing often requires USB/JTAG access, needing root permissions.

**Usage:**
```bash
sudo make flash
```
![VSD SQUADRON FPGA](VSDSquadron-FPGA-Mini-FM-board-Block-Diagram.png)
---

## Example Makefile Snippet

```makefile
build:
    yosys -p "synth_ice40 -top top -json top.json" top.v
    nextpnr-ice40 --hx8k --json top.json --asc top.asc
    icepack top.asc top.bin

flash:
    iceprog top.bin
```

---

## Full Workflow Example

```bash
make            # Synthesize and simulate the Verilog FSM
make build      # Generate bitstream for FPGA
sudo make flash # Flash the bitstream onto the FPGA board
```

---

## Notes

- Toolchains and flashing commands might vary based on your FPGA board (e.g., iCEBreaker, Arty).
- Replace `iceprog` with `openFPGALoader` or `vivado` for other FPGA platforms.
- Always check hardware permissions; `sudo` may be required for flashing.

---

## Tools Typically Used

- **Yosys:** Synthesis
- **NextPNR:** Place & Route
- **icepack / Vivado / openFPGALoader:** Bitstream generation
- **iceprog / openFPGALoader:** Flashing to FPGA hardware

---

## Summary

| Command           | Description                             |
|-------------------|-----------------------------------------|
| `make`            | Compile and synthesize Verilog code     |
| `make build`      | Generate FPGA-ready bitstream           |
| `sudo make flash` | Upload bitstream to the FPGA hardware   |

---

## Getting Started

- **Software Tools Required:** Project Icestorm, Yosys, NextPNR
- **Programming:** Onboard FTDI FT232H enables USB-based programming.
- **First Project:** A preloaded "blink LED" example is included for quick testing.
  

# BIT--VSD-FPGA-Mini-Maze

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/0b69ca197324d60a622b113e53be56435d2b689a/bot_pictures/3.jpg)

VSD FPGA Based Maze Solving Robot Design and Development.
# 🚀 VSDSquadron FPGA Mini (FM)

#### This document presents a detailed walkthrough of building a Maze Solver Bot using the VSDSquadron FPGA Mini (FM) board. It includes essential command-line operations, PCB design schematics, and RTL code implementation, offering a complete view of the hardware and software integration process.
---

## 📋 Table of Contents

* [Board Overview](#board-overview)
* [Specifications and Pinouts](#specifications-and-pinouts)
* [VSDSquadron FM FPGA - Software Installation Guide](#vsdsquadron-fm-fpga---software-installation-guide)
* [Command Breakdown](#command-breakdown)
  * [make](#make)
  * [make build](#make-build)
  * [sudo make flash](#sudo-make-flash)
* [Example Makefile Snippet](#example-makefile-snippet)
* [Full Workflow Example](#full-workflow-example)
* [Tools Typically Used](#tools-typically-used)
* [Main Commands](#main-commands)
* [Single Layer - Maze Solver Bot Development](#single-layer---maze-solver-bot-development)
* [Double Layer - Maze Solver Bot Development](#double-layer---maze-solver-bot-development)
  * [PCB Design](#pcb-design)
    * [Schematic Diagram](#schematic-diagram)
    * [Draftsman Drawing](#draftsman-drawing)
    * [PCB Layout – Top and Bottom Layers](#pcb-layout--top-and-bottom-layers)
    * [3D View of the PCB](#3d-view-of-the-pcb)
    * [Fabrication and Assembled Bot](#fabrication-and-assembled-bot)
  * [FPGA](#fpga)
    * [RTL code](#rtl-code)
    * [Resource Utilization](#resource-utilization)
* [Final Output – Working Bot Demo](#final-output--working-bot-demo)
* [Project Recognition](#project-recognition)




## Board Overview

The **VSDSquadron FPGA Mini (FM)** is a compact and cost-effective development board designed for FPGA prototyping and embedded system projects. It offers a seamless hardware development experience with an integrated programmer, versatile GPIO access, and onboard memory, making it ideal for students, hobbyists, and developers exploring FPGA-based designs.

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/12064d2e4938a2cc4965f8cfe82bb17c3c26cdf1/bot_pictures/VSD.jpg)
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

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/a90ec099435a084c8a732f5058099c76256c94a0/bot_pictures/vsd_image_6_0.png)

---
## 📌 Additional Features
- **4MB SPI Flash**
- **RGB LED indicators**
- **Onboard 3.3V**
- **32 GPIO accessible for prototyping**
- **Form Factor:** 57mm x 29mm, Height: Top 8mm, Bottom 1mm

---

---
# VSDSquadron FM FPGA - Software Installation Guide

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
![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/9faa17b0a74b86f9fc1814ec87962e763c962154/bot_pictures/Block_Diagram.png)
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

## Main Commands

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


---


# Single Layer - Maze Solver Bot Development

You can refer to the following resources related to the single-layer maze bot in the dedicated repository:

- **Images** of the maze and bot
- **PCB Designs** used for the single-layer implementation
- **Output Videos** demonstrating the bot's working

👉 [Click here to visit the Single Layer Maze Development Repository](https://github.com/gowthamnow/VSD-MAZE-ROBOT)

---

# Double Layer - Maze Solver Bot Development

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/4fcfc892149c2ff764d52e138a69caa8df11c105/bot_pictures/old%20vs%20new%20.jpg)

 
![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/6212875e7ef7beb97addb6a4844ab26e8c45bf30/bot_pictures/1.jpg)

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/7904d269b63ad11fcb793d571d3f5d01846e4923/bot_pictures/2.jpg)

---

## PCB Design 
---

## Schematic Diagram

Below is the complete **schematic diagram** for the double-layer maze development board. It includes the following circuit blocks:

| **Component**                    | **Function**                                          |
| -------------------------------- | ----------------------------------------------------- |
| **Voltage Regulator**            | 5V power supply using **LM7805** with external switch |
| **VSD Microcontroller Unit**     | Core control logic and signal processing              |
| **Motor Driver**                 | Controls left and right motors using **ROB-14450**    |
| **Ultrasonic Sensors (HC-SR04)** | Three sensors for obstacle detection                  |
| **Encoders**                     | Wheel encoder input for feedback control              |
| **Bluetooth Module**             | Wireless communication                                |
| **LED Outputs**                  | Status indication                                     |


---
 

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/ccc17de0ef78f47d40051bdd0e8857a9d676e321/bot_pictures/MAZE_page-0001.jpg)

---
## Draftsman Drawing
---
The following image shows the **Draftsman view** of our PCB, including:

- Top view of component layout
- Left, right, and back profiles for assembly reference
- Proper alignment and height of all modules
---
![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/aa70aac27868814716829a5d5d7531ed4b8af8a5/PCB/PCB_pictures/Draftsman.jpg)

---

## PCB Layout – Top and Bottom Layers

---
![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/c38af81cfdb986aae24f634a7a06924426d394ef/PCB/PCB_pictures/Top%26Bottom_layers.jpg)

---

## 3D View of the PCB

---

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/18ccc91b92ec72b3cbb3d05aec9455673a311678/PCB/PCB_pictures/3D_view.jpg)

---

## Fabrication and Assembled Bot
---

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/2226986757d5095fbbd866b1b1e8b3b0e6d65892/bot_pictures/Assembled%20and%20soldered.jpg)

---

# FPGA

---

## RTL code

```verilog
`include "ultra_sonic_sensor.v"

module top (
  input  wire echo1, echo2, echo3, // Right, Front, Left ultrasonic sensors
  output wire trig1, trig2, trig3,
  output reg AIN1, AIN2, BIN1, BIN2, // Motor direction control
  output wire PWMA, PWMB,            // Motor speed control
  output wire STBY,                  // Motor driver standby
  output reg led1, led2, led3        // Debug LEDs
);

assign STBY = 1'b1; // Motor always enabled

// -----------------------------------------
// 1. Internal Oscillator (6 MHz)
// -----------------------------------------
wire int_osc;
SB_HFOSC #(.CLKHF_DIV("0b11")) u_SB_HFOSC (
  .CLKHFPU(1'b1),
  .CLKHFEN(1'b1),
  .CLKHF(int_osc)
);

// -----------------------------------------
// 2. Ultrasonic Triggering & Sensing
// -----------------------------------------
wire measure1, measure2, measure3;
refresher250ms refresher1 (.clk(int_osc), .en(1'b1), .measure(measure1));
refresher250ms refresher2 (.clk(int_osc), .en(1'b1), .measure(measure2));
refresher250ms refresher3 (.clk(int_osc), .en(1'b1), .measure(measure3));

wire [15:0] raw1, raw2, raw3;
hc_sr04 sensor1 (.clk(int_osc), .measure(measure1), .echo(echo1), .trig(trig1), .distance_cm(raw1)); // Right
hc_sr04 sensor2 (.clk(int_osc), .measure(measure2), .echo(echo2), .trig(trig2), .distance_cm(raw2)); // Front
hc_sr04 sensor3 (.clk(int_osc), .measure(measure3), .echo(echo3), .trig(trig3), .distance_cm(raw3)); // Left

// -----------------------------------------
// 3. Moving Average Filter (3 samples) - RESTORED
// -----------------------------------------
reg [15:0] sum1 = 0, sum2 = 0, sum3 = 0;
reg [15:0] dist1 = 0, dist2 = 0, dist3 = 0;
reg [1:0] count1 = 0, count2 = 0, count3 = 0;

always @(posedge int_osc) begin
  if (measure1 && raw1 < 200) begin
    sum1 <= sum1 + raw1;
    count1 <= count1 + 1;
    if (count1 == 3) begin
      dist1 <= (sum1 + raw1) >> 2;
      sum1 <= 0; count1 <= 0;
    end
  end

  if (measure2 && raw2 < 200) begin
    sum2 <= sum2 + raw2;
    count2 <= count2 + 1;
    if (count2 == 3) begin
      dist2 <= (sum2 + raw2) >> 2;
      sum2 <= 0; count2 <= 0;
    end
  end

  if (measure3 && raw3 < 200) begin
    sum3 <= sum3 + raw3;
    count3 <= count3 + 1;
    if (count3 == 3) begin
      dist3 <= (sum3 + raw3) >> 2;
      sum3 <= 0; count3 <= 0;
    end
  end
end

// -----------------------------------------
// 3.1 Out-of-range check
// -----------------------------------------
parameter MAX_DIST = 16'd200;
reg out_of_range = 0;

always @(posedge int_osc) begin
  if (dist1 > MAX_DIST || dist2 > MAX_DIST || dist3 > MAX_DIST)
    out_of_range <= 1;
  else
    out_of_range <= 0;
end

// -----------------------------------------
// 4. PWM Motor Speed Control with Bilateral PD
// -----------------------------------------
parameter BASE = 90;        // Increased base speed
parameter WALLP = 18;
parameter WALLD = 10;
parameter MAXCORR = 30;
parameter TURN_PWM = 30;
parameter PWM_MAX=90;
parameter PWM_MIN=30;    // Increased turn speed

reg signed [15:0] err = 0, prev_err = 0, corr = 0;
reg [7:0] pwm_left = BASE, pwm_right = BASE;
reg [7:0] pwm_counter = 0;
reg pwm_a = 0, pwm_b = 0;

reg [23:0] pd_blink_counter = 0;
reg led2_state = 0;

always @(posedge int_osc) begin
  if (!turning && !wait_turn && AIN1 == 0 && AIN2 == 1 && BIN1 == 0 && BIN2 == 1) begin
    // Bilateral wall-following: balance between right and left walls
    err <= dist1 - dist3;
    if (err > -2 && err < 2)
      corr <= 0;
    else
      corr <= ((WALLP * err + WALLD * (err - prev_err)) >>> 2);

    prev_err <= err;

    if (corr > MAXCORR) corr <= MAXCORR;
    if (corr < -MAXCORR) corr <= -MAXCORR;

    if (corr > 0) begin
      pwm_left  <= (BASE + (corr >>> 1) > PWM_MAX) ? PWM_MAX : BASE + (corr >>> 1);
      pwm_right <= (BASE - corr < PWM_MIN) ? PWM_MIN : BASE - corr;
    end else begin
      pwm_left  <= (BASE + corr < PWM_MIN) ? PWM_MIN : BASE + corr;
      pwm_right <= (BASE - (corr >>> 1) > PWM_MAX) ? PWM_MAX : BASE - (corr >>> 1);
    end

    pd_blink_counter <= pd_blink_counter + (corr[15] ? -corr : corr);
    if (pd_blink_counter > 100000) begin
      led2_state <= ~led2_state;
      pd_blink_counter <= 0;
    end
  end else begin
    pwm_left  <= TURN_PWM;
    pwm_right <= TURN_PWM;
    led2_state <= 0;
    pd_blink_counter <= 0;
  end
end

always @(posedge int_osc) begin
  pwm_counter <= pwm_counter + 1;
  pwm_a <= (pwm_counter < pwm_left);
  pwm_b <= (pwm_counter < pwm_right);
end

assign PWMA = pwm_a;
assign PWMB = pwm_b;

// -----------------------------------------
// 5. Movement Logic - Flag-based FSM
// -----------------------------------------
parameter THRESH = 16'd4;           // Adjusted threshold
parameter HYST = 16'd2;
parameter TURN_TIME = 24'd3600000;   // Adjusted for 6 MHz: ~0.6 seconds
parameter UTURN_TIME = 24'd6000000;  // Adjusted for 6 MHz: ~1.0 seconds
parameter MARGIN = 16'd4;

reg [23:0] turn_timer = 0;
reg turning = 0;
reg [1:0] dir = 0;
reg wait_turn = 0;
reg [23:0] pre_turn_delay = 0;

always @(posedge int_osc) begin
  led1 <= (dist2 < THRESH);
  led2 <= led2_state;
  led3 <= (dist3 >= (THRESH + HYST + MARGIN));

  if (turning) begin
    turn_timer <= turn_timer + 1;
    case (dir)
      2'b01: begin 
               AIN1 <= 0; AIN2 <= 1; 
               BIN1 <= 1; BIN2 <= 0; end // Right turn
      2'b10: begin
               AIN1 <= 1; AIN2 <= 0; 
               BIN1 <= 0; BIN2 <= 1; end // Left turn
      2'b11: begin 
               AIN1 <= 0; AIN2 <= 1;
               BIN1 <= 1; BIN2 <= 0; end // U-turn
      default: begin
                AIN1 <= 0; AIN2 <= 0; 
                BIN1 <= 0; BIN2 <= 0; end
    endcase
    if (turn_timer >= ((dir == 2'b11) ? UTURN_TIME : TURN_TIME)) begin
      turning <= 0;
      turn_timer <= 0;
    end
  end else begin
    if (out_of_range) begin
      AIN1 <= 0; AIN2 <= 0;
      BIN1 <= 0; BIN2 <= 0;
    end else if (!wait_turn) begin
      if ((dist1 >= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN ))) begin
          wait_turn <= 1;
          dir <= 2'b01;
          pre_turn_delay <= 0; // Turn right
      end 
      
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN)) begin
          AIN1 <= 0; AIN2 <= 1;
          BIN1 <= 0; BIN2 <= 1;//go forward 
      end
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN)) begin
          AIN1 <= 0; AIN2 <= 1;
          BIN1 <= 0; BIN2 <= 1;//go forward 
      end 
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN)) begin
          wait_turn <= 1; 
          dir <= 2'b10;
          pre_turn_delay <= 0; // Turn left
      end  
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN))begin
          wait_turn <= 1;
          dir <= 2'b11;
          pre_turn_delay <= 0; // U-turn if both blocked
      end
      end
       else begin
      // Wait state before turning
      AIN1 <= 0; AIN2 <= 0;
      BIN1 <= 0; BIN2 <= 0;
      pre_turn_delay <= pre_turn_delay + 1;
      if (pre_turn_delay >= 24'd3000_000) begin // 0.5  second delay at 6 MHz
        turning <= 1;
        wait_turn <= 0;
        pre_turn_delay <= 0;
        turn_timer <= 0;
      end
    end
  end
end

endmodule
```
---
## Resource Utilization
---

### ⚙️ Resource Utilization Workflow

The following commands were used to synthesize the design and generate the resource utilization report:
---
<pre><code># Step 1: Start Yosys
yosys

# Step 2: Run synthesis for the iCE40 FPGA family
synth_ice40 -top top_module_name -json out.json
</code></pre>

---

> ### Replace `top_module_name` with the name of your top module.

---

![image alt](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/6bc06af5dfce3dbc9b7e98691935b26ecb2018d7/bot_pictures/resource_report.png)

---

## Final Output – Working Bot Demo

---

### 🐢 23% Speed Demo

[![Watch the video](https://img.youtube.com/vi/Wy0YzF0P1mc/0.jpg)](https://youtu.be/Wy0YzF0P1mc)


[▶️ Watch Video](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/6bc06af5dfce3dbc9b7e98691935b26ecb2018d7/bot_videos/23%25_speed.mp4)
*Initial test at 23% speed to verify wall-following and obstacle detection.*

---

### ⚙️ 35% Speed Demo – Test Run 1
[![Watch the video](https://img.youtube.com/vi/dcbGqtxwAFU/0.jpg)](https://youtu.be/dcbGqtxwAFU)


[▶️ Watch Video](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/6bc06af5dfce3dbc9b7e98691935b26ecb2018d7/bot_videos/35%25%20_speed.mp4)
*First performance run at 35% speed to test navigation accuracy and speed handling.*

---

### ⚙️ 35% Speed Demo – Test Run 2

[▶️ Watch Video](https://github.com/eceelango/BIT--VSD-FPGA-Mini-Maze/blob/6bc06af5dfce3dbc9b7e98691935b26ecb2018d7/bot_videos/35%25_speed_2.mp4)
*Second 35% speed test to demonstrate stability and route correction.*

---

## Project Recognition

Our Maze Solver Bot, developed entirely using the VSDSquadron FPGA platform, was officially recognized for its innovation and practical hardware implementation. This accomplishment was independently achieved by our team and has been featured in prominent LinkedIn posts showcasing FPGA-based robotics.

---

### 🏆 Featured Mentions

* [🔧 Build a Robot with VLSI + FPGA](https://www.linkedin.com/posts/kunal-ghosh-vlsisystemdesign-com-28084836_final-5h-build-a-robot-with-vlsifpga-activity-7340366902277390336-0WHi?utm_source=share&utm_medium=member_desktop&rcm=ACoAAFm02L0BxEsQ5DqBZizIPfKSw7-9JeUtTNA)

* [🚗 Maze Solver Bot with VSDSquadron FPGA](https://www.linkedin.com/posts/kunal-ghosh-vlsisystemdesign-com-28084836_vsdsquadron-lattice-activity-7355861256722960384--0Xs?utm_source=share&utm_medium=member_desktop&rcm=ACoAAFm02L0BxEsQ5DqBZizIPfKSw7-9JeUtTNA)

---

# STM32 Development Guide - NUCLEO-F446RE 

This repository contains STM32 step-by-step labs for the NUCLEO-F446RE board. Both HAL-based (using STM32CubeMX) and Bare-Metal (no HAL) programming flows are covered.

---

## ⚙️ HAL Programming Workflow

HAL (Hardware Abstraction Layer) programming uses **STM32CubeMX** to auto-generate code, which is imported into **STM32CubeIDE**.

1. **Code Generation**
   - Open STM32CubeMX
   - Click **New Project**
   - Go to **MCU Selector** and select Part Number: `STM32F446RET6TR`
   - Click **Start Project**
   - When prompted: *Initialize all peripherals with default mode* → **Yes**

2. **Project Manager Setup**
   - Set Project Name (e.g., `LAB_1B`)
   - Choose a location (macOS example):  
     `/Users/<your-username>/STM32CubeIDE/workspace/`
   - Select Toolchain/IDE: **STM32CubeIDE**
   - Click **GENERATE CODE**

3. **Import into IDE**
   - Open STM32CubeIDE
   - Go to **File → Open Projects from File System**
   - Select your generated project folder
   - Click **Finish**

---

## 🔩 Bare-Metal Programming Workflow 

Bare-metal approach gives full hardware control, skipping HAL abstractions. Projects are created directly in STM32CubeIDE.

1. **Create Project**
   - Open STM32CubeIDE
   - Go to **File → New → STM32 Project**
   - Select Board/MCU: `STM32F446RETx`
   - Choose **Empty Project**
   - Enter Project Name (e.g., `LAB_1A`)
   - Click **Finish**

2. **Driver Setup**
   - Copy `Drivers` folder from a HAL project
   - Paste into your current project
   - Delete `Drivers/STM32F4xx_HAL_Driver`
   - Update include paths to:
     - `Drivers/CMSIS/Include`
     - `Drivers/CMSIS/Device/ST/STM32F4xx/Include`

   - In STM32CubeIDE:  
     Right Click Project → Properties → C/C++ Build → Settings → Include Paths

---

## 📁 Workspace Location (macOS)

Default location:
```
/Users/<your-username>/STM32CubeIDE/workspace/
```
You can change this during IDE setup or via **File → Switch Workspace**.

---

## 📌 Notes

- Replace `<your-username>` with your Mac username.
- **HAL workflow** is ideal for beginners (auto-generated peripheral setup).
- **Bare-Metal workflow** is ideal for advanced users (full hardware control).
- Make sure required permissions are allowed on macOS (especially first time running STM32 tools).

---

For details on each lab and concept, see individual folders or [STM32 Documentation](https://www.st.com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html).

# PID Motor Control Simulation (C++)

This project demonstrates a software implementation of a PID controller applied to a DC motor model.  
The motor’s dynamics are approximated using the following equation:

\[
a = \frac{K V - b w}{J}
\]

Where:  
- **a** → angular acceleration  
- **K** → motor constant  
- **b** → friction coefficient  
- **w** → angular velocity  
- **J** → moment of inertia  
- **V** → input voltage  

The PID controller adjusts the input voltage to drive the motor to a desired angular position or velocity.

---

## Features

- Implementation of **P**, **PI**, and **PID** control loops  
- Realistic DC motor dynamics simulation  
- Adjustable PID gains: `Kp`, `Ki`, and `Kd`  
- Modular design using C++ classes and header separation

---

## Build & Run Instructions (Windows)

### 1. Prerequisites
- Install **MinGW** and ensure `g++` and `mingw32-make` are added to your system PATH.

### 2. Build the Project
Open PowerShell or Command Prompt in the project directory and run:
```bash
mingw32-make
```

### 3. Run the Executable
After a successful build, run:
```bash
mingw32-make run
./main.exe
```
### 4. Clean Build Files
To remove compiled files:
```bash 
mingw32-make clean

```

### Usage
Enter the desired motor angle when prompted.
Choose between:
1 → P Control
2 → PI Control
3 → PID Control

Observe the iteration logs showing voltage, angular velocity, and position.

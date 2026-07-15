# Gimbal Designation Loop

This repository contains C-based control code for a gimbal position control experiment utilizing NI-DAQmx.  
The program reads signals from a gyroscope and a potentiometer, applies a PD controller along with a Low-Pass Filter (LPF), outputs motor commands, and logs experimental data.

## 🎬 Experimental Videos
Click the images below to watch the gimbal control experiment demonstrations on YouTube Shorts.

### Demo Videos
| 5-degree Test | 20-degree Test |
| :---: | :---: |
| <a href="https://youtube.com/shorts/k8pQFL60OwI"><img src="https://img.youtube.com/vi/k8pQFL60OwI/maxresdefault.jpg" width="300"></a> | <a href="https://youtube.com/shorts/wrr52FHKExw"><img src="https://img.youtube.com/vi/wrr52FHKExw/0.jpg" width="300"></a> |
| Designation 5 deg | Designation 20 deg |

---

## File Structure

| File | Role / Function |
|---|---|
| `main.c` | Main loop and execution sequence of the program |
| `mygimbal_runexp.c` | DAQ I/O, sensor conversion, PD control, LPF, and data logging |
| `mygimbal_runexp.h` | Experimental constants and function declarations |
| `mygimbal_designation_step.c` | Step experiment command logic and experimental info return |
| `mygimbal_designation_step.h` | Definitions for sampling frequency, trial duration, and output filename |

## Key Specifications & Settings

- **Sampling Frequency:** 200 Hz
- **Target Angle:** 20 deg
- **Controller Type:** PD Controller
- **Gyro LPF Cutoff Frequency:** 10 Hz
- **Emergency Stop Key:** `f`

## DAQ Channel Configuration

- `Dev4/ai2`: Gyro voltage
- `Dev4/ai3`: Potentiometer voltage
- `Dev4/ao0`: Switch command
- `Dev4/ao1`: Motor command

## System Requirements & Environment

This project is configured and tested under the following environment:

- **OS:** Windows
- **IDE:** Visual Studio (C Project)
- **Driver:** NI-DAQmx driver with C development support
- **Paths:** Include and Library paths configured to locate `NIDAQmx.h`
- **Device Name:** NI DAQ device named as `Dev4` in NI MAX (Measurement & Automation Explorer)

## Pre-execution Checklists

1. Ensure the physical DAQ channel connections match the configuration in the code (`Dev4`).
2. Perform the gyro offset calibration while the gimbal is **completely static (still)**.
3. Use the `f` key on the keyboard as the emergency stop.
4. Note that this program **will not execute** on standard PCs without physical NI DAQ hardware connected.

## Experimental Output Data

The experimental results are saved by default to the following file:

```text
Step_DesignationLoop.txt

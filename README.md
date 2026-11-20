# RAPID Studio 3D — Python Robotic Arm Simulator

A lightweight, ABB RobotStudio-inspired 3D simulator for the **ABB RAPID** programming language.
All robotics math (Forward Kinematics, Analytical Inverse Kinematics, Joint/Linear/Circular trajectory interpolation) and the RAPID parser/interpreter are implemented in **pure Python 3** using only the standard library—**zero `pip install` required**.

## How to Run

1. Make sure you have **Python 3.8+** installed.
2. Open a terminal in the project folder and run:
   ```bash
   python main.py
   ```
3. Your default web browser will automatically open at `http://127.0.0.1:8080`.

## Two Editor Modes

* **Demo Programs:** Explore 4 built-in industrial programs (*Pick & Place*, *Square Welding*, *Circular Arc MoveC*, and *2x2 Palletizing*).
* **Custom Code (Sandbox):** Write and execute your own RAPID programs from scratch, insert commands with the Quick-Insert snippet bar, teach coordinates directly from the 3D robot using the Manual Jog sliders, or import/export `.mod` files.

## Project Architecture

| File | Description |
| :--- | :--- |
| `main.py` | Built-in Python HTTP server exposing the simulation REST API (`/api/simulate`, `/api/fk`, `/api/ik`) and launching the 3D frontend. |
| `kinematics.py` | 6-axis industrial robot Forward Kinematics (FK), Analytical Inverse Kinematics (IK), `MoveJ` joint-space interpolation, `MoveL` linear interpolation, and `MoveC` 3D circular arc interpolation. |
| `rapid_parser.py` | General-purpose ABB RAPID parser and execution engine supporting variables, loops, conditionals, sub-procedures, motion commands, and I/O signals. |
| `static/index.html` | Interactive Three.js WebGL 3D viewport with an articulated 6-axis ABB-style robot, animated gripper, welding arc effect, real-time 3D path trails, manual jog sliders, and FlexPendant console. |

## Supported RAPID Instructions

* **Program Structure:** `MODULE ... ENDMODULE`, `PROC main() ... ENDPROC`, sub-procedure calls, comments (`!`).
* **Data Declarations:** `CONST`/`VAR`/`PERS robtarget`, `VAR num`, `CONST num`, variable assignments (`x := x + 50;`), and inline `robtarget` arrays.
* **Motion:**
  * `MoveJ target, speed, zone, tool;` (Joint-interpolated motion — **amber** trail)
  * `MoveL target, speed, zone, tool;` (Linear Cartesian motion — **cyan** trail)
  * `MoveC viaTarget, endTarget, speed, zone, tool;` (3D Circular arc motion — **emerald** trail)
  * `Offs(target, dx, dy, dz)` dynamic coordinate offsets
* **Flow Control & I/O:**
  * `FOR var FROM start TO end DO ... ENDFOR`
  * `WHILE condition DO ... ENDWHILE`
  * `IF condition THEN ... [ELSE ...] ENDIF`
  * `Set` / `Reset` / `SetDO` / `PulseDO` (`doGripper`, `doWeld`, or any custom signal)
  * `WaitTime <seconds>;` and `TPWrite "<message>";`
  * Standard RobotStudio directives (`ConfJ`, `ConfL`, `AccSet`, `VelSet`, `SingArea`)

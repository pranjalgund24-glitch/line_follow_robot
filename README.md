# Raylib Graph Line Follower

A C program built with raylib that simulates a line-following robot on a 15x15 coordinate graph. The user types five (x, y) waypoints, each value below 10, and the program connects them to a fixed end point at (10, 10). A robot sprite then travels along the path at a constant speed, with live coordinates shown and a reset option with the R key.

## Features

- Coordinate grid with labelled axes (labels every 5 units)
- Keyboard entry of five waypoints, one value at a time (X, then Y)
- Input validation: values of 10 or more are rejected with an error message
- Fixed end point at (10, 10) added automatically after the fifth point
- Path drawn as connected lines, with each waypoint labelled P1 to P5
- Animated robot that moves from point to point and shows its live (x, y) position
- Reset with a single key to start over

## Requirements

- A C compiler (GCC, Clang or MinGW)
- [raylib](https://www.raylib.com/) installed on your system

## Build and run

Save the program as `main.c`, then compile it.

**Linux**

```bash
gcc main.c -o robot -lraylib -lm
./robot
```

**macOS**

```bash
clang main.c -o robot -lraylib -framework IOKit -framework Cocoa -framework OpenGL
./robot
```

**Windows (MinGW)**

```bash
gcc main.c -o robot.exe -lraylib -lopengl32 -lgdi32 -lwinmm
robot.exe
```

## Controls

| Key | Action |
|---|---|
| `0-9`, `.`, `-` | Type a value |
| `Backspace` | Delete the last character |
| `Enter` | Confirm the value (X first, then Y) |
| `R` | Reset and enter new points |

## How to use

1. Start the program. A panel shows which point (1 to 5) you are entering.
2. Type the X value and press Enter, then type the Y value and press Enter.
3. Repeat until all five points are entered. Values must be less than 10.
4. The end point (10, 10) is added automatically and the robot starts moving.
5. When it reaches the end point, press `R` to run again with new points.

## Settings

These constants at the top of the file can be changed:

| Constant | Default | Meaning |
|---|---|---|
| `USER_POINTS` | 5 | Number of points the user enters |
| `ROBOT_SPEED` | 5.0 | Robot speed in graph units per second |
| `GRID_SCALE` | 25.0 | Pixels per graph unit |

## Known limitations

- The grid spans -15 to 15, but the 600 px window shows only about -12 to 12 vertically. Increase the window height or reduce `GRID_SCALE` to see the full grid.
- Only values of 10 or more are rejected. Very negative values are accepted and may be drawn off-screen.
- Malformed input such as `1.2.3` or `5-3` is accepted and read as a number by `atof`.
- On a very slow frame the robot can overshoot a waypoint slightly before correcting.

## Built with

- C
- [raylib](https://www.raylib.com/)

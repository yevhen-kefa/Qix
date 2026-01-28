# 🎮 Qix - Python Clone

A Python implementation of the classic arcade game **Qix**, developed using the `fltk` library and `matplotlib`. This project features single-player and two-player local co-op modes, custom enemies, and power-ups.

**Authors:** Irimbola & Yevhen

## 📋 Description

The goal of the game is to claim territory on the map by drawing lines (Stix) to close off rectangles. You must claim at least **75%** of the playfield to win the level, while avoiding the **Qix** (the chaotic entity moving inside) and **Sparx** (enemies moving along the borders).

## ⚙️ Prerequisites

To run this game, you need **Python 3** installed on your machine.

### Dependencies
The game requires the following libraries:
1.  **Matplotlib** (for polygon calculations)
2.  **Numpy**
3.  **FLTK** (Ensure the `fltk.py` file is in the same directory as the game script).

### Installation
Install the necessary Python packages using pip:

```bash
pip install matplotlib numpy
```

* Note: If you are on Linux/Ubuntu and encounter errors, try:

```bash
sudo apt-get install python3-matplotlib python3-numpy
```

## How to Run

1. Ensure qix.py, fltk.py, and QixImage.png are in the same folder.

2. Run the script via terminal:

```bash
python3 qix.py
```

## Controls
Menu

* Mouse Left Click: Select Game Mode (1 Player / 2 Players / Quit).

In-Game Controls

| Action | Player 1 | Player 2 |
| --- | --- | --- 
| Move Up | Arrow Up | z |
| Move Down | Arrow Down | s |
| Move Left | Arrow Left | q |
| Move Right | Arrow Right | d |
| Start/Stop | Enter | N/A (Automatic/Shared logic)|
| Speed Boost | Right Shift | Left Shift|

*Note for Player 2: The controls are mapped to the ZQSD layout (standard French AZERTY). On a QWERTY keyboard, use W, A, S, D positions (which correspond to Z, Q, S, D).

## Game Mechanics

* Drawing: Press Enter to leave the safety of the border and start drawing a line. Return to a border to close the shape and capture the area.

* Invincibility: Collect Yellow Apples 🍏 to become invincible for 5 seconds.

* Lives: You start with 4 lives. You lose a life if:

    * The Qix touches you or your unfinished line.

    * A Sparx touches you while you are on the border.

* Victory: Capture 75% of the map area.

## 📂 File Structure

* qix.py: Main game source code.

* fltk.py: Graphic library wrapper (Required).

* QixImage.png: Image resource for the main menu.

* README.md: This documentation.


University Project - 2024
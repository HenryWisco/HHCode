# Student Setup Guide

Do this once on every computer you'll code on. It takes about 20 minutes,
most of it waiting for downloads. You may need admin rights on the computer
for step 1 (school laptops may need IT).

## 1. Install VS Code

1. Go to https://code.visualstudio.com/ and download the version for your computer.
2. **Windows:** run the installer. On the "Select Additional Tasks" page, check
   **Add "Open with Code" action** and **Add to PATH**.
   **Mac:** open the download and drag *Visual Studio Code* into *Applications*.
3. Open VS Code.

## 2. Install the VS Code extensions

1. In VS Code, click the **Extensions** icon on the left (four squares), or press
   `Ctrl+Shift+X` (`Cmd+Shift+X` on Mac).
2. Search **VEX Robotics** and install the one published by **VEX Robotics**.
3. Search **C/C++** and install the one published by **Microsoft**.
4. **Do not** install or enable the **PROS** extension for this project. It
   conflicts with the VEX extension. (If you already have it, right-click it →
   *Disable (Workspace)*.)

Opening the template later also pops up "This workspace has extension
recommendations". Clicking **Install** installs both for you.

## 3. Get the code

Your coach gives you the template as a **zip file**.

1. Find the zip (usually in *Downloads*, or wherever your coach put it).
2. Unzip it into a folder you'll remember, like `Documents/VEX`.
   **Windows:** right-click the zip → **Extract All...** → pick the folder → **Extract**.
   **Mac:** double-click the zip, then drag the new folder to where you want it.
3. Rename the unzipped folder after your team, like `1234A-2026`.
4. In VS Code: **File → Open Folder...**, pick that folder, and click
   **Select Folder** (**Open** on a Mac).
5. If VS Code asks "Do you trust the authors?", click **Yes**.

On Windows, don't work on the files while they're still inside the zip.
Double-clicking a zip only peeks inside it, and VS Code can't build from
there. Extract it first.

## 4. First build (downloads the VEX toolchain)

1. With the project open, look at the bottom-left of VS Code for the VEX bar
   (the VEX logo, **Build**, **Download/Run** icons, and a slot number).
2. Click **Build** (the hammer). The first time, the VEX extension downloads the
   V5 SDK and compiler. This can take a few minutes. If it asks to update the
   SDK version, click **Yes**.
3. You should see `Build Finished` with no errors in the output panel.

If the VEX bar isn't there, make sure you opened the project **folder** (the
one with `makefile` in it), not a parent folder. Unzipping sometimes puts the
project one folder deeper than you expect.

## 5. Download to the robot

1. Plug the V5 Brain into the computer with a micro-USB cable (or plug the
   controller in, if it's paired with the Brain).
2. The VEX bar should show the device as connected. If the extension says the
   Brain firmware (VEXos) is out of date, let it update. Don't unplug during the update.
3. Pick a program **slot** (1–8) by clicking the slot number.
4. Click **Download** to send the program, or **Run** to send and start it.
5. On the Brain, the program appears under that slot with the project name.

## 6. Day-to-day workflow

1. Open your team folder in VS Code.
2. Make your changes. Save with `Ctrl+S` (`Cmd+S` on Mac).
3. **Build** often. Fix the first error first; later errors are often caused by it.
4. **Download** and test on the robot.
5. **Back up when something works.** Your code only lives on this computer,
   so copy the whole team folder to a USB drive or your school cloud drive at
   the end of each meeting. Put the date in the copy's name
   (`1234A-2026 backup Nov 17`) so you can go back to a version that worked.

Only one computer should hold the team's "real" copy. If two people edit
separate copies, someone has to merge the changes by hand.

## Troubleshooting

| Problem | Fix |
|---|---|
| No VEX bar at the bottom | Open the folder that contains `makefile`. Check the VEX Robotics extension is installed and enabled. |
| Build fails right after unzipping | Make sure the zip was extracted (step 3), not just opened, and that the folder isn't inside a cloud-synced folder that is still syncing. |
| Red squiggles everywhere but it builds fine | Build once so the SDK downloads, then reload VS Code (**Command Palette → Developer: Reload Window**). |
| `make: command not found` or toolchain errors | Let the VEX extension finish downloading the SDK. If it keeps failing, reinstall the VEX extension. |
| Brain not detected | Try a different USB cable (some are charge-only) and a different USB port. Turn the Brain on. |
| PROS buttons show up | Disable the PROS extension for this workspace. |
| Robot drives backwards / spins | A motor is reversed. Fix it in `src/robot-config.cpp`. |

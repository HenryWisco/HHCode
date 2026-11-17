# Student Setup Guide

Do this once on every computer you'll code on. It takes about 20–30 minutes,
most of it waiting for downloads. You need admin rights on the computer for
steps 1–3 (school laptops may need IT).

## 1. Install VS Code

1. Go to https://code.visualstudio.com/ and download the version for your computer.
2. **Windows:** run the installer. On the "Select Additional Tasks" page, check
   **Add "Open with Code" action** and **Add to PATH**.
   **Mac:** open the download and drag *Visual Studio Code* into *Applications*.
3. Open VS Code.

## 2. Install Git

Git is how you download the template and save/share your code.

- **Windows:** download from https://git-scm.com/download/win and click Next through
  the installer (the defaults are fine).
- **Mac:** open Terminal and type `git --version`. If it isn't installed, macOS
  will offer to install the developer tools. Click **Install**.

Then tell Git who you are (open VS Code, then **Terminal → New Terminal**):

```
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

Make a free GitHub account at https://github.com if you don't have one, and give
your username to your coach.

## 3. Install the VS Code extensions

1. In VS Code, click the **Extensions** icon on the left (four squares), or press
   `Ctrl+Shift+X` (`Cmd+Shift+X` on Mac).
2. Search **VEX Robotics** and install the one published by **VEX Robotics**.
3. Search **C/C++** and install the one published by **Microsoft**.
4. **Do not** install or enable the **PROS** extension for this project. It
   conflicts with the VEX extension. (If you already have it, right-click it →
   *Disable (Workspace)*.)

Opening the template later also pops up "This workspace has extension
recommendations". Clicking **Install** does steps 2–3 for you.

## 4. Get the code

Your coach will tell you which of these to use.

**A. Your team already has its own repo**
1. In VS Code: **View → Command Palette** (`Ctrl+Shift+P`) → type **Git: Clone** → Enter.
2. Paste the repo link (e.g. `https://github.com/<team-account>/<team-repo>`).
3. Pick a folder (e.g. `Documents/VEX`), then click **Open** when asked.
4. If VS Code asks "Do you trust the authors?", click **Yes**.

**B. Starting a new team repo from the template**
1. On the template's GitHub page, click **Use this template → Create a new repository**.
2. Name it after your team (e.g. `1234A-2026`) and click **Create**.
3. Clone your new repo as in option A.

## 5. First build (downloads the VEX toolchain)

1. With the project open, look at the bottom-left of VS Code for the VEX bar
   (the VEX logo, **Build**, **Download/Run** icons, and a slot number).
2. Click **Build** (the hammer). The first time, the VEX extension downloads the
   V5 SDK and compiler. This can take a few minutes. If it asks to update the
   SDK version, click **Yes**.
3. You should see `Build Finished` with no errors in the output panel.

If the VEX bar isn't there, make sure you opened the project **folder** (the
one with `makefile` in it), not a parent folder.

## 6. Download to the robot

1. Plug the V5 Brain into the computer with a micro-USB cable (or plug the
   controller in, if it's paired with the Brain).
2. The VEX bar should show the device as connected. If the extension says the
   Brain firmware (VEXos) is out of date, let it update. Don't unplug during the update.
3. Pick a program **slot** (1–8) by clicking the slot number.
4. Click **Download** to send the program, or **Run** to send and start it.
5. On the Brain, the program appears under that slot with the project name.

## 7. Day-to-day workflow

1. **Pull** before you start: click the Source Control icon (left bar) → `...` → **Pull**.
2. Make your changes.
3. **Build** often. Fix the first error first; later errors are often caused by it.
4. **Commit** when something works: Source Control → type a short message
   ("tuned turn PID", "added intake subsystem") → **Commit**.
5. **Sync/Push** so teammates get it.

## Troubleshooting

| Problem | Fix |
|---|---|
| No VEX bar at the bottom | Open the folder that contains `makefile`. Check the VEX Robotics extension is installed and enabled. |
| Red squiggles everywhere but it builds fine | Build once so the SDK downloads, then reload VS Code (**Command Palette → Developer: Reload Window**). |
| `make: command not found` or toolchain errors | Let the VEX extension finish downloading the SDK. If it keeps failing, reinstall the VEX extension. |
| Brain not detected | Try a different USB cable (some are charge-only) and a different USB port. Turn the Brain on. |
| PROS buttons show up | Disable the PROS extension for this workspace. |
| Robot drives backwards / spins | A motor is reversed. Fix it in `src/robot-config.cpp`. |

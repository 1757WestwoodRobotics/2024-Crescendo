# Robot Base

1757 base swerve robot

[Chief delphi form](https://www.chiefdelphi.com/t/frc-1757-wolverines-2022-2023-build-thread/416564)

[The Blue Alliance](https://www.thebluealliance.com/team/1757)

[Website](https://whsrobotics.org)

## Installation (for the average programmer)

### Visual Studio 2019 redistributable

[vc_redist.x64](https://aka.ms/vs/16/release/vc_redist.x64.exe)

### Python

[3.10+ amd64](https://www.python.org/downloads/release/python-31013/)

### VS Code

[VS Code](https://code.visualstudio.com)


### Install robotpy

1. **To save yourself from pain, run the following in your bash terminal:**
    ```bash
    py -3 --version #or python3 --version
    ```
    MAKE SURE IT IS **3.10** OR ABOVE (if not, use the link above to get it)

1. **Globally install pipenv (for the virual environment)**
     ```bash
     py -3 -m pip install -U pipenv
     ```
1. **Create virtualenv/sync from pipenv**

    Try to install the packages using the following command, which will automatically create a virtual environment:
    ```bash
    pipenv sync
    ```
    If that does not work, instead manually create the environment by running
    ```bash
    cd <path-to-mentorbot-repo>
    py -3 -m venv ./.venv # wait until this is done and the terminal prompt comes back
    pipenv sync
    ```
    Also, be sure not to upload it when committing to the team github (it should already be blocked in .gitignore)

1. **Update Dependancies**
   (must have internet connection)
   ```bash
   pipenv update
   ```
1. **Activate virtual environment**
   (must have internet connection)
   ```bash
   pipenv shell
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Deploy robotpy program**

    And that's it! On your own machine, open the simulator to see if everything worked!

   - To robot
     (must be connected to roboRIO)
     ```bash
     python -m robotpy deploy
     ```
   - To simulator
     ```bash
     python -m robotpy sim


### Steps to take when commiting

1. **Make sure Pylint, Black and Prettier are installed**

- Run Pylint in terminal and check for a help output. It should be autoinstalled with python.

```bash
pylint
```

- Run Black in terminal empty and look for "No path provided". If nothing happens, run the second command

```bash
black
python -m pip install -U black
```

2. **Make Pylint happy**

- Run Pylint on your files

```bash
pylint $(git ls-files "*.py")
```

- With the error "Method could be a function" on an isFinished in a command, add this command above it. If necessary, swap out "no-self-use" with whatever error it gives you at the end.

```bash
# pylint: disable-next=no-self-use
```

3. **Formatting with Black and Prettier**

- Run black on your files (autoformats). Also note that it can be configured to run on save of a file.

```bash
black .
```

-You also may need to format json files using Prettier. When opening a json file in VSCode, it should prompt you to download Prettier in a small window in the bottom right. If not, go to the extensions tab on the left and search "Prettier". The top result is it (about 19 million downloads)

4.  **Make sure it starts in sim and works as expected**

- Should be obvious but make sure to do it

1. **CODE FORMATTING PRACTICES**
   - for python we are using [black](https://github.com/psf/black)
   - for json we are using [prettier](https://prettier.io)
   - all JSON files must be alphabetized, you can use the [following extension for prettier](https://www.npmjs.com/package/prettier-plugin-sort-json)
   

### Advanced User Stuff (With robot electronics)
### FRC Game Tools

[FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#479842)

### FRC Radio Configuration Utility

[FRC Configuration Utility](https://firstfrc.blob.core.windows.net/frc2024/Radio/FRC_Radio_Configuration_24_0_1.zip)

### CTRE Phoenix

[Phoenix Tuner](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases)

## Setup

### roboRIO

1. Image the roboRIO
   [Imaging instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
1. Configure the roboRio
   | Item | Value |
   | - | - |
   | Team number | `1757` |
   | Firmware | `6.0.0f1` |
   | Image | `FRC_roboRIO_2021_v3.0 (note: outdated, just use a new version)` |
   | Static IP | `10.17.57.2` |
   | Subnet Mask | `255.255.255.0` |

### Run Phoenix Tuner

#### Update device firmware

- PDH
- FalconFX
- CANCoder
- Pneumatics Hub

#### Configure CAN devices

| Device              | Class   | Range   | ID             |
| ------------------- | ------- | ------- | -------------- |
| robo_rio            | core    | 0 - 9   | master (no ID) |
| pdh                 | core    | 0 - 9   | 0              |
| front_left_drive    | motors  | 10 - 29 | 10             |
| front_left_steer    | motors  | 10 - 29 | 11             |
| front_right_drive   | motors  | 10 - 29 | 12             |
| front_right_steer   | motors  | 10 - 29 | 13             |
| back_left_drive     | motors  | 10 - 29 | 14             |
| back_left_steer     | motors  | 10 - 29 | 15             |
| back_right_drive    | motors  | 10 - 29 | 16             |
| back_right_steer    | motors  | 10 - 29 | 17             |
| front_left_encoder  | sensors | 40 - 59 | 40             |
| front_right_encoder | sensors | 40 - 59 | 41             |
| back_left_encoder   | sensors | 40 - 59 | 42             |
| back_right_encoder  | sensors | 40 - 59 | 43             |

#### Configure network devices

| Device                  | IP Address   | Subnet Mask       |
| ----------------------- | ------------ | ----------------- |
| OpenMesh radio          | `10.17.57.1` | `???.???.???.???` |
| roboRIO                 | `10.17.57.2` | `255.255.255.000` |
| Driver Station (laptop) | `10.17.57.5` | `255.000.000.000` |
1. **Download python for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download-python
   ```
1. **Download robotpy modules for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Install python on roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install-python
   ```
1. **Upload robotpy modules to roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))


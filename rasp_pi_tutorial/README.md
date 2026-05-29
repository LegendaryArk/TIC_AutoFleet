# Use Raspberry Pi to Control Your Robot
You have the option to run the client code on your local computer or on the Raspberry Pi. Running on the Raspberry Pi allows the robot to be operated wirelessly. It also enables additional functions like local computer vision process. But this requires a bit more setup. This tutorial will walk you through the steps to set up your Raspberry Pi for running the source code.

## Usage Description: 
The raspberry pi essentially replaces your laptop to plug into the PRIZM controller. It will run the same client.py code that you would run on your laptop. It also enables for local processing on the pi, such as computer vision processing through a camera module. For a documentation of useful computer vision for the robot, click [here].

**TL, DR for running**: launch central-arbiter.py in your computer, client.py on the raspberry pi, and prizm firmware on the prizm controller. The raspberry pi will communicate with the prizm controller through a USB cable, and communicate with your computer through the network.

## Note:
- Please **speak to a co-op student or a TA** if you are interested in controlling one or more of your robots with Raspberry Pi. They need to provide you with the raspberry pi, credentials to use the pi, a generated authentication key, and hardware to mount the pi on the robot. Note **inventory is limited**, so please reach out early if you are interested in using the raspberry pi.

- You will also be on an alternate network to allow direct IP pinging. Ensure you obtain credential for the network as well.

- After obtaining the raspberry pi and its kits, the steps below are for setting up the "sync" between your local computer and the Raspberry Pi. This allows you to edit code on your local computer and have it automatically update on the Raspberry Pi, which is much more convenient than editing code directly on the Pi.

## Setting Up Tailscale:
1. Download Tailscale by going to the following website: https://tailscale.com/docs/install/windows
2. Activate auth key with `tailscale up --auth-key=tskey-auth-your-key-here`. If needed, use `sudo`.
3. By doing `tailscale status`, you can see the other nodes on the server. You will see your Pi connnected to the server; if the Pi is disconnected, **get a TA**
4. SSH into your Pi:
    - The Pi Provided to you should have a piece of paper with the following information: IP adderess, username, and password
    - Open a terminal (either through VS code or your computer's own terminal) and type `ssh (username)@(IP address)` (replace with the actual username and IP address provided to you)
    - If prompted, enter the password. 
  For example:
    ```bash
    ~ tailscale up --auth-key=tskey-auth-k123e657fgyhuoji68t79huionji76fy8gu
    ~ tailscale up         //do this if tailscale status returns an error
    ~ tailscale status

    100.Your.Ip.Address   YourLaptop       skaushik@  windows  -
    100.Your.Pi.Ip        ideasclinicpi#   skaushik@  linux    -

   ~ ssh (username)@(IP address)
    ```
    -  You should now be logged into the Pi through the terminal.

5. Now you can setup a folder that your code will live in.
    ```bash
    # 1. Navigate to the home directory (the safest place for code)
        cd ~
    # 2. Create a new folder (e.g., 'robot_project')
        mkdir robot_project
    # 3. Enter that folder
        cd robot_project
    ```

## Setting Up the "Sync"
1. Install: Search for the SFTP extension in VS Code (the one by Natizyskane is popular) and install it.
<img src="sftp.png" width="50%" alt="Description">

2. Initialize: Press Ctrl + Shift + P and type SFTP: Config.
3. Configure: Your .vscode/sftp.json. Paste the following code into that file.
```json
{
    "name": "Robot-Pi-Sync", // Name of the connection 
    "host": "10.37.x.x", // Your Pi's IP (given on paper)
    "protocol": "sftp",
    "port": 22,
    "username": "XX", //Pi username (given on paper)
    "remotePath": "/home/pi/robot_project", //The folder on the Pi where you want to sync your code
    "uploadOnSave": true,
    "ignore": [
        ".vscode",
        ".git",
        ".env" 
    ]
}
```
4. Replace the "host", "username" fields in the sftp.json file with the actual information provided to you.
5. Now, update the "remotePath" field in your sftp.json file to match the path of the folder you just created. simply copy the path shown in your terminal (after you cd into the folder) and paste it into the "remotePath" field. For example, if your terminal shows that you are in `"/home/pi/robot_project"`, then your "remotePath" should be `"/home/pi/robot_project"`. Ensure the quotes around the path are preserved when you paste it in, and forward slash.

6. ctrl+s in the json file to save the sftp information 

## Prepare for syncing
Be sure you are SSHed into the Pi through the terminal before proceeding with the steps below. 
`ssh (username)@(IP address)`

1. clone the entire repository from github to your local computer if you have not already, and open the folder in VS code.

2. right click on the **Autonomous_Fleets** folder. Scroll down on the list of commands and click sync local-remote. This folder will be synce to the pi at the location specified in the "remotePath" field of your sftp.json file.
    - alternatively, you can also press Ctrl + Shift + P and type SFTP: Sync Local -> Remote to achieve the same result.This will sync the entire challenge folder

3. navigate to the folder on the Pi through the terminal to ensure the files are there. For example, if your remotePath is "/home/pi/robot_project", you can type `cd /home/pi/robot_project` and then `ls` to list the files in that folder. You should see the same files that are in your local folder.

## Workflow for SFTP Sync

Ensure you are SSHed into the Pi. Also check the COM port your prizm controller connects to and change robot_a.env if needed.
    - to check COM port: unplug the prizm controller, run `ls /dev/ttyUSB* /dev/ttyACM*` to list all COM ports(could say no such directory if nothing is plugged in), plug the prizm controller back in, run `ls /dev/ttyUSB* /dev/ttyACM*` again and look for the new COM port that appears. This is the COM port your prizm controller is connected to. Update robot_a.env with this COM port information. For example, if the new COM port is "/dev/ttyUSB0", then you would set `LOCAL_SERIAL_PORT= /dev/ttyUSB0` in robot_a.env.

1. Edit code on your local computer in VS code.
2. Press Ctrl + S to save the file. This will automatically upload the file to the Pi.
3. in the bottom of VScode, select the Output tab, and select SFTP from the dropdown menu. Look for the "SFTP:uploaded" message to confirm that your file has been uploaded to the Pi. If you see an error message instead, please check your sftp.json configuration and ensure you are SSHed into the Pi through the terminal.
3. Run the code on the Pi through the terminal. For example, if you have a Python script called "main.py", you can run it by typing `python main.py` in the terminal while you are in the correct directory.

**Note: if you are making changes to the prizm firmware code, we highly recommend plugging your computer directly to the controller to flash the firmware, and use the above tutorial to modify codes on raspberry pi only.**

Read this document here if you absolutely need to upload the firmware to the robot through the raspberry pi.

# Headless Robotics Setup: Pi to TETRIX PRIZM
This documentation covers setting up an **Arduino CLI** environment on a Raspberry Pi. This allows you to program a TETRIX PRIZM controller directly from the Pi via Remote-SSH, skipping the need for a heavy Desktop GUI.

## 1. Install the Arduino CLI
Since we are skipping the Desktop GUI, we install the command-line interface directly.

* **Download and Install:** Run this in your VS Code terminal:
    ```bash
    curl -fsSL [https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh](https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh) | sh
    ```
* **Make it Global:** Move the tool to a system folder so you can run it from any directory:
    ```bash
    sudo mv ./bin/arduino-cli /usr/local/bin/
    ```
* **Verify Installation:**
    ```bash
    arduino-cli version
    ```
---

## 2. Transfer & Install the PRIZM Library
The PRIZM library is a 3rd-party dependency. Use this "Drag-and-Drop" method to move it to the Pi.

### A. Local Transfer
1.  **Download:** On your main laptop, download the `TETRIX_PRIZM.zip` from the official Pitsco site.
2.  **Open VS Code Explorer:** Ensure you have your project folder open in the VS Code sidebar.
3.  **Drag-and-Drop:** Drag the `.zip` file from your laptop's file manager (Finder or Explorer) directly into the VS Code sidebar.
    * *VS Code will automatically upload the file to the Pi's disk via your SFTP/SSH connection.*

### B. CLI Installation
By default, the CLI blocks ZIP installs for security. You must unlock it first:
1.  **Unlock Unsafe Installs:**
    ```bash
    arduino-cli config set library.enable_unsafe_install true
    ```
2.  **Install the ZIP:** Run this command (adjust the filename if yours has spaces):
    ```bash
    arduino-cli lib install --zip-path "./TETRIX_PRIZM.zip"
    ```
---

## 3. Configure Board & Cores
The PRIZM is an Arduino Uno clone. You need the AVR "drivers" to talk to it.

* **Update Index:** `arduino-cli core update-index`
* **Install AVR Core:** `arduino-cli core install arduino:avr`
* **Identify Port:** Plug in the PRIZM via USB and run:
    ```bash
    arduino-cli board list
    ```
    *Look for the port name, usually `/dev/ttyUSB0` or `/dev/ttyACM0`.*

---

## 4. Compile & Upload Workflow
The `arduino-cli` is folder-dependent. You must be inside the directory containing your `.ino` file.

### The "One-Step" Deployment
To check for errors and send the code to the robot in one go, use:
```bash
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno .
```
Summary of common commands:
| Task | Command |
| :--- | :--- |
| **Check Port** | `arduino-cli board list` |
| **Update Libraries** | `arduino-cli lib update-index` |
| **Check Pi Temp** | `vcgencmd measure_temp` |
| **Monitor Serial Data** | `arduino-cli monitor -p /dev/ttyUSB0 -s 115200` |

## 5. Configure Correct Ports 
The connection between devices is now setup as such:

Laptop -> Rasp Pi (through Tailscale network)

Rasp Pi -> PRIZM (through USB serial)

The system was initially configured for communication with a local Windows laptop. To run it using Raspberry Pi OS and Tailscale, update the `.env` file and device settings as follows

- On the **laptop running `central-arbiter.py`**, retrieve its Tailscale IP:
     - `tailscale ip -4`. This will return your device IP address, for e.x. 100.12.345.67. Input this into SERVER_HOST_IP_ADDRESS
    
- Ensure USE_WIFI_BRIDGE=false
  
- On Raspberry Pi OS, USB serial devices typically appear as: `/dev/ttyUSB#` and not `COM#`
    - So change LOCAL_SERIAL_PORT="/dev/ttyUSB0" to reflect the serial port that is connected to the PRIZM


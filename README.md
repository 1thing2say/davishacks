## Install

```bash
pip install -r requirements.txt
```

## Setup

Copy the example env file and fill in your key:

```bash
cp .env.example .env
# edit .env and paste your Gemini API key
# get one free at https://aistudio.google.com
```

## Run

```bash
export $(cat .env) && python3 robot_pi.py
```

Then open the URL printed in the terminal (e.g. `http://192.168.1.42:8080`) in any browser.

## Controls

- **D-pad** — manual drive (hold to move, release to stop)
- **Speed slider** — adjust motor speed (0–255)
- **Capture** — take a photo from the ESP32-CAM
- **Gemini** — start AI autonomous mode (prompts for a goal, drives every 2.5s)

## Raspberry Pi setup

The script is designed to run on a Raspberry Pi mounted on the robot:

```bash
# copy files to the Pi
scp robot_pi.py requirements.txt .env pi@<pi-ip>:~/robot/

# on the Pi
cd ~/robot
pip3 install -r requirements.txt
export $(cat .env) && python3 robot_pi.py
```

The Arduino serial port auto-detects as `/dev/ttyACM0` on Linux.

## Files

| File | Purpose |
|---|---|
| `robot_pi.py` | Main controller — web UI, serial bridge, Gemini integration |
| `robot_web.py` | Laptop web server with WiFi switching |
| `robot_console.py` | CLI controller |
| `capture_image.py` | Standalone image capture |
| `requirements.txt` | Python dependencies |
| `.env.example` | API key template |

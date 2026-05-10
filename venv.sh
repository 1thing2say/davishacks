#!/bin/bash
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
echo ""
echo "Virtual environment ready. Run: python3 robot_pi.py"
exec bash

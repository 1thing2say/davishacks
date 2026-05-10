#!/bin/bash
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
echo ""
[ -f .env ] || cp .env.example .env
export $(cat .env)
exec bash

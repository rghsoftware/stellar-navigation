# Stellar Navigation Dashboard

Web-based visualization and control interface.

## Features

- Real-time 3D spacecraft visualization (Three.js)
- Live telemetry displays
- Servo gimbal control
- WebSocket communication with cFS

## Setup

```bash
# Install dependencies with uv (faster than pip)
uv sync
```

## Running

```bash
# Run with uv
uv run python app.py

# Or activate the virtual environment
source .venv/bin/activate
python app.py
```

Access at: http://localhost:5000

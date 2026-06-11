# CezeriSim

CezeriSim is a packaged Unreal Engine 5 VTOL drone simulator. The 3D world and
(optionally) the flight physics run in the `CezeriSim.exe` Windows build, while
the flight controller — ArduPlane SITL — runs inside a Docker container and
talks to the sim over UDP/MAVLink.

## Repository layout

```
Windows/
├── CezeriSim.exe                  # Launcher for the packaged simulator
└── CezeriSim/
    ├── Binaries/                  # Game binaries and runtime DLLs
    ├── Content/Paks/              # Packaged game content
    ├── Scripts/                   # SITL Docker setup + flight/control scripts
    │   ├── Dockerfile             # Builds the ArduPlane SITL image
    │   ├── docker-compose.yml     # Runs the SITL container
    │   └──start_sitl_docker.py   # Starts/stops SITL (resolves active vehicle)
    └── Vehicles/                  # Vehicle configs (params, mechanical/electrical)
        └── active_vehicle.txt     # Selects which vehicle config is used
```

## Prerequisites

### 1. Docker Desktop

The ArduPilot SITL flight controller runs in a Linux container, so you need
Docker Desktop for Windows:

1. Download Docker Desktop from <https://www.docker.com/products/docker-desktop/>.
2. Run the installer and keep the default **WSL 2 backend** option enabled
   (if prompted to install/enable WSL 2, accept it).
3. Restart your machine if the installer asks you to.
4. Start Docker Desktop and wait until the whale icon in the system tray
   reports "Docker Desktop is running".
5. Verify from a terminal:

   ```powershell
   docker --version
   docker compose version
   ```


## First-time setup

Build the SITL Docker image once (clones and compiles ArduPlane, takes a while):

```powershell
cd Windows\CezeriSim\Scripts
docker compose build
```

## Running the simulator

### Manual workflow

1. Start the SITL container:

   ```powershell
   python Windows\CezeriSim\Scripts\start_sitl_docker.py
   ```

   Useful flags: `--vehicle <name>` (override active vehicle), `--build`
   (rebuild image first), `--stop`, `--logs`, `--list`.

2. Launch the simulator: run `Windows\CezeriSim.exe`.

3. Connect a ground station or script over MAVLink:

   | Port | Purpose |
   |------|---------|
   | `tcp:localhost:5760` | MAVLink SERIAL0 — control scripts / MAVProxy |
   | `tcp:localhost:5762` | MAVLink SERIAL1 — visualizer bridge |
   | `udp 9003` | JSON physics from UE → ArduPilot |

4. Stop SITL when done:

   ```powershell
   python Windows\CezeriSim\Scripts\start_sitl_docker.py --stop
   ```

## Vehicles

Vehicle configurations live in `Windows/CezeriSim/Vehicles/<name>/`
(`params.parm`, `mechanical.json`, `electrical.json`). The active vehicle is
selected by `Vehicles/active_vehicle.txt`. Each vehicle's `mechanical.json`
declares its physics `backend`:

- `ue_physics` — Unreal Engine simulates the physics (ArduPilot JSON model)
- `ap_native` — ArduPilot's built-in quadplane physics

To create a new vehicle, copy `Vehicles/_template/` and edit the files.
Do **not** edit `Scripts/vehicle.parm` directly — it is generated from the
active vehicle's `params.parm` by `start_sitl_docker.py`.

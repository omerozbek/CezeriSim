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
    │   ├── start_sitl_docker.py   # Starts/stops SITL (resolves active vehicle)
    │   └── control/
    │       └── servo_relay.py     # Relays AP servo output to UE (ue_physics mode)
    └── Vehicles/                  # Vehicle configs (params, mechanical/electrical)
        └── active_vehicle.txt     # Selects which vehicle config is used
```

## Getting the repository

The packaged simulator binaries (`CezeriSim.exe`, `.pak`/`.ucas` content) are
stored in **Git LFS**. You must clone with Git LFS installed — do **not** use
GitHub's "Download ZIP" button, it gives you tiny LFS pointer files instead of
the real binaries and the simulator will not launch.

### 1. Install Git + Git LFS

- **Windows:** install [Git for Windows](https://git-scm.com/download/win)
  (Git LFS is included; keep the default "Checkout Windows-style" option —
  the repo forces correct line endings itself).
- **macOS:** `brew install git git-lfs`
- **Linux:** `sudo apt install git git-lfs` (or your distro's equivalent)

Then, once per machine:

```
git lfs install
```

### 2. Clone only your platform

Each platform build is ~600 MB of LFS binaries. A plain `git clone` downloads
**all** platforms — instead, clone with your platform selected so only your
files are downloaded. Replace `Windows` with `Linux` or `Mac` in **both**
places:

```
git clone --filter=blob:none --no-checkout https://github.com/omerozbek/CezeriSim.git
cd CezeriSim
git sparse-checkout set Windows
git config lfs.fetchinclude "Windows/**"
git checkout main
```

What this does:

- `--no-checkout` + `sparse-checkout set` — only your platform's folder (plus
  top-level files like this README) appears in the working tree.
- `lfs.fetchinclude` — Git LFS downloads only your platform's binaries, now
  and on every future `git pull`.
- `--filter=blob:none` — remaining git objects are fetched on demand instead
  of up front.

To add another platform later:

```
git sparse-checkout add Linux
git config lfs.fetchinclude "Windows/**,Linux/**"
git checkout main
```

If you don't care about download size, a plain `git clone` (with Git LFS
installed) still works and gets everything.

### 3. Verify

The game binary must be real content, not an LFS pointer (should be
~300 MB, not a few hundred bytes):

```powershell
Get-Item Windows\CezeriSim\Binaries\Win64\CezeriSim.exe   # Windows
ls -lh Linux/CezeriSim/Binaries/Linux/CezeriSim           # Linux/Mac: du -sh Mac/
```

If it is tiny, run `git lfs pull` inside the repo.

## Prerequisites

### 1. Python 3.10+

`start_sitl_docker.py` (and the servo relay it launches) run on Windows with
plain Python — no extra pip packages required.

1. Install Python 3.10 or newer from <https://www.python.org/downloads/>
   (check **"Add python.exe to PATH"** in the installer), or:

   ```powershell
   winget install Python.Python.3.12
   ```

2. Verify from a **new** terminal:

   ```powershell
   python --version
   ```

### 2. Docker Desktop

The ArduPilot SITL flight controller runs in a Linux container, so you need
Docker. On Linux install Docker Engine + the compose plugin
(<https://docs.docker.com/engine/install/>); on macOS use Docker Desktop for
Mac. On Windows, install Docker Desktop for Windows:

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

   For vehicles with the `ue_physics` backend this also starts
   `Scripts/control/servo_relay.py` in the background, which forwards
   ArduPilot's servo output (UDP 9006) to the simulator (UDP 127.0.0.1:9002).
   Leave it running — without it the aircraft will not respond.

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

## Troubleshooting

**`exec /home/ardupilot/entrypoint.sh: no such file or directory`** when the
container starts: the shell scripts were checked out with Windows (CRLF) line
endings and baked into a stale image. Update to the latest commit, then force
a clean rebuild:

```powershell
cd Windows\CezeriSim\Scripts
docker compose build --no-cache
```

**Simulator launches but the aircraft never moves / never arms** (ue_physics
vehicles): the servo relay is not running. Make sure you started SITL via
`start_sitl_docker.py` (not `docker compose up` directly) and that
`Scripts/control/servo_relay.py` exists — the launcher prints a
`Servo relay : pid ...` line when it is active.

**`CezeriSim.exe` is only a few hundred bytes / won't start**: the clone was
made without Git LFS (or via "Download ZIP"). Install Git LFS and run
`git lfs pull` inside the repo.

## Maintainers: adding or updating a platform build

The game reads `Vehicles/` and is launched alongside `Scripts/` from inside
its own tree, so every platform build ships its own copy of both. Treat
`Windows/CezeriSim/{Scripts,Vehicles}` as the canonical copy and sync it into
other platform trees whenever it changes.

**Updating the Windows build:** copy the new packaged build *over* the
existing `Windows/` tree (do not delete it first — `Scripts/` and `Vehicles/`
are not part of a UE export and would be lost). Check `git status`, then
commit.

**Adding a Linux or Mac build** (from a Windows machine):

1. Stage the packaged build to a top-level `Linux/` or `Mac/` folder,
   mirroring the `Windows/` layout.
2. Copy `Scripts/` and `Vehicles/` from `Windows/CezeriSim/` into the new
   platform's `CezeriSim/` folder.
3. Verify every large binary resolves to LFS **before** committing —
   GitHub hard-rejects non-LFS files over 100 MB and fixing that later
   requires rewriting history:

   ```powershell
   git check-attr filter -- Linux/CezeriSim/Binaries/Linux/CezeriSim
   # must print: filter: lfs
   ```

4. Git on Windows does not record the executable bit. After `git add`,
   mark the launcher and game binaries executable in the index:

   ```powershell
   git update-index --chmod=+x Linux/CezeriSim.sh Linux/CezeriSim/Binaries/Linux/CezeriSim
   ```

5. Commit and push, then verify on a real Linux/Mac machine: fresh clone,
   `ls -l` shows the binaries as executable, and the game finds its
   `Vehicles/` configs.

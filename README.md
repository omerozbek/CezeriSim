# CezeriSim

CezeriSim is a packaged Unreal Engine 5 VTOL drone simulator. The 3D world and
(optionally) the flight physics run in the packaged game build for your
platform (`Windows/`, `Mac/`, or `Linux/`), while the flight controller —
ArduPlane SITL — runs inside a Docker container and talks to the sim over
UDP/MAVLink.

## Quick start: clone only your OS (sparse checkout)

Each platform ships as its own ~1 GB build (`Windows/`, `Mac/`, and `Linux/`),
and a plain `git clone` pulls **all** of them. Instead, sparse checkout grabs
only your platform's folder (plus top-level files like this README). With
**Git + Git LFS installed** (run `git lfs install` once per machine), pick
your OS and run:

```bash
# Choose ONE platform and use the SAME name in both marked lines:
#   Windows  |  Mac  |  Linux
git clone --filter=blob:none --no-checkout https://github.com/omerozbek/CezeriSim.git
cd CezeriSim
git sparse-checkout set Windows            # <-- your OS
git config lfs.fetchinclude "Windows/**"   # <-- same OS
git checkout main
```

Only your platform's LFS binaries are downloaded, now and on every future
`git pull`. To add another platform later, verify the download, or read what
each flag does, see [Getting the repository](#getting-the-repository) below.

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

Linux/                             # Same layout, Linux binaries
├── CezeriSim.sh                   # Launcher (game binary: CezeriSim/Binaries/Linux/CezeriSim)
└── CezeriSim/{Binaries, Content, Scripts, Vehicles}

Mac/
└── CezeriSim.app                  # Scripts/ + Vehicles/ live inside the app bundle
                                   # (Contents/UE/CezeriSim/)
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

The game binary must be real content, not an LFS pointer (should be tens to
hundreds of MB, not a few hundred bytes):

```powershell
Get-Item Windows\CezeriSim\Binaries\Win64\CezeriSim.exe   # Windows
```

```bash
ls -lh Mac/CezeriSim.app/Contents/MacOS/CezeriSim          # macOS
ls -lh Linux/CezeriSim/Binaries/Linux/CezeriSim             # Linux
```

If it is tiny, run `git lfs pull` inside the repo. `du -sh Mac/` is **not**
a substitute — it sums the whole tree, so a real multi-hundred-MB `Content/`
pak next to a tiny LFS-pointer binary still looks "big enough" while the app
itself won't launch.

### 4. macOS only: fix the code signature

The `.app` is packaged on Windows, which breaks its code signature (Apple
Silicon refuses to run an app with an invalid signature). After every clone
or pull that changes `Mac/`, re-sign it locally:

```bash
chmod +x Mac/CezeriSim.app/Contents/MacOS/CezeriSim
codesign --force --deep --sign - Mac/CezeriSim.app
```

This is a local, ad-hoc signature (not notarized), so macOS Gatekeeper will
still flag the app on first launch — right-click it and choose **Open** once
to allow it. See **Known issues on macOS** below for why this step exists
and isn't baked into the repo yet.

### 5. Linux only: first launch

The build needs a **Vulkan-capable GPU and driver** (the renderer targets
Vulkan SM6 — recent Mesa/RADV, NVIDIA or Intel ANV drivers all qualify;
`vulkaninfo --summary` verifies). The launcher and game binary are committed
with the executable bit set, but if your checkout somehow lost it:

```bash
chmod +x Linux/CezeriSim.sh Linux/CezeriSim/Binaries/Linux/CezeriSim
./Linux/CezeriSim.sh
```

The in-game **Start/Stop Docker** buttons open the SITL launcher in your
terminal emulator (x-terminal-emulator, gnome-terminal, konsole,
xfce4-terminal, or xterm — whichever is found first; with none installed the
containers still start, just without a visible console). They invoke
`python3` and `docker compose`, so the prerequisites below apply the same
as when running `start_sitl_docker.py` by hand.

## Prerequisites

### 1. Python 3.10+

`start_sitl_docker.py` (and the servo relay it launches) run on plain
Python on every platform — no extra pip packages required.

**Windows:**

1. Install Python 3.10 or newer from <https://www.python.org/downloads/>
   (check **"Add python.exe to PATH"** in the installer), or:

   ```powershell
   winget install Python.Python.3.12
   ```

2. Verify from a **new** terminal:

   ```powershell
   python --version
   ```

**macOS:** `brew install python3` (or install from python.org), then verify
with `python3 --version`.

**Linux:** install via your distro's package manager (e.g.
`sudo apt install python3`), then verify with `python3 --version`.

### 2. Docker Desktop

The ArduPilot SITL flight controller runs in a Linux container, so you need
Docker. On Linux install Docker Engine + the compose plugin
(<https://docs.docker.com/engine/install/>); on macOS use Docker Desktop for
Mac, or Colima (see the **macOS + Colima** note below). On Windows, install
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

> **macOS + Colima:** if you use [Colima](https://github.com/abiosoft/colima)
> instead of Docker Desktop, start it with the gVisor-based port forwarder —
> Colima's default forwarder tunnels over SSH, which only carries TCP and
> silently drops the UDP JSON-physics traffic (port 9003) between UE and
> ArduPilot. Without this flag the simulator connects fine over MAVLink (TCP)
> but the aircraft never moves, with ArduPilot endlessly logging
> `No JSON sensor message received, resending servos`:
>
> ```bash
> colima start --port-forwarder grpc
> ```
>
> This is a one-time setting, persisted in `~/.colima/default/colima.yaml` as
> `portForwarder: grpc`. Docker Desktop for Mac does not need this — its
> vpnkit forwarder already carries UDP.


## First-time setup

Build the SITL Docker image once (clones and compiles ArduPlane, takes a while).
Use your platform's `Scripts` folder — same command everywhere:

```powershell
cd Windows\CezeriSim\Scripts     # Windows
docker compose build
```

```bash
cd Mac/CezeriSim.app/Contents/UE/CezeriSim/Scripts    # macOS
cd Linux/CezeriSim/Scripts                             # Linux
docker compose build
```

## Running the simulator

### Manual workflow

1. Start the SITL container:

   ```powershell
   python Windows\CezeriSim\Scripts\start_sitl_docker.py    # Windows
   ```

   ```bash
   python3 Mac/CezeriSim.app/Contents/UE/CezeriSim/Scripts/start_sitl_docker.py   # macOS
   python3 Linux/CezeriSim/Scripts/start_sitl_docker.py                            # Linux
   ```

   Useful flags: `--vehicle <name>` (override active vehicle), `--build`
   (rebuild image first), `--stop`, `--logs`, `--list`, and fleet mode
   `--drones N` / `--vtols M` (see **Fleet mode** below).

   For vehicles with the `ue_physics` backend this also starts
   `Scripts/control/servo_relay.py` in the background, which forwards
   ArduPilot's servo output (UDP 9006) to the simulator (UDP 127.0.0.1:9002).
   Leave it running — without it the aircraft will not respond.

2. Launch the simulator:

   - **Windows:** run `Windows\CezeriSim.exe`.
   - **macOS:** `open Mac/CezeriSim.app` (first launch: right-click →
     **Open** to bypass Gatekeeper if the app isn't signed with a
     Developer ID — see **Known issues on macOS** below).
   - **Linux:** run `Linux/CezeriSim.sh` (needs Vulkan drivers — see
     **Linux only: first launch** above).

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

### Fleet mode (multiple vehicles)

`start_sitl_docker.py --drones N --vtols M` starts a matching number of SITL
instances. **The UE in-game menu's drone/VTOL count is independent of this —
it is not read from Docker, and Docker does not read it either.** Set them to
the *same* number yourself:

- Menu count > SITL count: extra aircraft in UE sit forever at
  `Waiting for ArduPilot` with nothing to connect to.
- Menu count < SITL count (or SITL started with no `--drones`/`--vtols` at
  all, i.e. legacy single-vehicle mode): the legacy container's fixed JSON
  port can coincidentally land on drone 1's port, so **drone 1 flies while
  the rest report `connection refused`** — a partially-working setup that is
  easy to misread as a general bug rather than a count mismatch.

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

## Known issues on macOS

The simulator and SITL themselves are solid on Apple Silicon — these are
packaging/build gaps specific to producing the `.app` from a Windows
machine, not runtime bugs. Tracked here until a Mac build machine closes
them properly:

- **Broken code signature.** Zipping/committing the `.app` on Windows
  invalidates its signature seal. Run the re-sign command in **Getting the
  repository → step 4** above after every clone/pull. The permanent fix is
  to package the Mac build as a signed `.zip`/`.dmg` (via `ditto -c -k
  --keepParent`, which preserves the signature) instead of committing the
  raw `.app` tree — that needs to happen on an actual Mac.
- **Settings/Models menu shows the C++ fallback UI**, not the designed
  widget (`WB_SettingsMenu` / `WB_ModelsMenu` not found in the cooked
  content). Doesn't block functionality, just looks different from the
  Windows build. Needs those widgets cooked into the Mac content package —
  only possible from a Mac (or a Mac cook agent), not from Windows.
- **Misleading `[JSONBridge] GPS home for ALL ArduPilot instances` log
  line.** It prints a single value labeled "ALL instances" once per bridge
  (so a fleet prints several different "ALL" values), and that value can be
  ~600 m off the real per-vehicle home shown in the `[Fleet] Registered
  Drone` line just above it — looks like a leftover from a legacy
  single-vehicle code path. Harmless: `start_sitl_docker.py` computes each
  vehicle's actual home correctly from its own params and ignores this
  line. Don't hand-copy this logged `--home` value; use `--vehicle` or
  `--home` on `start_sitl_docker.py` instead. This message is emitted by
  the compiled Unreal Engine C++ (not by anything in this packaged repo),
  so fixing the text requires the UE source project.

## Maintainers: adding or updating a platform build

The game reads `Vehicles/` and is launched alongside `Scripts/` from inside
its own tree, so every platform build ships its own copy of both. Treat
`Windows/CezeriSim/{Scripts,Vehicles}` as the canonical copy and sync it into
other platform trees whenever it changes.

**Updating the Windows build:** copy the new packaged build *over* the
existing `Windows/` tree (do not delete it first — `Scripts/` and `Vehicles/`
are not part of a UE export and would be lost). Check `git status`, then
commit.

**Adding or updating a Linux or Mac build** (from a Windows machine — the
Linux build is cross-compiled there; see the UE source project's
`docs/HOW_TO_RUN.md` "Package a Linux Build" for the toolchain + command):

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
   git update-index --chmod=+x Mac/CezeriSim.app/Contents/MacOS/CezeriSim
   ```

   This step was missed for the first Mac build — always check
   `git ls-files -s <path-to-binary>` shows mode `100755`, not `100644`,
   before committing.

5. If staging from a `.zip` exported on Windows, make sure macOS
   AppleDouble/resource-fork junk didn't get swept in with it:

   ```powershell
   git status --porcelain Mac/ | Select-String "__MACOSX"
   ```

   `__MACOSX/` is in `.gitignore`, but only `git add`-ing the intended
   subfolders (not the whole zip extraction root) avoids it in the first
   place.

6. Commit and push, then verify on a real Linux/Mac machine: fresh clone,
   `ls -l` shows the binaries as executable, `codesign --verify --deep
   --strict Mac/CezeriSim.app` succeeds (re-sign and re-export if not — see
   **Known issues on macOS**), and the game finds its `Vehicles/` configs.

# Mac field workstation

This is the delivery-computer boundary. It owns the browser workflow, the
version catalog, resumable robot-to-Mac recording transfer, Mac-to-cloud GLIM
jobs, map review, and selected map deployment back to the robot. It contains no
ROS 2, FAST-LIO, Nav2, or Unitree motion process.

On the commissioned Mac, run the native process so its wired-LAN and SSH
traffic use the Mac network directly:

```bash
make workstation
```

For a portable container delivery on another workstation:

```bash
docker compose -f deployment/workstation/compose.yaml up --build -d
```

Open <http://127.0.0.1:8080>. Runtime artifacts stay in
`workstation-data/`; the robot and cloud only receive the artifacts required by
their narrow contracts. Compose mounts only the Mac's `id_ed25519` and
`known_hosts` files for the workstation-to-cloud adapter. It intentionally does
not mount the macOS SSH config because Linux does not support `UseKeychain`.
The robot never mounts a cloud key.

The page is a guided workflow rather than a collection of independent command
buttons. A map publication button appears only when the selected Mac map is not
the robot's prepared candidate. Once published, the page moves directly to
localization/Nav2 startup and then patrol. See `docs/FIELD_WORKSTATION_GUIDE.md`
for the exact field sequence and recovery actions.

Robot publication is deliberately two-phase so the dog does not wait for a
five-gigabyte image export. Run `prepare-robot-release` while the dog is off,
then use `stage-robot-release` only after the wired SSH endpoint is online.

Port 8080 is published to the Mac while outbound robot/cloud traffic uses the
normal container network.

For local UI development without Docker or a robot, use `make demo`. The
`workstation-demo` target still expects an Edge Agent at the configured robot
URL but replaces cloud GLIM with a deterministic local map worker.

# Robot deployment boundary

This directory defines one systemd service and one primary Humble ARM64
container. It does not replace the three existing SaaS services.

The robot container owns realtime sensing, recording, localization, navigation
and motion only. The Mac workstation owns GitHub, image publication, historical
catalogs and cloud GLIM orchestration. Therefore the robot release mounts no
cloud SSH key and its map worker defaults to `none`.

Inside the primary container, the Site Console HTTP process is not a process
supervisor. `gogoguard-navigation-supervisor` is the unique owner of the
Unitree receiver and localization/Nav2 runtime. Restarting the web process does
not stop motion orchestration or create duplicate ROS graphs; the container
entrypoint waits for the supervisor socket before exposing the Edge Agent.

The 2026-08-05 native ARM64 build and image smoke test succeeded. Livox SDK2,
Livox Driver2 and FAST-LIO are compiled in the image; Nav2 including MPPI and
Collision Monitor is installed from the Humble ARM64 repository. Exact source
revisions are recorded under `dependencies/`.

The connected production target was read-only audited as Orin NX 16 GB,
Ubuntu 20.04.5 / L4T R35.3.1 with Docker 24. The Humble userspace stays inside
the container. MID-360 at `192.168.1.161` and its robot-side host address
`192.168.1.5` are both present. Robot installation still requires sudo because
the `unitree` account cannot access the Docker daemon directly.

The same primary container also runs the pinned MediaMTX v1.20.0 ARM64 binary.
It pulls the commissioned Z1Pro H.264 RTSP stream on demand and exposes the LAN
WebRTC player on TCP 8889 with ICE on TCP/UDP 8189. This is a remux-only live
preview; it does not own patrol recording or inspection evidence.

`install-release` validates the transferred image archive, loads it, and
installs the runtime files. Installation is separate from activation so a bad
release cannot silently replace a running one. Once the stationary receipts
pass, the production policy is to keep this service running and enable it at
boot. Explicit Docker stop handling keeps later upgrades under systemd control.

The first robot deployment is an explicit experiment with these receipts:

1. image digest and container health;
2. Livox and IMU topic rate for five minutes;
3. live point cloud and trajectory visible in the site console;
4. a sealed stationary recording;
5. cloud GLIM job ID and returned artifacts.

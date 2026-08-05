# Robot deployment boundary

This directory defines one systemd service and one primary Humble ARM64
container. It does not replace the three existing SaaS services.

The 2026-08-05 native ARM64 build and image smoke test succeeded. Livox SDK2,
Livox Driver2 and FAST-LIO are compiled in the image; Nav2 including MPPI and
Collision Monitor is installed from the Humble ARM64 repository. Exact source
revisions are recorded under `dependencies/`.

The connected production target was read-only audited as Orin NX 16 GB,
Ubuntu 20.04.5 / L4T R35.3.1 with Docker 24. The Humble userspace stays inside
the container. MID-360 at `192.168.1.161` and its robot-side host address
`192.168.1.5` are both present. Robot installation still requires sudo because
the `unitree` account cannot access the Docker daemon directly.

`install-release` validates the transferred image archive, loads it, and
installs the runtime files. It deliberately does not start or enable the
service. The first start must remain supervised and stationary; boot-time
activation comes only after the real sensor receipts pass.

The first robot deployment is an explicit experiment with these receipts:

1. image digest and container health;
2. Livox and IMU topic rate for five minutes;
3. live point cloud and trajectory visible in the site console;
4. a sealed stationary recording;
5. cloud GLIM job ID and returned artifacts.

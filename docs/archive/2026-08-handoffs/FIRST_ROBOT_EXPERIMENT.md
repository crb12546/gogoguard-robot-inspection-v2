# First robot experiment: observe before moving

The first real deployment does not command robot motion. The operator keeps the
robot externally powered and uses the Unitree remote only after the stationary
checks pass.

## Entry conditions

- the exact robot SSH address is recorded outside Git or in the operator's SSH
  config;
- an ARM64 image is built and its immutable digest recorded;
- MID-360 host and sensor IPs are confirmed for this robot;
- the old production services and rollback command are inventoried.

The Mac workstation must reach both the Edge Agent and the fixed cloud
`gogoguard-map-job` wrapper before the cloud step. Neither connection blocks
robot-side live sensing or recording.

## Acceptance sequence

1. Start the V2 container under systemd. After the stationary receipts pass,
   keep the production service running and enable it at boot.
2. Open the Mac workstation page and confirm it reports the robot online.
3. Observe Livox PointCloud2 and IMU rate for five stationary minutes.
4. Confirm `/Odometry` is finite and stationary drift is recorded.
5. Record 30 stationary seconds, stop, and verify the bundle hashes.
6. Let Mac resume/copy the sealed bundle, submit it to cloud GLIM and retain its
   job ID and artifacts in the workstation history.
7. Only then walk one short out-and-back loop with the human remote.

Failure at any step stops the experiment at that layer. It does not trigger an
unrelated rewrite of the UI, SaaS or route modules.

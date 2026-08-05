# Robot deployment boundary

This directory defines one systemd service and one primary Humble ARM64
container. It does not install or replace the robot runtime automatically.

Before first deployment, the ARM64 image must build successfully, Livox and
FAST-LIO must be present in that image, and the real topic names must be
observed on the robot. The current Dockerfile contains the application and ROS
recording dependencies, but third-party driver builds are deliberately not
claimed as complete yet. Their exact revisions are recorded in
`dependencies/robot_runtime.lock.json`.

The first robot deployment is an explicit experiment with these receipts:

1. image digest and container health;
2. Livox and IMU topic rate for five minutes;
3. live point cloud and trajectory visible in the site console;
4. a sealed stationary recording;
5. cloud GLIM job ID and returned artifacts.

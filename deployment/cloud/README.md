# Existing cloud GLIM adapter

V2 treats the existing Alibaba Cloud GLIM host as an external worker. The SSH
adapter uploads one sealed `recording_bundle.v1`, invokes one fixed remote
command, and expects `output/map.json`, `output/map.ply`, and
`output/overview.svg`.

The server is known to have Ubuntu 24.04, ROS 2 Jazzy, GLIM and the
PDAL/GDAL/Potree toolchain. A native PointCloud2+IMU smoke produced artifacts;
dynamic production mapping has not yet been proven.

`gogoguard-map-job` is the narrow V2 adapter over that installed capability.
It accepts exactly one immutable `/opt/go2/jobs/map-*/input` bundle, executes
the pinned single-session pipeline as the unprivileged `go2mapping` user, and
returns `map.json`, `map.ply`, `overview.svg`, and the GLIM build receipt under
`output/`. It refuses an existing output or unsafe path.

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
`output/`. It refuses an existing output or unsafe path. During execution it
emits `GOGOGUARD_PROGRESS <percent> <stage> <message>` markers; the Mac
workstation forwards those real cloud stages to the browser instead of showing
a fixed percentage.

`gogoguard-glim-editor` adds an isolated review session around GLIM's official
`map_editor`. Each session copies the immutable source dump, runs as the
unprivileged `go2mapping` user, and binds x11vnc/noVNC only to cloud loopback.
The Mac workstation creates an SSH local forward and opens the returned
`127.0.0.1` URL. After the operator saves into `SAVED_MAP`, the adapter uses
GLIM `offline_viewer --export_path` to create a new immutable map version; it
never overwrites the original map job.

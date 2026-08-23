# Existing cloud GLIM adapter

V2 treats the existing Alibaba Cloud GLIM host as an external worker. The SSH
adapter uploads one sealed `recording_bundle.v1`, invokes one fixed remote
command, and expects `output/map.json`, `output/map.ply`,
`output/trajectory-poses.json`, and `output/overview.svg`. The optimized-pose
companion preserves every GLIM timestamp and quaternion for deterministic
recording-checkpoint binding; it is not a robot runtime map artifact.

The server is known to have Ubuntu 24.04, ROS 2 Jazzy, GLIM and the
PDAL/GDAL/Potree toolchain. A native PointCloud2+IMU smoke produced artifacts;
dynamic production mapping has not yet been proven.

`gogoguard-map-job` is the narrow V2 adapter over that installed capability.
It accepts exactly one immutable `/opt/go2/jobs/map-*/input` bundle, executes
the pinned single-session pipeline as the unprivileged `go2mapping` user, and
returns `map.json`, `map.ply`, `trajectory-poses.json`, `overview.svg`, and the
GLIM build receipt under `output/`. It refuses an incomplete existing output
or unsafe path. If all five sealed artifacts already exist, a retry reuses them
so the workstation can recover from an interrupted cloud-to-Mac download
without rerunning or overwriting GLIM. During execution it
emits `GOGOGUARD_PROGRESS <percent> <stage> <message>` markers; the Mac
workstation forwards those real cloud stages to the browser instead of showing
a fixed percentage.

The route exported in `map.json` must cover the sealed recording start, not
merely the first pose that GLIM later decides is optimizable. If GLIM's first
optimized pose is more than 0.5 seconds after `session.started_at`, the cloud
exporter timestamp-matches the overlapping recording pose samples to the GLIM
timeline, fits one bounded planar rigid transform, converts and prepends the
missing prefix, and emits `routeCoverage` in both `map.json` and
`trajectory-poses.json`. The optimized quaternion timeline itself remains
unchanged and authoritative for checkpoint binding. Too few synchronized
samples, excessive fit residual, an unsafe join gap, or a route start outside
the 0.5-second recording boundary fails the cloud job instead of publishing a
truncated patrol route. The Mac repeats this validation after download.

`gogoguard-glim-editor` adds an isolated review session around GLIM's official
`map_editor`. Each session copies the immutable source dump, runs as the
unprivileged `go2mapping` user, and binds x11vnc/noVNC only to cloud loopback.
The Mac workstation creates an SSH local forward and opens the returned
`127.0.0.1` URL. After the operator saves into `SAVED_MAP`, the adapter uses
GLIM `offline_viewer --export_path` to create a new immutable map version; it
never overwrites the original map job.

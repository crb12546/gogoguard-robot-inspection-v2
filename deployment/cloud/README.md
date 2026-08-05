# Existing cloud GLIM adapter

V2 treats the existing Alibaba Cloud GLIM host as an external worker. The SSH
adapter uploads one sealed `recording_bundle.v1`, invokes one fixed remote
command, and expects `output/map.json`, `output/map.ply`, and
`output/overview.svg`.

The server is known to have Ubuntu 24.04, ROS 2 Jazzy, GLIM and the
PDAL/GDAL/Potree toolchain. A native PointCloud2+IMU smoke produced artifacts;
dynamic production mapping has not yet been proven. The fixed
`gogoguard-map-job` wrapper still needs to be installed and tested against a
real recording before `--map-worker ssh` is production-capable.

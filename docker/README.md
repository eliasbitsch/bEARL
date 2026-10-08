# Containers

One folder per SLAM system: `docker/<slug>/Dockerfile`. Pin the base image and the upstream commit,
so that every run can be reproduced. On a cluster without Docker, build an Apptainer image from it
(see `cluster/slurm_array.sh`). Systems installed via pip do not need a container.

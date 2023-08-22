# Container Maintenance

This document describes the entire structure of the RoboCup container definition.

## Building

To build the container, ensure you have a copy of the TIAGo Melodic container and then proceed with:

```bash
git clone https://github.com/lasr-at-home/containers.git ~/containers
cd ~/containers

# copy base container to tiago_pal_melodic.sif to current directory

sudo apptainer build robocup_container.sif robocup_container.def
```

:::caution

This should work with the open source container but it has not been battle-tested yet.

:::

### Debugging build process

You can save a lot of the entire build by redirecting all output to a file:

```bash
sudo apptainer build robocup_container.sif robocup_container.def 2>&1 | tee build.log
```

## Container

This section goes over how the container is built.

:::danger

This section is still being worked on.

:::

### Install packages from repositories

### Install Python 3.10 from source

### Configure global Python installation

### Create overlay workspace for additional ROS packages

### Perform clean up

# Container Maintenance

This document describes the entire structure of the RoboCup container definition as well as how to build it.

## Building

To build the container, ensure you have a copy of the TIAGo Melodic container and then proceed with:

```bash
git clone https://github.com/lasr-at-home/containers.git ~/containers
cd ~/containers

# copy base container to tiago_pal_melodic.sif to current directory

sudo apptainer build robocup_container.sif robocup_container.def
```

:::note

This also works with the open source container.

:::

### Debugging build process

You can save a lot of time debugging build errors by redirecting all output to a file:

```bash
rm build.log 2>/dev/null; sudo apptainer build robocup_container.sif robocup_container.def 2>&1 | tee build.log
```

### Testing new dependencies

If you'd like to test new dependencies without rebuilding the entire container, you can use the `stage2` definition file.

After building the RoboCup container, you can then run:

```bash
sudo apptainer build stage2.sif stage2.def

# .. with log:
rm build.log 2>/dev/null; sudo apptainer build stage2.sif stage2.def 2>&1 | tee build.log
```

## Container

This section goes over how the container is built.

We use the TIAGo Melodic container as a base which uses Ubuntu 18.04 as a base itself.

### Install packages from repositories

1. Ensure the container is up to date by updating and upgrading from PAL apt repositories.

2. Install additional apt packages

   | Package | Description |
   |:-:|---|
   | ros-melodic-audio-common | provides various ROS packages for capturing and playing back audio |
   | python3-numpy | numpy package for Python 3.6 |
   | python3-opencv | cv2 package for Python 3.6 |
   | libasound-dev<br/>libportaudio2<br/>libportaudiocpp0<br/>portaudio19-dev | libraries required to build PyAudio wheel |
   | ffmpeg | record, convert, and stream audio / video |

   :::caution

   Software packages on PAL repositories are quite old, there are situations where it is advisable to install software through alternative means.

   :::

3. Install Node.js 16

   All available distributions are [listed here](https://github.com/nodesource/distributions).

   :::caution
   
   Node.js 16 is the latest that may be installed for Ubuntu 18.04. Their build system is currently broken for Node.js 18 and later. When upgrading to noetic, Node.js should be pinned to LTS.
   
   :::

4. Install additional Python 3.6 packages

   | Package | Version | Description |
   |:-:|:-:|---|
   | rosnumpy | 0.0.5.2 | conversion helper between ROS and numpy † |
   | scipy | 1.5.4 | mathematics library |
   | black | 22.8.0 | Python code formatter |
   | scikit-build | 0.16.7 | build system for CPython C/C++/Fortran/Cython extensions using CMake |
   | scikit-learn | 0.24.2 | machine learning and data mining |
   | nvidia-ml-py3 | 7.352.0 | bindings for NVIDIA Management Library |
   | torch | 1.9.1+cpu | neural networks |
   | torchvision | 0.10.1+cpu | torch extension for vision |
   | torchaudio | 0.9.1 | torch extension for audio |

   † This library does not work on Python 3.10, ROS packages upgrading to newer Python versions must find a substitute.

   ‡ Installed with CPU support only, [see this page for more information](https://pytorch.org/get-started/previous-versions/#linux-and-windows-17).

5. Create a temporary `/deps` folder which will be used to build additional dependencies in.

### Install Python 3.10 from source

We install Python 3.10 to take advantage of modern Python libraries which no longer support Python 3.6, we can selectively choose to use it for some of our packages through the use of [catkin virtualenv](/guides/virtualenv). This build step is derived from the [Python documentation](https://devguide.python.org/getting-started/setup-building/).

1. Install dependencies required for build.

   We choose to build all optional Python modules for maximum compatibility.

2. Download Python sources and extract.

3. Build Python sources and install globally.

   :::caution

   This will override the global Python installation, this is cleaned up in a later build step.

   :::

4. Install additional global packages.

   | Package | Version | Description |
   |:-:|:-:|---|
   | pyyaml | 6.0.1 | provides yaml package (required by rospy) |
   | rospkg | 1.5.0 | environment agnostic ROS package utilities (required by rospy) |
   | pip | 23.2.1 | Python package manager |
   | setuptools | 68.0.0 | Python build tools |
   | wheel | 0.41.1 | Python build tools |

:::note

We install from source as it's not available in PAL repositories and Python
is no longer available from the deadsnakes PPA as Ubuntu 18.04 has lost support.
When migrating this build step to noetic, you may be able to just use the
deadsnakes repository like:

```bash
apt install software-properties-common -y && apt-add-repository ppa:deadsnakes/ppa \
&& apt update && apt install python3.8 python3.8-dev python3-setuptools [.. etc]
```

:::

:::warning

Python 3.11+ is incompatible with rospy (Melodic & Noetic)

:::

### Configure global Python installation

1. Reconfigure Python symlinks

   - `/usr/bin/python` is made to point to Python 2.7
   - `/usr/local/bin/python3` is removed to restore Python 3.6 for `python3`

2. Force catkin build tool to use Python 3.10

   This step is necessary to get `catkin_virtualenv` to work properly.

   We create a folder `/path/python3.10` which contains a single symlink for `python3` which we can then prepend to the PATH when invoking catkin.

   We only want to override `catkin build` so this should be sufficient.

### Create overlay workspace for additional ROS packages

1. Create a new overlay workspace

2. Clone ROS packages we want to build from source

   | Package | Description | Source |
   |:-:|---|---|
   | catkin_virtualenv | Bundle python requirements in a catkin package via virtualenv | [locusrobotics/catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) @ `4af9970` |

3. Build the workspace

4. Add workspace setup to container environment

   We hijack `/opt/env.sh` and add our workspace at the end of the source chain.

   :::caution

   This relies on the TIAGo PAL or open source containers being used as a base to detect where the line should be added.

   :::

### Perform clean up

1. Remove the build folder `/deps`

2. Remove extraneous environment variables

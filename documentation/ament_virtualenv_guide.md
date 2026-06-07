# ament_virtualenv Guide for LASR ROS2 Packages

## The Problem

The `ros2.def` container was cloning `ament_virtualenv` pinned to version `0.5.0`:

```bash
git clone --branch 0.5.0 --depth 1 https://github.com/locusrobotics/ament_virtualenv.git
```

This old version had two bugs:

1. `glob_requirements()` did not accept a `source_dir` parameter, so it could only search
   for `requirements.txt` in already-installed packages (via `get_package_share_directory`).
   During `colcon build`, the package is not yet installed, so it was never found.

2. `find_in_workspaces()` did not filter out `None` from the workspaces list, causing a
   `TypeError` crash when `source_dir=None`.

The result: `colcon build` failed with either:
- `RuntimeError: Failed to find package.xml for package <name>`
- `[ERROR] ament_virtualenv Package <name> declares <pip_requirements> requirements.txt, which cannot be found`


## Why `--symlink-install` Does Not Work with `ament_virtualenv`

When you run `colcon build --symlink-install`, colcon tells setuptools to run in
**develop mode** (`python setup.py develop`) instead of **install mode**
(`python setup.py install`).

The `ament_virtualenv` hook is attached to the `install` command via `cmdclass`:

```python
cmdclass={"install": InstallCommand}
```

In develop mode, setuptools never calls the `install` command — it just creates
an `.egg-link` file pointing to the source directory and moves on. This means
`InstallCommand.run()` is never executed, `install_venv()` is never called,
and the virtualenv with the pip dependencies is never created.

**Bottom line:** `--symlink-install` is convenient for fast iteration on pure Python
code, but it is incompatible with `ament_virtualenv`. Always use `colcon build`
(without `--symlink-install`) for packages that have pip dependencies.


## The Fix

### 1. Updated `ros2.def` to clone `ament_virtualenv` from `main` (no version pin)

```bash
# Before (broken)
git clone --branch 0.5.0 --depth 1 https://github.com/locusrobotics/ament_virtualenv.git

# After (fixed)
git clone --depth 1 https://github.com/locusrobotics/ament_virtualenv.git
```

The `main` branch has `source_dir` support and the `None`-safe workspace filtering.

### 2. Updated all `setup.py` files to pass `source_dir`

Each package's `InstallCommand` now passes `source_dir=_here` so `ament_virtualenv`
knows where to find `requirements.txt` during the build, before the package is installed.

### 3. Added `requirements.txt` to `data_files` in `setup.py`

This ensures `requirements.txt` is copied to the installed share directory, so transitive
dependency lookups (e.g. package A depends on package B which has pip requirements) also work.


## How to Write a New Package with pip Dependencies

### `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
    <name>my_package</name>
    <version>0.0.0</version>
    <description>My package</description>
    <maintainer email="you@example.com">Your Name</maintainer>
    <license>MIT</license>

    <!-- your ROS deps -->
    <depend>sensor_msgs</depend>

    <!-- required for pip virtualenv -->
    <build_depend>ament_virtualenv</build_depend>

    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>python3-pytest</test_depend>

    <export>
        <build_type>ament_python</build_type>
        <!-- tells ament_virtualenv where your pip deps are -->
        <pip_requirements>requirements.txt</pip_requirements>
    </export>
</package>
```

### `setup.py`

```python
from setuptools import find_packages, setup
import os
from glob import glob
import setuptools.command.install
import ament_virtualenv.install

package_name = "my_package"

# Absolute path to this setup.py — needed so ament_virtualenv can find requirements.txt
# during colcon build, before the package is installed.
_here = os.path.dirname(os.path.abspath(__file__))


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version="3",
            source_dir=_here,       # <-- required: tells ament_virtualenv where to look
        )
        return


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Include requirements.txt in the installed share dir so transitive
        # dependency lookups work (e.g. when another package depends on this one)
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="My package description",
    license="MIT",
    extras_require={"test": ["pytest"]},
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "my_node = my_package.node:main",
        ],
    },
)
```

### `requirements.txt`

List your pip dependencies, one per line. Pin versions where possible to avoid
incompatibilities (especially numpy — always pin `numpy<2` to avoid cv_bridge crashes):

```
torch
torchvision
numpy<2
opencv-python
transformers
```

### Building

Always build **without** `--symlink-install` for packages that use `ament_virtualenv`:

```bash
colcon build --packages-select my_package
```

With `--symlink-install`, setuptools uses `develop` mode which skips the `install`
cmdclass entirely — the virtualenv is never created and pip deps are not installed.


## Summary Checklist for New Packages

- [ ] `package.xml` has `<build_depend>ament_virtualenv</build_depend>`
- [ ] `package.xml` has `<pip_requirements>requirements.txt</pip_requirements>` in `<export>`
- [ ] `setup.py` defines `_here = os.path.dirname(os.path.abspath(__file__))`
- [ ] `setup.py` `InstallCommand` passes `source_dir=_here` to `install_venv`
- [ ] `setup.py` `data_files` includes `"requirements.txt"` alongside `"package.xml"`
- [ ] `python_version="3"` (not `"2"`)
- [ ] `requirements.txt` exists and has `numpy<2` if using cv_bridge
- [ ] Build with `colcon build` (no `--symlink-install`)

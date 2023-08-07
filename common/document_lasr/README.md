# document_lasr

Package for documenting ROS packages in your current workspace.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)

If you would like to view the documentation in the browser, ensure you have at least [Node.js 18 LTS](https://nodejs.org/en).

```bash
# install Node.js 18 LTS on Ubuntu:
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash - &&\
sudo apt-get install -y nodejs
```

## Usage

View workspace documentation in the browser:

```bash
rosrun document_lasr view.py
```

Your `package.xml` should define:

- A description
- Maintainers and authors
- A full list of ROS dependencies

You should write or create blank files for:

- `doc/PREREQUISITES.md`: Additional requirements for using the package.
- `doc/USAGE.md`: Instructions for using the package.
- `doc/EXAMPLE.md`: Demonstration of how the package works.
- `doc/TECHNICAL.md`: An overview of how the package works.

Document a single package from your current workspace:

```bash
rosrun document_lasr generate_readme.py ros_package_name
```

Document all LASR packages in current workspace:

```bash
rosrun document_lasr generate_all.py
```

## Example

Delete the `README.md` for this package and then run:

```bash
rosrun document_lasr generate_readme.py document_lasr
```

## Technical Overview

This package scans your workspace to find packages and uses:

- `package.xml`
- `doc` files
- `srv` / `msg` / `action` definitions

to generate a README file for the package.

## ROS Definitions

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.

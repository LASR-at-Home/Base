# document_lasr

Package for documenting ROS packages in your current workspace.

This package is maintained by:
- [Paul Makles](me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)



## Usage

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

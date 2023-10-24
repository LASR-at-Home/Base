# documentation

Package for documenting ROS packages in your current workspace.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)

If you would like to view the documentation in the browser, ensure you have at [Node.js v16.x](https://nodejs.org/en) installed.

## Usage

View workspace documentation in the browser:

```bash
rosrun documentation view.py
```

Please see the [example section](#example) to see how to document your package.

Document a single package from your current workspace:

```bash
rosrun documentation generate_readme.py ros_package_name
```

Document all LASR packages in current workspace:

```bash
rosrun documentation generate_all_lasr.py
```

## Example

For this example, let's suppose you have a package `my_package`.

1. Ensure your `package.xml` file is up to date.

   You should add a proper description if one is not present.

   You should update the list of maintainers and authors if they are not correct.

   You should make sure all of your ROS package dependencies are listed, this includes packages installed from apt repositories and the workspace.

   ```xml
   <package>
     <name>my_package</name>
     <description>Package for interacting with X and Y.</description>

     <!-- Maintainers -->
     <maintainer email="your@current.email">Your Name</maintainer>
     <maintainer email="their@current.email">Other Maintainer</maintainer>

     <!-- Authors -->
     <author email="their@current.email">Other Author</author>
   </package>
   ```

2. Create the `doc/PREREQUISITES.md` file.

   If you have any additional dependencies that need to be installed into the container, please list them here.

   Otherwise, create a blank file.

   ```md
   This package requires Go 1.21.0 to be present and a valid GOPATH configured for the package to build correctly.
   ```

3. Create the `doc/USAGE.md` file.

   Provide instructions for using the package, this should include a brief description of how it can be integrated and consumed from other packages if applicable.

   ```md
   Start the service by running:

   \`\`\`bash
   rosrun my_package my_service
   \`\`\`

   You can now interact with the service as such:

   \`\`\`python
   from my_package import MyPackage

   client = MyPackage()
   client.do_something()
   \`\`\`
   ```

4. Create the `doc/EXAMPLE.md` file.

   Provide a start-to-finish example of how the package can be used.

   This will vary between packages, but in general it should explain what services need to be started, how testing data can be provided / generated / published, how tests may be run / validated, and how to launch any visualisations.

   ```md
   1. Start the service

      \`\`\`bash
      roslaunch my_package my_service.launch
      \`\`\`

   2. Start the example script

      \`\`\`bash
      rosrun my_package example.py
      \`\`\`
   ```

5. Create the `doc/TECHNICAL.md` file.

   This is up to you to figure out, but in this file you should provide a technical overview of how the package works. Include whatever you think is required for someone who would want to contribute to the package.

6. Describe fields in messages / services / actions.

   You should go through all of your `.msg`, `.srv`, and `.action` files in their respective folders, and add comments to each field.

   ```
   # the table we are going to check
   uint8 table
   ---
   # whether the table is ready
   bool result # (you can also put comments here)
   ```

7. Add descriptions, examples and documentation to launch files.

   Within your launch files, you can add `<description>` and `<usage>` tags to better explain what the launch file is there for:

   ```xml
   <launch>
     <description>Generate documentation for a single package</description>

     <usage doc="Document this package" />
     <usage doc="Document coffee_shop package">package:=coffee_shop</usage>
   </launch>
   ```

   As well as this, make sure your `<arg>` tags have documentation strings.

   ```xml
   <launch>
     <arg name="package" default="documentation" doc="Package to document" />
     <arg name="set_me_to_anything" doc="This is an example argument without a default" />
     <arg name="hidden_arg" value="this won't show up" />
   </launch>
   ```

Finally, regenerate the README file:

```bash
rosrun documentation generate_readme.py my_package
```

## Technical Overview

This package scans your workspace to find packages and uses:

- `package.xml`
- `CMakeLists.txt`
- `doc` files
- `srv` / `msg` / `action` / `launch` definitions
- `requirements.txt` / `requirements.in`

to generate a README file for the package.

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.

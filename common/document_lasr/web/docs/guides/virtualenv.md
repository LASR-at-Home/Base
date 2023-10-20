# Configuring Python dependencies for packages

There are two ways of managing Python dependencies for your packages, it is recommended that you use a virtual environment with a pinned Python version and pinned requirements, however if necessary this document also details how to add a dependency globally to the container.

## (A) Adding dependencies to container

:::warning

Strongly advised against unless if you cannot migrate your code to a virtual environment right now.

:::

Find the line in the definition file where Python dependencies are declared:

```diff
# Additional Python packages
+ # pyyaml: yaml library
- python3.6 -m pip install rosnumpy==0.0.5.2 scipy==1.5.4
+ python3.6 -m pip install rosnumpy==0.0.5.2 scipy==1.5.4 pyyaml==6.0.1
```

Then update the container maintenance document accordingly, you can do this by going to [Container Maintenance](/guides/container) and then clicking edit page at the bottom.

## (B) Setting up a virtual environment using catkin_virtualenv

:::caution

Using catkin_virtualenv requires that catkin itself uses Python 3.7 or later during the build process, this is already managed for you by the container however may prove to be an issue if building a new container.

:::

:::note

The following Python versions are available in the container:

- Python 2.7
- Python 3.6
- Python 3.10 (recommended)

:::

Begin by configuring your package to use catkin_virtualenv, add the following entries to `package.xml`:

```xml
<!-- package.xml -->
<package>
  <!-- metadata -->

  <!-- .. your other dependencies -->
  <build_depend>catkin_virtualenv</build_depend>

  <export>
    <!-- .. your other exports -->
    <pip_requirements>requirements.txt</pip_requirements>
  </export>
</package>
```

> We add a dependency on the tool so the package won't ever build without it, we also add a new export to inform the wrapper script where to find the requirements file at runtime and to indicate that other packages with virtual environments should inherit dependencies from this package.

:::info

Given two packages with virtual environments, you can add a dependency to the `package.xml` to inherit the other's dependencies.

```xml
<!-- package.xml -->
<package>
  <!-- metadata -->

  <!-- .. your other dependencies -->
  <depend>my_other_pkg_with_venv</depend>
</package>
```

:::

In your `CMakeLists.txt`, add the package as a dependency:

```cmake
find_package(catkin REQUIRED catkin_virtualenv .. your other dependencies)
```

Then ensure Python setup is enabled and configure settings for the virtual environment:

```cmake
catkin_python_setup()
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3.10
)
```

> Reasonable defaults are provided above, it is advised you always specify the interpreter and use locked dependencies (as opposed to just providing a requirements.txt file).
>
> You can read more about [available options here](https://github.com/locusrobotics/catkin_virtualenv/blob/master/README.md#additional-cmake-options).

:::warning

Ensure you have a `setup.py` file configured, template provided below:

```python
#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  packages=['my_package'],
  package_dir={'': 'src'}
)

setup(**setup_args)
```

:::

If you have any Python scripts, you must install them to be wrapped with the virtual environment:

```cmake
catkin_install_python(PROGRAMS
  nodes/my_service
  scripts/do_something.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Make sure to also install the requirements file if you want other packages to inherit dependencies:

```cmake
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

:::note

This step is optional but recommended, you might as well add it.

:::

Finally, create a `requirements.in` file (or `requirements.txt` if you did not set `INPUT_REQUIREMENTS`):

```requirements
# requirements.in
ultralytics==8.0.150
numpy>1
PyYaml
```

You can generate the `requirements.txt` file by running `catkin build my_package` (or just a full build).

:::info

If you want to update packages or install new packages, delete the `requirements.txt` file and run build again.

:::

:::tip

This also integrates with the `document_lasr` package, you can regenerate your README to include / update your Python dependencies. 

:::

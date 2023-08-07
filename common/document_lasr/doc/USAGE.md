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

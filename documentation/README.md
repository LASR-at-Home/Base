# LASR BASE development guide

## Creating a new package
1. `cd` to the relevent directory (this will be one of the folders in common)
2. Run `ros2 pkg create --build-type ament_python <package_name>`
    - Use `--build-type ament_cmake`instead for C++ packages or interface packages
3. Add virtual environment support for your package (refer to: [venv guide](ament_virtualenv_guide.md)) 


## Supporting Documentation
- [YASMIN](YASMIN_PORT.md)
- [Virtual Environment](ament_virtualenv_guide.md)

## Key Do's and Don't

### Do's
 - For state machine (YASMIN) 
    - set sigint_handler true only for top level SM. 
    - Always use dedicated states for service clients, subscriber states and so on. This is becuase spins and threading is handled under the hood hence is more reliable.

### Dont's
 - Prevent using rclpy.spin() as can cause issues with callbacks
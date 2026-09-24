# LASR BASE development guide

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
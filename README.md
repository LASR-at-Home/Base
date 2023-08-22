# Base

The main repository for the LASR codebase.

                            +------------------+                      
                            |                  |                      
                            |                  |                      
                            |    Main Tasks    |                      
                            |                  |                      
                            +--------|---------+                      
                                     |                                
                                     |                                
                                     |                                
                            +------------------+                      
                            |                  |                      
                            |    Sub Tasks     |                      
                            |                  |                      
                            |                  |                      
                            +--------|---------+                      
                                     |                                
                                     |                                
                                     |                                
        +--------------------------------------------------------------+
        | Modules   -----------------------------------------          |
        |    +------|------+     +-------------+     +------|------+   |
        |    |             |     |             |     |             |   |
        |    | Interaction ------| Perception  |------  Navigation |   |
        |    |             |     |             |     |             |   |
        |    +-------------+     +-------------+     +-------------+   |
        +--------------------------------------------------------------+

## Documentation

You can view documentation for this repository by running:

```bash
rosrun document_lasr view.py
```

This requires Node.js which is already provided in the RoboCup container.

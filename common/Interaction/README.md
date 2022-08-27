# The Interaction Module

Core module for user interaction via speech and text. Provides a web interface for diagnostics, logs and interaction.

This module consists of the following packages:

- lasr_dialogflow
- lasr_interaction_server
- lasr_web_server

## Running the stack

To run the Interaction stack, simply run the following:

```roslaunch lasr_interaction_server interaction_server.launch```

This will launch the interaction server, which handles user input and provides responses, dialogflow and the web interface.
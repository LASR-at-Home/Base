## Instructions

Download models.tar from the link below, run the command `tar -xvf models.tar` and move the `models` folder into `custom_worlds` root directory 

Link: [Models](https://leeds365-my.sharepoint.com/personal/sc19jcm_leeds_ac_uk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fsc19jcm%5Fleeds%5Fac%5Fuk%2FDocuments%2Frobocup2022)

To setup the mappings for the Robocup competition world of 2021:

    Move the directory wrs2020, which contains the mappings, to ~/.pal/tiago_maps/configurations/

To launch wrs2020 world:

    roslaunch custom_worlds wrs2020.launch

To setup the mappings for the Robocup competition world of 2022 receptionist task:

    Move the directory wrs_receptionist, which contains the mappings, to ~/.pal/tiago_maps/configurations/    

To launch wrs_receptionist world:

    roslaunch custom_worlds wrs_receptionist.launch

To launch custom_worlds in mapping mode:

    roslaunch custom_worlds tiago_mapping.launch world:="<world_name>"


to launch custom_worlds in navigation mode:

    roslaunch custom_worlds tiago_navigation.launch world:="<world_name>"

Please note that the `.world` suffix should be ommitted.

About wrs2020 world:

    There is an exact copy of the wrs2020 world without a box on top of the table. You can find it in **/worlds/wrs2020_nobox.world**
    The script **/scripts/spawn_objects** will load the world with objects on the table and on the floor. To execute it:
        
        rosrun custom_worlds spawn_objects


## Directories

- Worlds can be found in and added to the **/worlds** directory.
- 3D models can be found in and added to the **/models** directory.
- Quick-launch files for navigation can be found in the **/launch** directory.

# Overview
The General Service Purpose Robot (GPSR) task is a *Stage I* task in which the robot must execute three randomly generated commands requested by an operator. The grammar and rules used to generate the commands are defined in the [Command Generator Repository](https://github.com/kyordhel/GPSRCmdGen), and so the possible set of tasks are known in advance. 

# Rules
This section replicates the task information and rules that are provided in the [RoboCup rulebook](https://robocupathome.github.io/RuleBook/rulebook/master.pdf).
## Task Setup

### Locations
**Task location:** The task takes place inside the arena, but some commands may require the robot to go outside of the arena. The arena is in its *nominal* state for this task.

**Start location:** The robot starts outside of the arena. When the door opens, the robot moves towards the *instruction point*.

**Instruction point:** At the beginning of the test, as well as after finishing the first and second command, the robot moves to the *instruction point*.

### People
**Operators:** A *professional operator* (the referee) commands the robot to execute the task. Optionally, for bonus points, commands can be issued by a *non-expert operator* i.e., a person from the audience with no robotics background. In this case, the referee gives the goal of the command to the non-expert operator, who will then issue it to the robot **in their own words**. For example, the generated command might be *"Bring me a coke from the kitchen"*. Then, the non-expert operator will be told *"The robot should bring you a coke, which is found in the kitchen"*, who then tells the robot, *"I want a coke. Go to the kitchen and get me one"*. If the robot consistently fails to understand the non-expert operator (e.g., after two retries), teams can default to a custom operator. 

## Procedure
**1. Instruction point** - two hours before the test, the referees announce the location of the *instruction point*.

**2. Test start** - the robot moves to the *instruction point* when the arena door is opened.

**3. Command execution** - the operator instructs the robot to execute a command and the robot performs the task.

**4. Back to the instruction point** - the robot goes back to the *instruction point* and waits for the next command.

## Additional Rules and Remarks
**1. Partial scoring** - the main task allows partial scoring (per **completed** command).

**2. Command generator** - tasks will be generated using the official [GPSR Command Generator](https://github.com/kyordhel/GPSRCmdGen).

**3. Non-expert operators** - Referees are not allowed to instruct non-expert operators on how to operate the robot. Teams attempting to instruct or bias the operator will be disqualified from the task.

**4. Deus ex Machina** - the scores are reduced if human assistance is received, in particular for:
- using a custom operator
- bypassing speech recognition by using an alternative HRI
- receiving human assistance to accomplish a task
- instructing a human assistant to perform the whole task

## Score Sheet
The maximum time for this test is 5 minutes.

| Action                                                                                                                                  | Score  |
| --------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| **Main Goal** (executing the task associated with each command)                                                                         | 3x400  |
| **Bonus Rewards** (understanding a command given by a non-exper operator)                                                               | 3x100  |
| **Deus Ex Machina Penalties**                                                                                                           |        |
| Using a custom operator                                                                                                                 | 3x-50  |
| Bypassing speech recognition                                                                                                            | 3x-50  |
| Instructing a human to perform parts of the task will apply a percentage penalty according to similar penalties in other Stage I tests. | 3x-400 |
| **Special Penalties & Bonuses**                                                                                                         |        |
| Not attending                                                                                                                           | -500   |
| Using alternative start signal                                                                                                          | -100   |
| **Total Score**                                                                                                                         | 1500       |
# Command Generation
## Installation 
The guide to install the command generator can be found in a separate document titled: [[Installing GPSR Command Generator]].

## Usage
Once installed, in the top level of the command generator repository, you can run the command `make gpsr` to run an interactive terminal program that allows you to generate a single command at a time.

Alternatively, to generate multiple commands in one go (where results are saved in a sub directory named after the grammar used for command generation), you can run the command `mono bin/Release/GPSRCmdGen.exe --bulk 1000`, which will generate 1000 GPSR commands.

## Grammar 
The GPSR commands are randomly generated based on a set of pre-defined grammar production rules. These rules dictate the set of possible tasks the robot can be asked to complete, along with additional meta information.

The format of the rules is as follows: A rule is made up of English text, (optional) variables, and (optional) wildcards. Every `$` defines a variable that is either defined in terms of other variables, wild cards, or English text. Curly braces `{}` define wildcards which are defined below.

If you recursively trace the rule definition, you will eventually be able to define the rule in terms of (a set of) English text. One of these sets will then be randomly chosen if the given rule is used to produce a command. 
### Wildcards
Wildcards are strings within curly braces that are replaced by random values during random sentence generation. The syntax is as follows:

```
{wildcard [type] [id] [where condition] [meta:[metadata]]}
```

The wildcard name is mandatory: all other parameters are optional. The optional parameters are defined below:

The "type" parameter defines a sub-category of the wildcard which is used to constrain the generation of a wildcard to a random element in that certain sub-category.

The "id" is essentially a random seed which means that any other wildcards of the same type will have the same random value used across matching ids. 

The "where" condition is used to filter the possible set strings that can be randomly generated. 

The "meta" tag is a literal string defined in the wildcard which is used to display additional information separate from the command itself (often used for the referee to determine how to set up the task).

#### Wildcard Types

The following types of wildcards are defined:

| Wildcard Name | Description                    | Allowed Types                                                               |
| ------------- | ------------------------------ | --------------------------------------------------------------------------- |
| {category}    | An object category             | Types are ignored                                                           |
| {gesture}     | A gesture name                 | Types are ignored                                                           |
| {location}    | Any location                   | room or placement or beacon                                                 |
| {name}        | A person name                  | male or female                                                              |
| {object}      | An object name                 | known or alike                                                              |
| {question}    | The "question" literal string. | Types are ignored. Metadata contains a Q/A from the predefined question set |
| {void}        | Used to inject metadata only   | Types are ignored.                                                                            |

The categories are strings belonging to these wildcards are defined in a set of `.xml` files. We currently do not have these data files, but they should be provided to us a few months ahead of the competition. The `.xml` files for the 2023 competition can be accessed [here](https://github.com/RoboCupAtHome/Bordeaux2023/tree/master/cmdgen). 



#### Wildcard Aliases
The following wildcard aliases are defined:

| Wildcard Name | Alias                          |
| ------------- | ------------------------------ |
| {beacon}      | Alias for {location beacon}    |
| {aobject}     | Alias for {object alike}       |
| {female}      | Alias for {name female}        |
| {kobject}     | Alias for {object known}       |
| {male}        | Alias for {name male}          |
| {placement}   | Alias for {location placement} |
| {room}        | Alias for {location room}                               |
#### Obfuscating Information
Some wildcard names can be ended with a question mark which obfuscates the value instead of the random value chosen for that wildcard, which is instead shown as metadata. The following obfuscation wildcards are defined:

| Wildcard name | Obfuscation                                                                                                         |
| ------------- | ------------------------------------------------------------------------------------------------------------------- |
| {category?}   | The literal string "objects"                                                                                        |
| {location?}   | The literal string "room" for rooms. For placement and beacons, the name of the room to which the location belongs. |
| {object?}     | The name of the category to which the object belongs.

#### Wildcard Pronouns 
Wildcards can be referenced with the `{pron}` (standing for pronoun) construct. By default, this construct refers to the last wildcard found in the rule in the following order:
1. {name} wildcards.
2. {object} wildcards.
3. Last wildcard found.
   
By default, the `{pron}` construct is replaced with a Personal Pronoun (him, her, or it). Subjective cases of the personal pronoun (he, she, it) are also supported by setting the type of the `{pron}` construct to the 'sub' value. Namely:

- {pron} -- reference with pronoun, objective case
- {pron obj} -- reference with pronoun, objective case (explicit)
- {pron sub} -- reference with pronoun, subjective case
  
Likewise, possessive pronouns can be specified in both cases, absolute (mine, yours, theirs), and adjective (my, your, their) which is the default. These wildcards are defined as follows:

- {pron pos} -- Reference with possessive pronoun, adjective case.
- {pron pab} -- Reference with possessive pronoun, absolute case.
- {pron paj} -- Reference with possessive pronoun, adjective case.
- {pron posabs} -- Reference with possessive pronoun, absolute case.
- {pron posadj} -- Reference with possessive pronoun, adjective case.
### Example
As an example, the first two production rules for manipulation are:

```
$man = $deliver
$deliver = $vbbtake the {aobject} from the {room} to the {placement 2}
```


The `$vbbtake` specifies the verb bring/take, whose definition can be found in the `CommonRules.txt` file in the command repository: `$vbbtake = bring | take`. On command generation, any rule with a `$vbbtake` is then randomly substituted with the English word *"bring"* or *"take"*.

The wildcards are substituted in with a random value according to the objects and locations specified in the contest-specific `.xml` files.

The final English instruction would be something akin to *"bring the apple from the kitchen to the living room side tables."* (using the 2023 `.xml` files).

# Tasks
We are now in a position to decompose and group the set of possible commands according to the task/skill that the robot needs to be able to undertake. We follow the structure used in the grammar definition that groups the commands into a set of tasks categories. 

## Manipulation 
All manipulation tasks involve moving an object from one location to another. They can be broadly grouped into the following commands:

1. Take the object from a room to another location.
2. Fetch an object from a location.
3. Deliver an object to a named person at a beacon.
4. Take an object from a placement location and place on a different placement location.
5. Deliver luggage to the taxi

## Complex Manipulation
The complex manipulation commands are as follows:

1. Take a known object to a placement location. *This includes metadata that the access to the location should be blocked with furniture or a movable object*
2.  Bring a (absolute position) object from a placement location. *Absolute position is one of left most or right most and there will be at least give objects at the location*
3. Bring me the object (relative position) the other object at the placement location. *Relative position is one of at the right-of, left-of, or on-top-of*
4.  Bring the *property* (object or object category) from the placement location. *Where property is one of: biggest, largest, smallest, heaviest, lightest, or thinnest.*
5. Clean a named room. *Meta says to place 3 objects randomly, at least one on the floor*
6. Takeout the garbage. *It is not clear where to take the garbage to or where to find the garbage*. 

## Find Objects
Broadly speaking there are four high-level object finding tasks:

1. Find a (one or three) category of object in a room.
2. Tell me how many of an object (or object category) are on a placement location.
3. Tell me the *property* object on a placement (optionally of a category of object).
4. Tell me the three *property* objects in a given category in a given placement location.

Where *property* is one of: biggest, largest, smallest, heaviest, lightest, or thinnest. 
## Find People
There are broadly three people finding tasks:

1. Tell me the (name, or gender, or pose) of the person at a room or beacon.
2. Tell me how many people in a room are in a certain pose or are (men/women/boys/girls).
3. Go to a room, find a person, and talk.

Talk in the above context means either repeating a given phrase, or answering a question using the given Q/A document.

Pose is one of: sitting, standing, or lying down.
## Follow People
There are two types of following tasks: one in the arena (i.e., mapped navigation) and another outside of the arena (i.e., un-mapped navigation).

The mapped navigation commands take the form of:
1. Follow a named person from a beacon to a room.
2. Meet a named person at the beacon and follow them.

The un-mapped navigation command takes the form of:
1. Meet a named person at a beacon, follow them, and then go to a room.

For the un-mapped, it seems like we need to be able to store the route travelled outside of the arena, such that we can navigate back to the arena.
## Guide People
As with the following tasks, the guiding tasks can be divided into mapped and un-mapped navigation.

For the mapped navigation commands, it should be noted that every command has the following metadata at the end of it:

*"The person being guided must deviate when indicated by the referee before reaching the beacon"*.

We thus need a way of detecting when the person is no longer following the robot, and handle this accordingly.

All mapped navigation commands share the following form:

1. Meet a named person at a beacon, and guide them to another beacon.

The un-mapped navigation command is:

1. Meet a named person at a beacon, follow them outside of the arena, and then **guide them back**.

We therefore need to re-use the route-storing skill that is needed for the person following un-mapped navigation.
## Incomplete Commands
It is not explicitly stated, but these commands are all lacking some information, so I assume that we need to detect this and ask the instruction giver for the information that we are missing.

The incomplete commands are:

1. "Follow" -- *need to ask for the name of the person and their location*
2.  Bring instruction giver a category of object -- *need to ask for the specific object name*
3. Deliver a category of object to a named person -- *need to ask for the specific object name*
4. Meet a named person and guide them and the person will get lost along the way -- *need to ask for the location of the person and the location of where to guide them to*

## Party Host
Note that in the following commands *people* refers to *everyone* or all of the *people, men, women, guests, elders, or children*. It is not explicitly stated that we do not have to detect for gender/age -- perhaps the safest bet would be to simply have a function to ask that we can deploy if absolutely necessary.

The party host commands are as follows:

1. Serve drinks or snacks to people in a given room.
2. Meet a named person at a named door (where named means front or back etc.) and introduce them to people in a given room.
3. Meet a named person at a beacon and ask them to leave.
4. Meet a named person and a beacon and introduce them to another named person at a different beacon.
5. Meet a named person at a beacon and guide them to their taxi.


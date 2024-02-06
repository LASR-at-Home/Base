# lasr_vector_database_weaviate

Integration of the Weaviate vector database into LASR stack.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)
- lasr_vector_database_msgs

This packages requires Python 3.10 to be present.

This package has 18 Python dependencies:
- [weaviate-client](https://pypi.org/project/weaviate-client)==v4.4b2
- [requests](https://pypi.org/project/requests)==2.31.0
- .. and 16 sub dependencies

Currently this package will only work on Linux amd64.

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Create a new collection called "Hello":

```bash
rosservice call /database/vectors/weaviate/create_collection "name: 'Hello'
skip_if_exists: false
clear_if_exists: false"
```

Insert some vectors:

```bash
rosservice call /database/vectors/weaviate/insert "name: 'Hello'
properties:
- key: 'thing'
  value: 'true'
vector:
- 0 
- 1 
- 0"

rosservice call /database/vectors/weaviate/insert "name: 'Hello'
properties:
- key: 'thing'
  value: 'false'
vector:
- 1 
- 0 
- 1"
```

Now you can query the database:

```bash
rosservice call /database/vectors/weaviate/query "name: 'Hello'
limit: 3
vector:
- 1 
- 1 
- 1"
```

You should receive results like:

```yaml
results: 
  - 
    certainty: 0.9082483053207397
    properties: 
      - 
        key: "thing"
        value: "false"
  - 
    certainty: 0.7886751294136047
    properties: 
      - 
        key: "thing"
        value: "true"
```

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `server`

Start the Weviate vector database

```bash
# Default directory and ports
roslaunch lasr_vector_database_weaviate server.launch 

# Customise the ports used
roslaunch lasr_vector_database_weaviate server.launch http_port:=40050 grpc_port:=40051

# Create a separate database instance
roslaunch lasr_vector_database_weaviate server.launch db_name:=your_database_name
```

| Argument | Default | Description |
|:-:|:-:|---|
| version | 1.22.11 | Version of Weaviate to download and run |
| http_port | 50050 | Port to bind HTTP service to |
| grpc_port | 50051 | Port to bind GRPC service to |
| db_name | weaviate | Custom database name |



### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.

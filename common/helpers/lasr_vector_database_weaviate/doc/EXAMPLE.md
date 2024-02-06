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

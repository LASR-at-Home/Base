## Find Objects
Broadly speaking there are four high-level object finding tasks:

1. Find a (one or three) category of object in a room.
2. Tell me how many of an object (or object category) are on a placement location.
3. Tell me the *property* object on a placement (optionally of a category of object).
4. Tell me the three *property* objects in a given category in a given placement location.

Where *property* is one of: biggest, largest, smallest, heaviest, lightest, or thinnest. 


solutions:
1. for the heavest, set the default weight for each label like
average_weights = {
    "car": 1500,
    "bicycle": 15,
    "person": 70
}

2. for the biggest using the bounding_box = detection.bounding_box to calculate the area of each object
This package provides the `/lasr_rasa/parse` service which uses the `Rasa` service definition from `lasr_rasa`.

```python
from lasr_rasa.srv import Rasa, RasaRequest

# create service proxy
rasa_service = rospy.ServiceProxy('/lasr_rasa/parse', Rasa)

# create request
request = RasaRequest("hello")

# send request
response = rasa_service(request)
# .. use request.json_response, if request.success
```

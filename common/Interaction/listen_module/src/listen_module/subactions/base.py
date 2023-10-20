#!/usr/bin/env python3
from google.cloud import dialogflow_v2 as dialogflow
from listen_module.dialogflow_client import DialogClient
from proto.marshal.collections.maps import MapComposite
import uuid


class BaseActions:
    def __init__(self, action_id):
        self.action_id = action_id
        self.client = DialogClient(action_id)
        self.session_id = uuid.uuid4()

    def get_context(self, context, lifespan=1):
        return dialogflow.types.Context(
            name=dialogflow.ContextsClient.context_path(self.action_id, self.session_id, context),
            lifespan_count=lifespan)

    def dialog_in_context(self, audio_or_text: str = "SOUND:PLAYING:PLEASE", context: str = None):
        query_params = None
        if context:
            query_params = dialogflow.types.QueryParameters(contexts=[self.get_context(context)])
        return self.client.detect_intent(audio_or_text=audio_or_text, query_params=query_params)


    def get_parameters(self, response):
        response = response.query_result.parameters
        resp = []
        # for response in responses:
        if response:
            res = {}
            query_result = response.query_result
            query_text = query_result.query_text
            parameters = query_result.parameters
            for parameter_name, parameter_value in parameters.items():
                if isinstance(parameter_value, MapComposite):
                    parameter_values = dict(parameter_value)
                    resp.append((parameter_name, parameter_values))
                else:
                    resp.append((parameter_name, parameter_value))

            res["query_text"] = query_text
            res["parameters"] = parameters
            res["query_result"] = query_result
            res["response"] = resp
        print(res, "res in audio")
        return res

    def stop(self):
        self.client.stop()

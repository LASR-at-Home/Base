#!/usr/bin/env python3

from lasr_dialogflow.dialogflow_client_stream import DialogflowClientStream
import uuid
import dialogflow

class BaseAction():

    def __init__(self, project_id, df_lang_id="en"):

        self.project_id = project_id
        self.df_lang_id = df_lang_id
        self.attempts = 0
        self.max_attempts = 3
        self.streaming_client = None

        self.session_id = uuid.uuid4()

        self.streaming_client = DialogflowClientStream(
            self.project_id,
            self.session_id,
            language_code=self.df_lang_id
        )
    
    def stop():
        self.streaming_client.stop()
    
    def listen_in_context(self, context=None):

        query_params = None
        if context:
            query_params = dialogflow.types.session_pb2.QueryParameters(contexts=[self.get_context(context)])

        response = None
        for response in self.streaming_client.response_generator(query_params):
            if response.recognition_result.message_type == dialogflow.enums.StreamingRecognitionResult.MessageType.END_OF_SINGLE_UTTERANCE:
                rospy.loginfo("received end of utterance")
                self.stop()

        if response and not response.query_result.query_text:
            rospy.loginfo("no audio request")
            self.attempts +=1
            if self.attempts >= self.max_attempts:
                return None
            else:
                # speak here
                return self.listen_in_context(context)
        else:
            self.attempts = 0
            return reponse

    def text_in_context(self, text, context=None):
        query_params = None
        if context:
            query_params = dialogflow.types.session_pb2.QueryParameters(contexts=[self.get_context(context)])
        
        return self.streaming_client.text_request(text, query_params=query_params)

    def get_context(self, context, lifespan=1):
        return dialogflow.types.context_pb2.Context(name=dialogflow.ContextsClient.context_path(self.project_id, self.session_id, context),
                              lifespan_count=lifespan)
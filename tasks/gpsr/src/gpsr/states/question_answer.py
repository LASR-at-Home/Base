import smach

from lasr_skills import AskAndListen, JsonQuestionAnswer, Say


class QuestionAnswer(smach.StateMachine):

    def __init__(self, index_path: str, txt_path: str, json_path: str, k: int = 1):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["query_sentence"],
            output_keys=["closest_answers"],
        )

        with self:

            smach.StateMachine.add(
                "GET_QUESTION",
                AskAndListen(
                    tts_phrase="I hear you have a question for me. What is it?"
                ),
                transitions={
                    "succeeded": "XML_QUESTION_ANSWER",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "XML_QUESTION_ANSWER",
                JsonQuestionAnswer(index_path, txt_path, json_path, k),
                transitions={
                    "succeeded": "SAY_ANSWER",
                    "failed": "failed",
                },
                remapping={
                    "query_sentence": "transcribed_speech",
                },
            )

            smach.StateMachine.add(
                "SAY_ANSWER",
                Say(format_str="The answer to your question is: {}"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "closest_answers"},
            )

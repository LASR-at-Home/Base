import os
import subprocess
import tempfile

import smach
from geometry_msgs.msg import Point, Pose, Quaternion
from gtts import gTTS

from autonomous_behaviour.states.query_llm import load_locations
from lasr_skills import GoToLocation


class DispatchSkill(smach.State):
    """SMACH state that executes a skill chosen by the LLM.

    Reads `skill` and `skill_args` from userdata and calls the matching handler
    (say, go_to_location). Falls back to a spoken error for unknown skills.
    """

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["skill", "skill_args"],
        )
        self.node = node
        self.locations = load_locations()

    def _say(self, text):
        """Synthesize `text` with gTTS and play it through the default audio speaker.
        Used in simulation now. TODO: use the say skill of tiago with a flag
        if you're using the robot.          
        """
        self.node.get_logger().info(f"Saying: {text}")
        try:
            tts = gTTS(text=text, lang="en")
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                mp3_path = f.name
            wav_path = mp3_path.replace(".mp3", ".wav")
            tts.save(mp3_path)
            subprocess.run(
                ["ffmpeg", "-y", "-i", mp3_path, wav_path, "-loglevel", "quiet"],
                check=False,
            )
            subprocess.run(["aplay", "-D", "pulse", wav_path], check=False)
            os.unlink(mp3_path)
            os.unlink(wav_path)
        except Exception as e:
            self.node.get_logger().error(f"TTS error: {e}")

    def _go_to_location(self, location_name):
        """Navigate to a named location from `locations.yaml` via the GoToLocation skill.

        Returns "succeeded" on arrival, "failed" if the location is unknown
        or navigation fails.
        """
        if location_name not in self.locations:
            self.node.get_logger().error(f"Unknown location: {location_name}")
            self._say(f"I don't know where {location_name} is")
            return "failed"
        loc = self.locations[location_name]
        pose = Pose(
            position=Point(
                x=float(loc["position"]["x"]),
                y=float(loc["position"]["y"]),
                z=float(loc["position"].get("z", 0.0)),
            ),
            orientation=Quaternion(
                x=float(loc["orientation"]["x"]),
                y=float(loc["orientation"]["y"]),
                z=float(loc["orientation"]["z"]),
                w=float(loc["orientation"]["w"]),
            ),
        )
        self.node.get_logger().info(f"Navigating to '{location_name}'")
        state = GoToLocation(node=self.node, location=pose)
        return state.execute(userdata={})

    def execute(self, userdata):
        """Route the skill name to its handler. Unknown skills fail with a spoken error."""
        skill = userdata.skill
        args = userdata.skill_args

        if skill == "say":
            self._say(args.get("text", ""))
            return "succeeded"
        elif skill == "go_to_location":
            return self._go_to_location(args.get("location", ""))
        else:
            self.node.get_logger().warn(f"Unknown skill: {skill}")
            self._say(f"I don't know how to {skill}")
            return "failed"

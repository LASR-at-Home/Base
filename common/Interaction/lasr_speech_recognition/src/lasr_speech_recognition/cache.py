import whisper
import rospy

# Keep all loaded models in memory
MODEL_CACHE = {}

def load_model(name: str):
    '''
    Load a given Whisper model
    '''
    global MODEL_CACHE

    if name not in MODEL_CACHE:
        rospy.loginfo(f'Load model {name}')
        MODEL_CACHE[name] = whisper.load_model(name)
    
    return MODEL_CACHE[name]

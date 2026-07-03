import csv
import numpy as np
import pyaudio
import tensorflow as tf
import kagglehub
import yasmin
import yasmin_ros
import time

class DetectDoorbell(yasmin.StateMachine):
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000  # YAMNet strictly requires 16kHz sample rate
    CHUNK_SIZE = 1024 
    REQUIRED_SAMPLES = 15600  # ~0.975s segments required by YAMNet
    WAIT_FOR_DOORBELL_TIMEOUT = 30
    
    def __init__(self, device_id=0, score_threshold=0.25, excluded_classes=['Speech', 'Silence']):
        super().__init__(outcomes=["succeeded", "failed"])
        self.device_id = device_id
        self.score_threshold = score_threshold
        self.excluded_classes = excluded_classes
        
        # State variables
        self.model = None
        self.class_names = []
        self.audio_interface = None
        self.stream = None
        self.audio_buffer = np.zeros(0, dtype=np.float32)

        self.load_model()
        self.load_class_map()
        self.initialize_audio()

    def load_model(self):
        model_path = kagglehub.model_download("google/yamnet/tensorFlow2/yamnet")
        self.model = tf.saved_model.load(model_path)

    def load_class_map(self):
        if self.model is None:
            raise RuntimeError("Model must be loaded before extracting the class map.")
            
        class_map_path = self.model.class_map_path().numpy().decode('utf-8')
        self.class_names = []
        with open(class_map_path, 'r', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.class_names.append(row['display_name'])

    def initialize_audio(self):
        self.audio_interface = pyaudio.PyAudio()
        self.stream = self.audio_interface.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            input_device_index=self.device_id,
            frames_per_buffer=self.CHUNK_SIZE
        )

    def process_audio_frame(self):
        data = self.stream.read(self.CHUNK_SIZE, exception_on_overflow=False)
        audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        
        yasmin_ros.logger_node.get_logger().info(f"Max amplitude: {np.max(np.abs(audio_chunk)):.4f}", end='\r')
        self.audio_buffer = np.append(self.audio_buffer, audio_chunk)

    def run_inference(self):
        if len(self.audio_buffer) >= self.REQUIRED_SAMPLES:
            input_data = self.audio_buffer[-self.REQUIRED_SAMPLES:]
            
            scores, _, _ = self.model(input_data)
            
            mean_scores = np.mean(scores.numpy(), axis=0)
            top_class_index = np.argmax(mean_scores)
            top_score = mean_scores[top_class_index]
            prediction_name = self.class_names[top_class_index]

            if top_score > self.score_threshold and prediction_name not in self.excluded_classes:
                print(f" Detected: {prediction_name:<25} (Score: {top_score:.2f})")
                return True

            self.audio_buffer = self.audio_buffer[-int(self.REQUIRED_SAMPLES / 2):]
            return False

    def cleanup(self):
        print("\nStopping stream...")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio_interface:
            self.audio_interface.terminate()
        print("Done.")

    def execute(self):
        try:
            t_end = time.time() + self.WAIT_FOR_DOORBELL_TIMEOUT
            while time.time() < t_end and not found:
                self.process_audio_frame()
                found = self.run_inference()

            if found:
                return "succeeded"
            else:
                return "failed"
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

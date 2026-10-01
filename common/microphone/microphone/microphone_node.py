import array
import queue
import threading
from collections import deque

import numpy as np
import rclpy
import sounddevice as sd
import torch
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from silero_vad import load_silero_vad

from lasr_speech_recognition_interfaces.srv import RecordAudio

SAMPLE_RATE = 16000
CHUNK_SIZE = 512
PRE_ROLL_CHUNKS = 16  # ~512 ms of audio kept from before speech starts
CHUNK_TIMEOUT = 0.5  # seconds without a chunk before the stream is considered dead


class MicrophoneNode(Node):
    """Owns the microphone and serves recordings via the /microphone/record service."""

    def __init__(self):
        super().__init__('microphone_node')

        self.declare_parameter('mic_device', 'default')
        self.declare_parameter('start_timeout', 5.0)
        self.declare_parameter('pause_threshold', 2.0)
        self.declare_parameter('max_phrase_duration', 15.0)

        self._mic_device = self.get_parameter('mic_device').value or None
        self._start_timeout = self.get_parameter('start_timeout').value
        self._pause_threshold = self.get_parameter('pause_threshold').value
        self._max_phrase_duration = self.get_parameter('max_phrase_duration').value

        self._audio_queue = queue.Queue()
        self._collecting = False
        # Only one recording can use the microphone at a time
        self._record_lock = threading.Lock()

        self._vad_model = load_silero_vad()

        self._stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='float32',
            blocksize=CHUNK_SIZE,
            device=self._resolve_mic_device(),
            callback=self._audio_callback,
        )
        self._stream.start()

        self._record_srv = self.create_service(
            RecordAudio, '/microphone/record', self._record_cb
        )

        self.get_logger().info('Microphone node has started')

    def _resolve_mic_device(self):
        if self._mic_device is None:
            return None
        if self._mic_device.isdigit():
            return int(self._mic_device)
        for idx, info in enumerate(sd.query_devices()):
            if self._mic_device in info['name']:
                return idx
        raise ValueError(f'Could not find microphone: {self._mic_device}')

    def _audio_callback(self, indata: np.ndarray, frames: int, time_info, status):
        if self._collecting:
            self._audio_queue.put_nowait(indata[:, 0].copy())

    def _start_collecting(self):
        # Drop any stale chunks left over from a previous recording
        with self._audio_queue.mutex:
            self._audio_queue.queue.clear()
        self._collecting = True

    def _next_chunk(self):
        return self._audio_queue.get(timeout=CHUNK_TIMEOUT)

    def _record_cb(self, request, response):
        response.sample_rate = SAMPLE_RATE

        with self._record_lock:
            try:
                if request.mode == 'phrase':
                    chunks = self._record_phrase(request)
                elif request.mode == 'fixed':
                    chunks = self._record_fixed(request)
                else:
                    response.success = False
                    response.message = f'Unknown mode: {request.mode!r}'
                    self.get_logger().warn(response.message)
                    return response
            except queue.Empty:
                chunks = None
                response.message = 'No audio received from the microphone'
                self.get_logger().error(response.message)
            except Exception as e:
                chunks = None
                response.message = f'Recording error: {e}'
                self.get_logger().error(response.message)
            finally:
                self._collecting = False

        if not chunks:
            response.success = False
            if not response.message:
                response.message = 'No speech detected'
            return response

        samples = np.concatenate(chunks).astype(np.float32)
        response.samples = array.array('f', samples.tobytes())
        response.success = True
        response.message = f'Recorded {len(samples) / SAMPLE_RATE:.2f}s'
        self.get_logger().info(response.message)
        return response

    def _record_fixed(self, request):
        if request.duration <= 0.0:
            raise ValueError('duration must be > 0 in fixed mode')
        target_chunks = max(1, int(request.duration * SAMPLE_RATE / CHUNK_SIZE))
        self.get_logger().info(f'Recording {request.duration:.2f}s of audio')

        self._start_collecting()
        return [self._next_chunk() for _ in range(target_chunks)]

    def _record_phrase(self, request):
        start_timeout = request.start_timeout or self._start_timeout
        pause_threshold = request.pause_threshold or self._pause_threshold
        max_start_chunks = int(start_timeout * SAMPLE_RATE / CHUNK_SIZE)
        max_silent_chunks = int(pause_threshold * SAMPLE_RATE / CHUNK_SIZE)
        max_phrase_chunks = int(self._max_phrase_duration * SAMPLE_RATE / CHUNK_SIZE)

        self._vad_model.reset_states()
        self.get_logger().info('Waiting for a phrase')

        self._start_collecting()
        pre_roll = deque(maxlen=PRE_ROLL_CHUNKS)
        start_chunks_elapsed = 0
        while True:
            chunk = self._next_chunk()
            pre_roll.append(chunk)
            if self._is_speech(chunk):
                collected_chunks = list(pre_roll)
                break
            start_chunks_elapsed += 1
            if start_chunks_elapsed > max_start_chunks:
                self.get_logger().warn('Start timeout - no speech detected.')
                return None

        silent_chunks = 0
        while len(collected_chunks) < max_phrase_chunks:
            chunk = self._next_chunk()
            collected_chunks.append(chunk)
            if self._is_speech(chunk):
                silent_chunks = 0
            else:
                silent_chunks += 1
                if silent_chunks >= max_silent_chunks:
                    return collected_chunks

        self.get_logger().warn('Max phrase duration reached.')
        return collected_chunks

    def _is_speech(self, chunk):
        return self._vad_model(
            torch.from_numpy(chunk).unsqueeze(0), SAMPLE_RATE
        ).item() > 0.5

    def destroy_node(self):
        self._stream.stop()
        self._stream.close()
        super().destroy_node()


def main():
    rclpy.init()

    microphone_node = MicrophoneNode()

    try:
        rclpy.spin(microphone_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        microphone_node.destroy_node()
        rclpy.try_shutdown()

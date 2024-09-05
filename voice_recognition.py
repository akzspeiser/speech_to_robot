import vosk
import json
import pyaudio

class VoiceRecognition:
    def __init__(self):
        # Load the Vosk model
        self.model = vosk.Model("/home/zac/Downloads/vosk")
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )
        self.stream.start_stream()

    def activate_phrase(self):
        print("Listening for activation phrase...")
        while True:
            data = self.stream.read(4000)
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                if 'text' in result:
                    print(f"Heard: {result['text']}")  # Print recognized words
                    if result['text'].lower() == "hey robot":
                        print("Activation phrase detected.")
                        return True

    def recognize_speech(self):
        print("Listening for commands...")
        valid_commands = {"poke object", "reset position", "never mind", "close program"}
        while True:
            data = self.stream.read(4000)
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                if 'text' in result and result['text']:
                    command = result['text']
                    print(f"Heard: {command}")  # Print what was heard
                    if command.lower() in valid_commands:
                        print(f"Command recognized: {command}")
                        return command

    def pause_recognition(self):
        print("Pausing recognition...")
        self.stream.stop_stream()

    def resume_recognition(self):
        print("Resuming recognition...")
        self.stream.start_stream()

    def shutdown(self):
        # Close the stream and PyAudio instance
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

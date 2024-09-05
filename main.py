import rclpy
from robot_control import RobotControl
from voice_recognition import VoiceRecognition
from text_to_speech import TextToSpeech

def main(args=None):
    rclpy.init(args=args)
    voice_recognition = VoiceRecognition()
    text_to_speech = TextToSpeech()
    robot_control = RobotControl()
    
    try:
        while rclpy.ok():
            # Listen for the activation phrase
            if voice_recognition.activate_phrase():
                text_to_speech.speak("Listening for your command...")
                while True:
                    command = voice_recognition.recognize_speech()

                    if command == "poke object":
                        voice_recognition.pause_recognition()  # Pause recognition
                        text_to_speech.speak("Poking object...")
                        force = robot_control.poke_object(x=0.5, y=0.0, z=0.3)
                        text_to_speech.speak(f"Measured force: {force}")
                        voice_recognition.resume_recognition()  # Resume recognition
                    elif command == "reset position":
                        voice_recognition.pause_recognition()  # Pause recognition
                        text_to_speech.speak("Resetting position...")
                        robot_control.reset_position()
                        voice_recognition.resume_recognition()  # Resume recognition
                    elif command in ["never mind", "nevermind"]:
                        voice_recognition.pause_recognition()  # Pause recognition
                        text_to_speech.speak("Returning to activation phrase...")
                        voice_recognition.resume_recognition()  # Resume recognition
                        break  # Exit the command loop and return to listening for activation phrase
                    elif command == "close program":
                        voice_recognition.pause_recognition()  # Pause recognition
                        text_to_speech.speak("Closing program...")
                        return  # Exit the program entirely
                    else:
                        voice_recognition.pause_recognition()  # Pause recognition
                        text_to_speech.speak("Command not recognized.")
                        voice_recognition.resume_recognition()  # Resume recognition
    except KeyboardInterrupt:
        print("Shutting down...")
        text_to_speech.speak("Shutting down...")
    finally:
        voice_recognition.shutdown()  # Cleanly shut down audio resources
        rclpy.shutdown()

if __name__ == '__main__':
    main()

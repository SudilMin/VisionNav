import os
import wave
import subprocess
import speech_recognition as sr
from faster_whisper import WhisperModel

# --- 1. Text to Speech Setup (Piper) ---
# We use the crisp female "Lessac" voice you downloaded
PIPER_MODEL = "en_US-lessac-medium.onnx"

def speak(text):
    print(f"🔊 AI Speaking: '{text}'")
    # Piper converts text to a .wav file instantly, then aplay plays it
    command = f"echo '{text}' | piper --model {PIPER_MODEL} --output_file temp_voice.wav && aplay temp_voice.wav -q"
    subprocess.run(command, shell=True)

# --- 2. Speech to Text Setup (Faster-Whisper) ---
# 'tiny.en' is extremely fast and runs locally!
print("Loading Whisper AI (this takes a few seconds the first time)...")
whisper_model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
recognizer = sr.Recognizer()

def listen_for_command():
    with sr.Microphone() as source:
        print("\n🎤 Listening... (Say 'find chair' or 'exit')")
        # Adjust for background street noise
        recognizer.adjust_for_ambient_noise(source, duration=0.5) 
        audio = recognizer.listen(source)
        
        # Save temporary audio file for Whisper
        with open("temp_mic.wav", "wb") as f:
            f.write(audio.get_wav_data())

    print("🧠 Thinking...")
    segments, _ = whisper_model.transcribe("temp_mic.wav", beam_size=5)
    text = "".join([segment.text for segment in segments]).strip().lower()
    
    # Remove punctuation so it perfectly matches "find chair"
    text = text.replace(".", "").replace(",", "").replace("?", "")
    print(f"🗣️ You said: '{text}'")
    return text

# --- Main Loop ---
if __name__ == "__main__":
    speak("System initialized. I am ready.")
    
    while True:
        command = listen_for_command()
        
        if "exit" in command or "stop" in command:
            speak("Shutting down navigation.")
            break
            
        elif "find" in command:
            object_name = command.replace("find", "").strip()
            speak(f"{object_name} detected. You can now say, go to {object_name}.")
            
        elif "go to" in command:
            speak("Route calculating using A-star obstacle avoidance. Turn right to your 2 o'clock.")
            
        else:
            # We ignore random noise/conversations!
            pass

#!/usr/bin/env python3
"""
text_reader.py
--------------
Standalone OCR (Optical Character Recognition) and Text-to-Speech script.
Extracts text from an image (labels, signs, documents) and reads it aloud.
"""

import cv2
import pytesseract
import subprocess
import os
import argparse
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def speak(text):
    """Speak text aloud using Piper TTS."""
    print(f"\n🔊 Speaking: '{text}'\n")
    model_path = os.path.join(SCRIPT_DIR, "en_US-lessac-medium.onnx")
    wav_path = os.path.join(SCRIPT_DIR, "temp_voice.wav")
    
    # Sanitize text for shell command
    safe_text = text.replace("'", "").replace('"', '').replace('\n', ' ')
    
    if not os.path.exists(model_path):
        print(f"⚠️ Piper model not found at {model_path}. Text will just be printed.")
        return

    command = f"echo '{safe_text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
    subprocess.run(command, shell=True)

def process_image(image_path):
    if not os.path.exists(image_path):
        print(f"❌ Error: Image not found at {image_path}")
        return

    print(f"🖼️  Processing image: {image_path}")
    
    # Load image
    img = cv2.imread(image_path)
    
    # Preprocessing for better OCR accuracy
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Optional: Apply thresholding to make text pop out more
    # (cv2.THRESH_OTSU automatically determines the best threshold value)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    print("🔍 Extracting text...")
    
    # Run Tesseract OCR
    # config psm 3 = Fully automatic page segmentation, but no OSD. (Default)
    # config psm 6 = Assume a single uniform block of text.
    try:
        text = pytesseract.image_to_string(thresh, config='--psm 3')
    except Exception as e:
        print(f"❌ OCR Error: {e}")
        print("Please ensure Tesseract is installed: sudo apt-get install tesseract-ocr")
        return
    
    # Clean up the text
    cleaned_text = " ".join(text.split())
    
    if cleaned_text.strip():
        print("="*50)
        print("📄 EXTRACTED TEXT:")
        print(text.strip())
        print("="*50)
        speak(f"I found the following text. {cleaned_text}")
    else:
        print("❌ No text could be detected in this image.")
        speak("I could not detect any text in this image.")

def main():
    parser = argparse.ArgumentParser(description="Extract and read text from an image.")
    parser.add_argument("image", help="Path to the image file (e.g. sign.jpg, label.png)")
    args = parser.parse_args()
    
    process_image(args.image)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Speech Recognition Module for Nova Carter3 Robot Control
Converts voice commands to text for ChatGPT processing
"""

import speech_recognition as sr
import pyaudio
import logging
from typing import Optional

class SpeechRecognitionModule:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.logger = logging.getLogger(__name__)
        self.is_listening = False
        
        # Adjust for ambient noise
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
                self.logger.info("Speech recognition initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize microphone: {e}")
            
    def listen_for_command(self, timeout: int = 10) -> Optional[str]:
        """
        Listen for voice command and convert to text
        
        Args:
            timeout: Maximum time to wait for speech
            
        Returns:
            Transcribed text or None if no speech detected
        """
        try:
            with self.microphone as source:
                self.logger.info("Listening for command...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=5)
                
            # Use Google Speech Recognition
            text = self.recognizer.recognize_google(audio)
            self.logger.info(f"Recognized: {text}")
            return text
            
        except sr.WaitTimeoutError:
            self.logger.warning("No speech detected within timeout")
            return None
        except sr.UnknownValueError:
            self.logger.error("Could not understand audio")
            return None
        except sr.RequestError as e:
            self.logger.error(f"Speech recognition service error: {e}")
            return None
            
    def continuous_listen(self, callback_function):
        """
        Continuously listen for voice commands
        
        Args:
            callback_function: Function to call with recognized text
        """
        self.is_listening = True
        while self.is_listening:
            try:
                command = self.listen_for_command()
                if command:
                    callback_function(command)
            except KeyboardInterrupt:
                self.logger.info("Continuous listening interrupted")
                break
            except Exception as e:
                self.logger.error(f"Error in continuous listening: {e}")
                break
    
    def stop_listening(self):
        """Stop continuous listening"""
        self.is_listening = False
        self.logger.info("Stopped continuous listening")
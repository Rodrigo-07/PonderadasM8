import os
from ibm_watson import SpeechToTextV1, TextToSpeechV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from dotenv import load_dotenv

load_dotenv()

# STT
def get_speech_to_text():
    stt_authenticator = IAMAuthenticator(os.getenv("STT_API_KEY"))
    speech_to_text = SpeechToTextV1(authenticator=stt_authenticator)
    speech_to_text.set_service_url(os.getenv("STT_SERVICE_URL"))
    return speech_to_text

def transcribe_audio(audio_data):
    speech_to_text = get_speech_to_text()
    response = speech_to_text.recognize(
        audio=audio_data,
        content_type='audio/wav',
        model='en-US'
    ).get_result()
    results = response.get('results', [])
    return results[0]['alternatives'][0]['transcript'].strip() if results else ""

# TTS

def get_text_to_speech():
    tts_authenticator = IAMAuthenticator(os.getenv("TTS_API_KEY"))
    text_to_speech = TextToSpeechV1(authenticator=tts_authenticator)
    text_to_speech.set_service_url(os.getenv("TTS_SERVICE_URL"))
    return text_to_speech

def synthesize_speech(text, voice='en-US_AllisonV3Voice', accept='audio/mp3'):
    text_to_speech = get_text_to_speech()
    return text_to_speech.synthesize(
        text, voice=voice, accept=accept
    ).get_result().content

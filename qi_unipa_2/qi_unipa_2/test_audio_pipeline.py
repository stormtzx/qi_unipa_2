#!/usr/bin/env python3
# test_audio_pipeline.py

import os
import sys
import sounddevice as sd
from openai import OpenAI

print("=" * 80)
print("🔧 TEST AUDIO PIPELINE")
print("=" * 80)

# 1. Test sounddevice
print("\n1️⃣  TEST SOUNDDEVICE")
try:
    devices = sd.query_devices()
    print(f"   ✅ Dispositivi trovati: {len(devices)}")
    for i, dev in enumerate(devices):
        if dev['max_input_channels'] > 0:
            print(f"      [{i}] {dev['name']} (input channels: {dev['max_input_channels']})")
except Exception as e:
    print(f"   ❌ ERRORE: {e}")
    sys.exit(1)

# 2. Test OpenAI API key
print("\n2️⃣  TEST OPENAI API KEY")
api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    print("   ❌ ERRORE: OPENAI_API_KEY non trovata!")
    sys.exit(1)
print(f"   ✅ API key trovata: {api_key[:10]}...")

# 3. Test OpenAI client
print("\n3️⃣  TEST OPENAI CLIENT")
try:
    client = OpenAI(api_key=api_key)
    print("   ✅ OpenAI client creato")
except Exception as e:
    print(f"   ❌ ERRORE: {e}")
    sys.exit(1)

# 4. Test recording
print("\n4️⃣  TEST REGISTRAZIONE (3 secondi)")
try:
    print("   Parla adesso...")
    audio = sd.rec(int(3 * 16000), samplerate=16000, channels=1, dtype='int16')
    sd.wait()
    print(f"   ✅ Registrazione completata: {audio.shape[0]} samples")
except Exception as e:
    print(f"   ❌ ERRORE: {e}")
    sys.exit(1)

print("\n" + "=" * 80)
print("✅ TUTTI I TEST PASSATI")
print("=" * 80)

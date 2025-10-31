import qi
import requests
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import json


class Utils:
    def __init__(self, ip="192.168.0.102", port=9559, mock_mode=False):
        self.ip = ip
        self.port = port
        self.mock_mode = mock_mode
        self.session = None
        self.logger_error = f"Can't connect to Naoqi at ip \"{self.ip}\" on port {self.port}.\nPlease check your script arguments."
        
        # Connessione sessione SOLO se non mock
        if not mock_mode:
            self.session = qi.Session()
            self.session = self.set_connection()
        else:
            print("[UTILS] MOCK MODE - Nessuna connessione a Pepper")

        # URL dell'API Groq
        self.API_URL = "https://api.groq.com/openai/v1/audio/transcriptions"
        
        # Token API di Groq (opzionale in mock)
        self.API_TOKEN = os.environ.get("GROQ_API_KEY")
        
        # NON solleva errore in mock mode
        if not self.API_TOKEN and not mock_mode:
            print("[WARNING] GROQ_API_KEY non trovata - trascrizioni Groq disabilitate")
        
        # Headers per le richieste API
        self.headers = {
            "Authorization": f"Bearer {self.API_TOKEN}",
            "Accept": "application/json"
        } if self.API_TOKEN else {}

    def set_connection(self):
        if self.mock_mode:
            return None
            
        try:
            self.session.connect(f"tcp://{self.ip}:{self.port}")
            return self.session
        except RuntimeError:
            print(self.logger_error)
            # NON fare sys.exit(), solleva eccezione
            raise RuntimeError(f"Cannot connect to Pepper at {self.ip}:{self.port}")
    
    def get_ip(self):
        return self.ip
    
    def get_port(self):
        return self.port
    
    def get_QoS(self, reliability_type='reliable', qos_depth=10):
        qos = QoSProfile(depth=qos_depth)
        qos.history = HistoryPolicy.KEEP_LAST
  
        if reliability_type.lower() == 'reliable':
            qos.reliability = ReliabilityPolicy.RELIABLE
        elif reliability_type.lower() == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT 
        else:
            raise ValueError("Tipo di affidabilità QoS sconosciuto: scegliere 'reliable' o 'best_effort'")

        return qos
    
    def transcribe_audio(self, audio_path):
        if self.mock_mode:
            return "[MOCK] Trascrizione simulata"
        
        if not self.API_TOKEN:
            raise RuntimeError("GROQ_API_KEY non configurata")
            
        try:
            files = {
                'file': (os.path.basename(audio_path), open(audio_path, 'rb'), 'audio/mpeg'),
                'model': (None, 'whisper-large-v3-turbo'),
                'language': (None, 'it'),
            }
            
            response = requests.post(
                self.API_URL, 
                headers=self.headers,
                files=files
            )
            
            if response.status_code == 200:
                result = response.json()
                return result.get("text", "Trascrizione non disponibile")
            else:
                raise ValueError(f"Errore API Groq: {response.status_code}, {response.text}")
                
        except Exception as e:
            raise RuntimeError(f"Errore durante la trascrizione: {e}")
        finally:
            if 'files' in locals() and 'file' in files:
                files['file'][1].close()

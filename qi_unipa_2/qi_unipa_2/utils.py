import qi
import requests
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import json
from ament_index_python.packages import get_package_share_directory
import logging
import socket


logger = logging.getLogger(__name__)  # ← con "__name__" usa il nome del modulo (quindi "utils")


class Utils:
    def __init__(self, ip="192.168.0.101", port=9559, mock_mode=False):
        self.ip = ip
        self.port = port
        self.mock_mode = mock_mode
        self.session = None
        self.logger_error = f"Impossibile connettersi a pepper all'ip: \"{self.ip}\" sulla porta: {self.port}.\n."
        
        # Connessione sessione SOLO se non mock
        if not mock_mode:
            self.session = qi.Session()
            self.session = self.set_connection()
        else:
            print("[UTILS] Avvio utilizzo del file utils in MOCK MODE")

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
            raise RuntimeError(f"Impossibile connettersi a pepper sull'indirizzo {self.ip}:{self.port}")
    
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
     
    @staticmethod
    def get_html_pages_dir(): #Metodo statico, il self non serve
        """
        Ottiene path portabile alla cartella html_pages.
        Usa ament_index per portabilità su diversi sistemi.
        """
        try:
            package_dir = get_package_share_directory('qi_unipa_2')
            return os.path.join(package_dir, 'html_pages')
        except Exception as e:
            logger.warning(f"Impossibile usare ament_index: {e}")
            # Fallback per sviluppo
            current_dir = os.path.dirname(os.path.abspath(__file__))
            return os.path.join(current_dir, '..', 'html_pages')

    
    @staticmethod
    def get_local_ip(): #Metodo statico, il self non serve
        """
        Ottiene IP locale della macchina.
        
        Returns:
            str: Indirizzo IP locale (es: "192.168.1.100")
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            return "127.0.0.1"
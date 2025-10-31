import rclpy
import wave
import time
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from qi_unipa_2.utils import Utils
import json



class QiUnipa2_stt(Node):
    def __init__(self):
        super().__init__("qi_unipa_2_stt")
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        
        # Lettura parametri
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        
        # Crea Utils (STT non usa connessione Pepper, quindi mock_mode solo per riferimento)
        utils = Utils(mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        
        # Subscribers
        self.stt_sub = self.create_subscription(String, "/stt", self.stt_callback, qos_reliable_10)
        self.stt_bdi_sub = self.create_subscription(String, "/stt_bdi", self.stt_bdi_callback, qos_reliable_10)
        self.risposta_sub = self.create_subscription(String, "/risposta_si_no", self.condividi_risposta, qos_reliable_10)
        
        # Publishers
        self.transcription_pub = self.create_publisher(String, "/transcription", qos_reliable_10)
        self.transcription_bdi_pub = self.create_publisher(String, "/transcription_bdi", qos_reliable_10)
        self.show_pub = self.create_publisher(String, '/show', qos_reliable_10)
        self.hide_pub = self.create_publisher(Bool, '/hide', qos_reliable_10)
        
        # Variabili BDI
        self.agente = "agente"
        self.percept = "percept"
        
        # Setup STT basato su mock_mode
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Trascrizioni simulate (no Whisper)")
            self.local_whisper = None
            self.mock_transcriptions = [
                "Ciao, come stai?",
                "Sì, va bene",
                "No, grazie",
                "Perfetto, procediamo",
                "Questa è una trascrizione simulata"
            ]
            self.mock_counter = 0
        else:
            self.get_logger().info("Modalità NORMALE - Caricamento Whisper...")
            try:
                import whisper
                self.local_whisper = whisper.load_model("medium")
                self.get_logger().info("Whisper caricato con successo")
            except Exception as e:
                self.get_logger().error(f"Errore caricamento Whisper: {e}")
                self.get_logger().warn("Fallback a MOCK mode")
                self.local_whisper = None
                self.mock_transcriptions = [
                    "Ciao, come stai?",
                    "Sì, va bene",
                    "No, grazie",
                    "Perfetto, procediamo",
                    "Questa è una trascrizione simulata"
                ]
                self.mock_counter = 0



    def stt_callback(self, msg):
        """Callback per trascrizione audio normale"""
        audio_path = msg.data
        self.get_logger().info(f"Path audio ricevuto: {audio_path}")
        
        # MOCK: Trascrizione simulata
        if self.local_whisper is None:
            transcription = self.mock_transcriptions[self.mock_counter % len(self.mock_transcriptions)]
            self.mock_counter += 1
            self.get_logger().info(f"[MOCK] Trascrizione simulata: {transcription}")
        
        # REALE: Trascrizione con Whisper
        else:
            try:
                self.get_logger().info("Trascrizione in corso con Whisper...")
                result = self.local_whisper.transcribe(audio_path, language="it")
                transcription = result['text']
                self.get_logger().info(f"Trascrizione completata: {transcription}")
            except Exception as e:
                self.get_logger().error(f"Errore trascrizione: {e}")
                transcription = "[ERRORE TRASCRIZIONE]"
        
        # Pubblica trascrizione
        res = String()
        res.data = transcription
        self.transcription_pub.publish(res)
        
        self.get_logger().info(f"\n{'='*50}\nTRASCRIZIONE: {transcription}\n{'='*50}")


    def stt_bdi_callback(self, msg):
        """Callback per richieste STT da BDI agent"""
        try:
            data = json.loads(msg.data)
            self.agente = data.get("agente", "agente")
            self.percept = data.get("percept", "percept")
            
            self.get_logger().info(f"BDI - Agente: {self.agente}, Percept: {self.percept}")
            
            # Mostra pagina HTML per input
            url = "si_no.html"
            msg_show = String()
            msg_show.data = url
            self.show_pub.publish(msg_show)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Errore parsing JSON: {e}")


    def condividi_risposta(self, msg):
        """Gestisce risposta da pagina HTML (si/no)"""
        risposta = msg.data
        self.get_logger().info(f"Risposta ricevuta: {risposta}")
        
        # Nascondi pagina HTML se risposta è "no"
        if risposta == 'no':
            hide_msg = Bool()
            hide_msg.data = True
            self.hide_pub.publish(hide_msg)
        
        # Crea messaggio JSON per BDI
        messaggio_json = {
            "trascrizione": risposta,
            "agente": self.agente,
            "percept": self.percept
        }
        
        # Pubblica risposta per BDI
        res = String()
        res.data = json.dumps(messaggio_json)
        self.transcription_bdi_pub.publish(res)
        
        self.get_logger().info(f"Risposta BDI pubblicata: {messaggio_json}")


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_stt()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

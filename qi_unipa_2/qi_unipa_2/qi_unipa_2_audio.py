import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
import json
import math
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.action import Browsing, Navigating
from qi_unipa_2_interfaces.srv import MoveToSound
import random



class QiUnipa2_audio(Node):
    def __init__(self):
        super().__init__("qi_unipa_2_audio")
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        
        self.mock_mode = mock_mode
        self.local_whisper = None
        self.mock_transcriptions = [
            "Ciao, come stai?",
            "Sì, va bene",
            "No, grazie",
            "Perfetto, procediamo",
            "Questa è una trascrizione simulata"
        ]
        self.mock_counter = 0
        
        # Caricamento modello Whisper
        if not mock_mode:
            try:
                import whisper
                self.local_whisper = whisper.load_model("medium")
                self.get_logger().info("Whisper caricato con successo")
            except Exception as e:
                self.get_logger().error(f"Errore caricamento Whisper: {e}")
                self.local_whisper = None
        
        # Action Client per Browsing
        self.browsing_client = ActionClient(self, Browsing, '/pepper/actions/browsing')
        
        # Action Client per Navigating
        self.navigating_client = ActionClient(self, Navigating, '/pepper/actions/navigating')
        
        # Publisher per posizione suono
        self.sound_location_pub = self.create_publisher(PointStamped,'/pepper/topics/sound_location',10)
        
        # Service per navigazione verso suono
        self.move_to_sound_service = self.create_service(MoveToSound,'/pepper/services/move_to_sound',self.move_to_sound_callback)
        
        # Variabile per memorizzazione ultima posizione suono rilevata
        self.last_sound_location = None
        
        # Subscriptions per STT
        self.risposta_sub = self.create_subscription(String, "/pepper/topics/risposta_si_no", self.condividi_risposta, qos_reliable_10)
        
        # Connessione a Pepper per audio localization
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.memory = None
            self.audio_service = None
            
            # Timer per pubblicazione suoni mock
            self.create_timer(3.0, self.publish_mock_sound)
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                self.memory = self.session.service("ALMemory")
                self.audio_service = self.session.service("ALAudioSourceLocalization")
                
                # Configurazione sensibilità audio localization
                self.audio_service.setParameter("Sensitivity", 0.8)
                
                # Sottoscrizione evento suono localizzato
                self.memory.subscribeToEvent(
                    "ALAudioSourceLocalization/SoundLocated",
                    self.get_name(),
                    self.on_sound_located
                )
                
                self.get_logger().info("Audio localization attivo")
                
            except Exception as e:
                self.get_logger().error(f"Errore connessione Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.memory = None
                self.audio_service = None
                
                # Timer per pubblicazione suoni mock
                self.create_timer(3.0, self.publish_mock_sound)
        
        self.get_logger().info("Nodo audio avviato")


    # ========================================
    # AUDIO LOCALIZATION
    # ========================================
    
    def on_sound_located(self, event_name, value, subscriber_identifier):
        """Callback evento suono localizzato da ALAudioSourceLocalization"""
        try:
            sound_data = value[0]
            azimuth = sound_data[0]      # Angolo orizzontale (radianti)
            elevation = sound_data[1]    # Angolo verticale (radianti)
            confidence = sound_data[2]   # Confidenza 0-1
            
            self.get_logger().info(
                f"Suono rilevato: azimuth={math.degrees(azimuth):.1f} gradi, "
                f"elevation={math.degrees(elevation):.1f} gradi, confidence={confidence:.2f}"
            )
            
            # Pubblicazione coordinate suono
            self.publish_sound_location(azimuth, elevation, confidence)
            
        except Exception as e:
            self.get_logger().error(f"Errore callback audio: {e}")


    def publish_sound_location(self, azimuth, elevation, confidence):
        """Pubblicazione posizione suono su topic"""
        
        # Stima distanza fissa (parametro configurabile)
        distance = 2.0
        
        # Conversione angoli sferici in coordinate cartesiane (robot frame)
        x = distance * math.cos(elevation) * math.cos(azimuth)
        y = distance * math.cos(elevation) * math.sin(azimuth)
        z = distance * math.sin(elevation)
        
        # Creazione messaggio
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        # Pubblicazione topic
        self.sound_location_pub.publish(msg)
        
        # Memorizzazione ultima posizione
        self.last_sound_location = {
            "x": x,
            "y": y,
            "z": z,
            "confidence": confidence,
            "timestamp": self.get_clock().now()
        }
        
        self.get_logger().debug(f"Posizione suono: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")


    def publish_mock_sound(self):
        """Pubblicazione suono mock per testing"""
        
        azimuth = random.uniform(-math.pi/4, math.pi/4)
        elevation = 0.0
        confidence = 0.8
        
        self.get_logger().debug(f"[MOCK] Suono simulato: azimuth={math.degrees(azimuth):.1f} gradi")
        self.publish_sound_location(azimuth, elevation, confidence)


    def move_to_sound_callback(self, request, response):
        """Service callback per navigazione verso ultima posizione suono rilevata"""
        
        approach_distance = request.approach_distance if hasattr(request, 'approach_distance') else 0.5
        
        # Verifica presenza suono rilevato
        if self.last_sound_location is None:
            response.success = False
            response.message = "Nessun suono rilevato"
            return response
        
        # Verifica recency del suono (timeout 10 secondi)
        time_since_sound = (self.get_clock().now() - self.last_sound_location["timestamp"]).nanoseconds / 1e9
        if time_since_sound > 10.0:
            response.success = False
            response.message = f"Ultimo suono rilevato {time_since_sound:.1f}s fa (timeout superato)"
            return response
        
        # Verifica confidence minima
        if self.last_sound_location["confidence"] < 0.5:
            response.success = False
            response.message = f"Confidence troppo bassa: {self.last_sound_location['confidence']:.2f}"
            return response
        
        # Calcolo coordinate target con offset approach_distance
        target_x = self.last_sound_location["x"]
        target_y = self.last_sound_location["y"]
        
        distance = math.sqrt(target_x**2 + target_y**2)
        if distance > approach_distance:
            # Riduzione distanza per fermata a approach_distance dal suono
            scale = (distance - approach_distance) / distance
            target_x *= scale
            target_y *= scale
        else:
            response.success = False
            response.message = f"Suono troppo vicino ({distance:.2f}m)"
            return response
        
        self.get_logger().info(f"Navigazione verso suono: target=({target_x:.2f}, {target_y:.2f})")
        
        # Verifica disponibilità action server navigating
        if not self.navigating_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Navigating action server non disponibile"
            return response
        
        # Invio goal navigating
        goal = Navigating.Goal()
        goal.target_x = target_x
        goal.target_y = target_y
        goal.use_map = False
        
        future = self.navigating_client.send_goal_async(goal)
        
        # Attesa accettazione goal
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response.success = True
            response.message = f"Navigazione avviata verso suono"
            response.target_x = target_x
            response.target_y = target_y
        else:
            response.success = False
            response.message = "Errore avvio navigazione"
        
        return response


    # ========================================
    # SPEECH-TO-TEXT (STT)
    # ========================================

    def send_browsing(self, html_page, use_tablet=False):
        """Invio action Browsing per visualizzazione pagina HTML"""
        if not self.browsing_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server browsing non disponibile")
            return
        
        goal_msg = Browsing.Goal()
        goal_msg.html_page = html_page
        goal_msg.use_tablet = use_tablet
        
        self.get_logger().info(f"Invio action Browsing: page='{html_page}', use_tablet={use_tablet}")
        send_goal_future = self.browsing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.browsing_goal_response_callback)


    def browsing_goal_response_callback(self, future):
        """Callback risposta accettazione/rifiuto action Browsing"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Action Browsing rifiutata")
            return
        
        self.get_logger().info("Action Browsing accettata")
        
        # Attesa risultato finale
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.browsing_result_callback)


    def browsing_result_callback(self, future):
        """Callback risultato finale action Browsing"""
        result = future.result().result
        self.get_logger().info(f"Action Browsing completata: success={result.success}")


    def stt_callback(self, msg):
        """Callback per trascrizione audio standard"""
        audio_path = msg.data
        self.get_logger().info(f"Path audio ricevuto: {audio_path}")
        
        if self.local_whisper is None:
            transcription = self.mock_transcriptions[self.mock_counter % len(self.mock_transcriptions)]
            self.mock_counter += 1
            self.get_logger().info(f"[MOCK] Trascrizione simulata: {transcription}")
        else:
            try:
                self.get_logger().info("Trascrizione in corso con Whisper")
                result = self.local_whisper.transcribe(audio_path, language="it")
                transcription = result['text']
                self.get_logger().info(f"Trascrizione completata: {transcription}")
            except Exception as e:
                self.get_logger().error(f"Errore trascrizione: {e}")
                transcription = "[ERRORE TRASCRIZIONE]"
        
        self.get_logger().info(f"\n{'='*50}\nTRASCRIZIONE: {transcription}\n{'='*50}")


    def stt_bdi_callback(self, msg):
        """Callback per richieste STT da BDI agent"""
        try:
            data = json.loads(msg.data)
            self.agente = data.get("agente", "agente")
            self.percept = data.get("percept", "percept")
            
            self.get_logger().info(f"BDI - Agente: {self.agente}, Percept: {self.percept}")
            
            # Visualizzazione pagina HTML si_no.html tramite action Browsing
            self.send_browsing("si_no.html", use_tablet=True)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Errore parsing JSON: {e}")


    def condividi_risposta(self, msg):
        """Callback per gestione risposta utente (si/no)"""
        risposta = msg.data
        self.get_logger().info(f"Risposta ricevuta: {risposta}")
        
        if risposta == 'no':
            # Nascondimento pagina HTML tramite visualizzazione pagina vuota
            self.send_browsing("about:blank", use_tablet=True)
        
        # Preparazione messaggio JSON con risposta
        messaggio_json = {
            "trascrizione": risposta,
            "agente": getattr(self, 'agente', 'agente'),
            "percept": getattr(self, 'percept', 'percept')
        }
        
        self.get_logger().info(f"Risposta BDI preparata: {messaggio_json}")


    def destroy_node(self):
        """Cleanup risorse prima della distruzione del nodo"""
        if self.memory is not None:
            try:
                self.memory.unsubscribeToEvent(
                    "ALAudioSourceLocalization/SoundLocated",
                    self.get_name()
                )
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_audio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

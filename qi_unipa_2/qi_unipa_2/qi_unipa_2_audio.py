import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
import json
import math
import time
import os
import paramiko
import threading
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.action import Browsing, Navigating
from qi_unipa_2_interfaces.srv import MoveToSound
import random


class QiUnipa2_audio(Node):
    def __init__(self):
        super().__init__("qi_unipa_2_audio")

        
        # Parametri esistenti
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        
        self.ip = ip
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
                self.local_whisper = whisper.load_model("base")
                self.get_logger().info("Whisper caricato con successo")
            except Exception as e:
                self.get_logger().error(f"Errore caricamento Whisper: {e}")
                self.local_whisper = None
        
        # Action Client per Browsing
        self.browsing_client = ActionClient(self, Browsing, '/pepper/actions/browsing')
        
        # Action Client per Navigating
        self.navigating_client = ActionClient(self, Navigating, '/pepper/actions/navigating')
        
        # Publisher per posizione suono
        self.sound_location_pub = self.create_publisher(PointStamped, '/pepper/topics/sound_location', 10)
        
        # Publisher per audio registrato (path file)
        self.audio_recorded_pub = self.create_publisher(String, '/pepper/topics/audio_recorded', 10)
        
        # Publisher per trascrizione (testo per LLM)
        self.transcription_pub = self.create_publisher(String, '/pepper/topics/transcription', 10)
        
        # Service per navigazione verso suono
        self.move_to_sound_service = self.create_service(MoveToSound, '/pepper/services/move_to_sound', self.move_to_sound_callback)
        
        # Variabile per memorizzazione ultima posizione suono rilevata
        self.last_sound_location = None
        
        # Subscriptions per STT
        #self.risposta_sub = self.create_subscription(String, "/pepper/topics/risposta_si_no", self.condividi_risposta, qos_reliable_10)
        
        # Subscription per stato talking (auto-recording)
        self.is_pepper_talking = False
        self.is_recording = False
        self.create_subscription(Bool, '/pepper/topics/is_talking', self.on_talking_changed, 10)
        
        # Connessione a Pepper per audio localization e recording
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.memory = None
            self.audio_service = None
            self.audio_recorder = None
            self.sound_detect_service = None
            
            # Timer per pubblicazione suoni mock
            self.create_timer(3.0, self.publish_mock_sound)
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                self.memory = self.session.service("ALMemory")
                self.audio_service = self.session.service("ALAudioSourceLocalization")
                self.audio_recorder = self.session.service("ALAudioRecorder")
                self.sound_detect_service = self.session.service("ALSoundDetection")
                
                # Configurazione sensibilità audio localization
                self.audio_service.setParameter("Sensitivity", 0.8)
                
                # Configurazione sensibilità sound detection per recording
                self.sound_detect_service.setParameter("Sensitivity", 0.8)
                
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
                self.audio_recorder = None
                self.sound_detect_service = None
                
                # Timer per pubblicazione suoni mock
                self.create_timer(3.0, self.publish_mock_sound)
        
        self.get_logger().info("Nodo audio avviato")


    # ========================================
    # AUTO-RECORDING (triggered da /is_talking)
    # ========================================
    
    def on_talking_changed(self, msg):
        """Callback per cambio stato talking di Pepper"""
        was_talking = self.is_pepper_talking
        self.is_pepper_talking = msg.data
        
        # Trigger: Pepper ha finito di parlare
        if was_talking and not self.is_pepper_talking:
            self.get_logger().info("Pepper ha finito di parlare, avvio registrazione automatica")
            if not self.is_recording:
                self.start_recording_async()


    def start_recording_async(self):
        """Avvia registrazione + trascrizione in thread separato (non-blocking)"""
        def _recording_thread():
            audio_path = self.record_audio()
            
            if audio_path:
                # Pubblica path audio registrato
                msg = String()
                msg.data = audio_path
                self.audio_recorded_pub.publish(msg)
                self.get_logger().info(f"Audio registrato: {audio_path}")
                
                # Trascrivi audio
                transcription = self.transcribe_audio(audio_path)
                
                if transcription:
                    # Pubblica trascrizione per LLM
                    transcription_msg = String()
                    transcription_msg.data = transcription
                    self.transcription_pub.publish(transcription_msg)
                    self.get_logger().info(f"Trascrizione pubblicata: {transcription}")
                else:
                    self.get_logger().warn("Trascrizione fallita o vuota")
            else:
                self.get_logger().warn("Registrazione fallita (timeout o errore)")
        
        thread = threading.Thread(target=_recording_thread, daemon=True)
        thread.start()


    def record_audio(self):
        """
        Registra audio da microfoni Pepper usando AudioLocalization come trigger,
        poi disattiva localization durante registrazione per evitare interferenza.
        """
        if self.audio_recorder is None or self.memory is None:
            self.get_logger().warn("[MOCK] Registrazione simulata")
            time.sleep(2)
            return "/mock/path/recording.wav"
        
        if self.is_recording:
            return ""
        
        self.is_recording = True
        
        try:
            # Stop eventuali registrazioni precedenti
            try:
                self.audio_recorder.stopMicrophonesRecording()
            except:
                pass
            
            # Config registrazione
            channels = [1, 1, 1, 1]
            audio_format = "wav"
            sample_rate = 16000
            output_file_robot = "/home/nao/audio_record_unipa/recording.wav"
            
            # FASE 1: Attesa trigger da AudioLocalization (già attivo)
            self.get_logger().info("Attesa suono localizzato (trigger)...")
            
            sound_triggered = False
            trigger_timeout = 10
            start_time = time.time()
            
            while (time.time() - start_time) < trigger_timeout and not sound_triggered:
                time.sleep(0.2)
                
                # Controlla se c'è un suono localizzato recente con alta confidence
                if self.last_sound_location is not None:
                    time_since_sound = (self.get_clock().now() - self.last_sound_location["timestamp"]).nanoseconds / 1e9
                    
                    # Suono rilevato negli ultimi 0.3 secondi con confidence > 0.7
                    if time_since_sound < 0.3 and self.last_sound_location['confidence'] > 0.7:
                        sound_triggered = True
                        self.get_logger().info(
                            f"Trigger confermato da AudioLocalization: confidence={self.last_sound_location['confidence']:.2f}"
                        )
            
            if not sound_triggered:
                self.get_logger().warn("Timeout: nessun suono localizzato rilevato")
                self.is_recording = False
                return ""
            
            # FASE 2: Disattiva AudioLocalization per evitare interferenza durante registrazione
            self.get_logger().debug("Disattivazione AudioLocalization durante registrazione")
            try:
                self.memory.unsubscribeToEvent(
                    "ALAudioSourceLocalization/SoundLocated",
                    self.get_name()
                )
            except Exception as e:
                self.get_logger().warn(f"Errore disattivazione AudioLocalization: {e}")
            
            # FASE 3: Registrazione con SoundDetection
            self.sound_detect_service.subscribe("Audio Detection")
            
            # Attesa inizializzazione sound detection
            max_retries = 10
            retry_count = 0
            
            while retry_count < max_retries:
                try:
                    sound_data = self.memory.getData("SoundDetected")
                    if sound_data and len(sound_data) > 0 and len(sound_data[0]) > 1:
                        break
                except:
                    pass
                time.sleep(0.5)
                retry_count += 1
            
            if retry_count >= max_retries:
                self.get_logger().error("Timeout inizializzazione SoundDetected")
                self.sound_detect_service.unsubscribe("Audio Detection")
                self._enable_localization_async()
                self.is_recording = False
                return ""
            
            # Avvio registrazione microfoni
            self.audio_recorder.startMicrophonesRecording(output_file_robot, audio_format, sample_rate, channels)
            self.get_logger().info("Registrazione avviata")
            
            # Attesa silenzio (4s)
            is_sound_detected = True
            last_sound_time = time.time()
            
            while is_sound_detected:
                time.sleep(0.5)
                try:
                    if self.memory.getData("SoundDetected")[0][1] == 1:
                        last_sound_time = time.time()
                    else:
                        silence_duration = time.time() - last_sound_time
                        if silence_duration > 4.0:
                            is_sound_detected = False
                            self.get_logger().info("Silenzio rilevato, stop registrazione")
                except:
                    pass
            
            # Stop registrazione
            self.stop_recording()
            
            # FASE 4: Riattiva AudioLocalization in background
            self._enable_localization_async()
            
            # Trasferimento file via SFTP
            self.get_logger().info("Trasferimento file audio...")
            path_ros_ws = os.path.join(os.path.abspath(__file__).split("/install")[0])
            local_output_file = os.path.join(path_ros_ws, "src/audio/recording.wav")
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.ip, username='nao', password='nao')
            
            sftp = ssh.open_sftp()
            sftp.get(output_file_robot, local_output_file)
            sftp.close()
            ssh.close()
            
            self.get_logger().info(f"File trasferito: {local_output_file}")
            
            self.is_recording = False
            return local_output_file
            
        except Exception as e:
            self.get_logger().error(f"Errore registrazione: {e}")
            self.stop_recording()
            self._enable_localization_async()
            return ""
        

    def _enable_localization_async(self):
        """Riattiva AudioLocalization in thread separato con controllo safety"""
        def _reactivate():
            time.sleep(0.5)
            
            # Safety check: non riattivare se registrazione in corso
            if self.is_recording:
                self.get_logger().debug("Riattivazione AudioLocalization saltata (registrazione in corso)")
                return
            
            if self.memory is not None:
                try:
                    self.memory.subscribeToEvent(
                        "ALAudioSourceLocalization/SoundLocated",
                        self.get_name(),
                        self.on_sound_located
                    )
                    self.get_logger().debug("AudioLocalization riattivato")
                except Exception as e:
                    self.get_logger().warn(f"Errore riattivazione AudioLocalization: {e}")
        
        threading.Thread(target=_reactivate, daemon=True).start()



    def transcribe_audio(self, audio_path: str) -> str:
        """
        Trascrivi file audio con Whisper.
        
        Args:
            audio_path: Path locale file audio WAV
        
        Returns:
            str: Testo trascritto o stringa vuota se errore
        """
        if self.local_whisper is None:
            transcription = self.mock_transcriptions[self.mock_counter % len(self.mock_transcriptions)]
            self.mock_counter += 1
            self.get_logger().info(f"[MOCK] Trascrizione simulata: {transcription}")
            return transcription
        
        try:
            self.get_logger().info("Trascrizione con Whisper in corso...")
            result = self.local_whisper.transcribe(audio_path, language="it")
            transcription = result['text'].strip()
            self.get_logger().info(f"Trascrizione completata: {transcription}")
            return transcription
        except Exception as e:
            self.get_logger().error(f"Errore trascrizione Whisper: {e}")
            return ""


    def stop_recording(self):
        """Stop registrazione e cleanup"""
        try:
            if self.audio_recorder:
                self.audio_recorder.stopMicrophonesRecording()
            if self.sound_detect_service:
                self.sound_detect_service.unsubscribe("Audio Detection")
        except:
            pass
        
        self.is_recording = False


    # ========================================
    # AUDIO LOCALIZATION
    # ========================================
    
    def on_sound_located(self, event_name, value, subscriber_identifier):
        """Callback evento suono localizzato da ALAudioSourceLocalization"""
        try:
            sound_data = value[0]
            azimuth = sound_data[0]
            elevation = sound_data[1]
            confidence = sound_data[2]
            
            self.get_logger().info(
                f"Suono rilevato: azimuth={math.degrees(azimuth):.1f} gradi, "
                f"elevation={math.degrees(elevation):.1f} gradi, confidence={confidence:.2f}"
            )
            
            self.publish_sound_location(azimuth, elevation, confidence)
            
        except Exception as e:
            self.get_logger().error(f"Errore callback audio: {e}")


    def publish_sound_location(self, azimuth, elevation, confidence):
        """Pubblicazione posizione suono su topic"""
        
        distance = 2.0
        
        x = distance * math.cos(elevation) * math.cos(azimuth)
        y = distance * math.cos(elevation) * math.sin(azimuth)
        z = distance * math.sin(elevation)
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.sound_location_pub.publish(msg)
        
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
        
        if self.last_sound_location is None:
            response.success = False
            response.message = "Nessun suono rilevato"
            return response
        
        if self.last_sound_location["confidence"] < 0.5:
            response.success = False
            response.message = f"Confidence troppo bassa: {self.last_sound_location['confidence']:.2f}"
            return response
        
        target_x = self.last_sound_location["x"]
        target_y = self.last_sound_location["y"]
        
        distance = math.sqrt(target_x**2 + target_y**2)
        if distance > approach_distance:
            scale = (distance - approach_distance) / distance
            target_x *= scale
            target_y *= scale
        else:
            response.success = False
            response.message = f"Suono troppo vicino ({distance:.2f}m)"
            return response
        
        self.get_logger().info(f"Navigazione verso suono: target=({target_x:.2f}, {target_y:.2f})")
        
        if not self.navigating_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Navigating action server non disponibile"
            return response
        
        goal = Navigating.Goal()
        goal.target_x = target_x
        goal.target_y = target_y
        goal.use_map = False
        
        future = self.navigating_client.send_goal_async(goal)
        
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
    # LEGACY CALLBACKS
    # ========================================
    

    def stt_callback(self, msg):
        """
        Callback legacy per trascrizione audio manuale.
        Mantiene compatibilità con vecchio sistema basato su topic.
        """
        audio_path = msg.data
        self.get_logger().info(f"Path audio ricevuto (legacy callback): {audio_path}")
        
        transcription = self.transcribe_audio(audio_path)
        
        if transcription:
            self.get_logger().info(f"\n{'='*50}\nTRASCRIZIONE: {transcription}\n{'='*50}")


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

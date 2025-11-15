import qi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time
import os
import paramiko
import threading
from qi_unipa_2.utils import Utils


class QiUnipa2_audio(Node):
    def __init__(self):
        super().__init__("qi_unipa_2_audio")

        # Parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        
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
        
        # Publisher per audio registrato (path file)
        self.audio_recorded_pub = self.create_publisher(String, '/pepper/topics/audio_recorded', 10)
        
        # Publisher per trascrizione (testo per LLM)
        self.transcription_pub = self.create_publisher(String, '/pepper/topics/transcription', 10)
        
        # Subscription per stato talking (auto-recording)
        self.is_pepper_talking = False
        self.is_recording = False
        self.create_subscription(Bool, '/pepper/topics/is_talking', self.on_talking_changed, 10)
        
        # Connessione a Pepper per recording
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.memory = None
            self.audio_recorder = None
            self.sound_detect_service = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                self.memory = self.session.service("ALMemory")
                self.audio_recorder = self.session.service("ALAudioRecorder")
                self.sound_detect_service = self.session.service("ALSoundDetection")
                
                # Configurazione sensibilità sound detection per recording
                self.sound_detect_service.setParameter("Sensitivity", 0.8)
                
                self.get_logger().info("Servizi audio pronti")
                
            except Exception as e:
                self.get_logger().error(f"Errore connessione Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.memory = None
                self.audio_recorder = None
                self.sound_detect_service = None
        
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
        Registra audio da microfoni Pepper.
        Usa SoundDetection per rilevare inizio voce e silenzio finale.
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
            output_file_robot = "/home/nao/audio_record_unipa/risposta_utente.wav"
            
            # Attiva SoundDetection
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
                self.stop_recording()
                self.is_recording = False
                return ""
            
            # Avvio registrazione microfoni
            self.audio_recorder.startMicrophonesRecording(
                output_file_robot, audio_format, sample_rate, channels
            )
            self.get_logger().info("Registrazione avviata, in attesa di voce...")
            
            # Attesa voce + silenzio (4s)
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
            
            # Trasferimento file via SFTP
            self.get_logger().info("Trasferimento file audio...")
            #path_ros_ws = os.path.join(os.path.abspath(__file__).split("/install")[0])
            local_output_file = "/home/daniele/Scrivania/ros2_ws/audio_rec/risposta_utente.wav" #os.path.join(path_ros_ws, "/home/daniele/Scrivania/ros2_ws/risposta_utente.wav")
            
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
            self.is_recording = False
            return ""


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
            self.get_logger().info(f"Whisper non disponibile=> [MOCK] Trascrizione simulata: {transcription}")
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


    def destroy_node(self):
        """Cleanup prima distruzione nodo"""
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

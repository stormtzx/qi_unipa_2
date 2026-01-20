import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import Bool, String
import time
import os
import tempfile
import paramiko
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.action import Listening
import sounddevice as sd #type: ignore
import scipy.io.wavfile as wavfile


class QiUnipa2_audio(Node):
    def __init__(self):
        super().__init__("qi_unipa_2_audio")

        # Parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('mock_audio_transcription', True)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        self.declare_parameter('openai_api_key', '')

        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        mock_audio_transcription = self.get_parameter('mock_audio_transcription').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        openai_api_key = self.get_parameter('openai_api_key').get_parameter_value().string_value

        self.ip = ip
        self.mock_mode = mock_mode
        self.openai_client = None
        
        # Sorgenti audio valide
        self.VALID_AUDIO_SOURCES = ["pepper", "pc", "external"]
        self.DEFAULT_AUDIO_SOURCE = "pc"


        # CARICA WHISPER Se è la modalità reale oppure se è nella modalità mock ma con mock_audio_transcription=True
        if not mock_mode or mock_audio_transcription:
            try:
                from openai import OpenAI

                api_key = openai_api_key if openai_api_key else os.getenv('OPENAI_API_KEY')

                if not api_key:
                    self.get_logger().error("ERRORE CRITICO: OpenAI API key non trovata!")
                    raise ValueError("OpenAI API key non trovata")

                self.openai_client = OpenAI(api_key=api_key)
                
                if mock_mode:
                    self.get_logger().warn("OpenAI Whisper API caricato (MOCK MODE CON TRASCRIZIONE ATTIVA)")
                else:
                    self.get_logger().info("OpenAI Whisper API caricato con successo")
                    
            except Exception as e:
                self.get_logger().error(f"ERRORE CRITICO caricamento Whisper: {e}")
                raise
        else:
            self.get_logger().warn("Audio transcription DISABILITATA (MOCK MODE senza trascrizione)")


        # Publisher per trascrizione
        self.transcription_pub = self.create_publisher(String, '/pepper/topics/transcription', 10)

        # Action Server SINCRONO
        self._action_server = ActionServer(
            self,
            Listening,
            '/pepper/actions/listening',
            execute_callback=self.listening_execute_callback,
            goal_callback=self.listening_goal_callback,
            cancel_callback=self.listening_cancel_callback
        )

        # Subscription
        self.is_pepper_talking = False
        self.is_recording = False
        self.create_subscription(Bool, '/pepper/topics/is_talking', self.on_talking_changed, 10)

        # Connessione a Pepper
        if mock_mode:
            if mock_audio_transcription:
                self.get_logger().warn(f"Nodo AUDIO attivo in MOCK MODE CON TRASCRIZIONE ATTIVA (default audio source: {self.DEFAULT_AUDIO_SOURCE})")
            else:
                self.get_logger().warn(f"Nodo AUDIO attivo in MOCK MODE")
            self.session = None
            self.memory = None
            self.audio_recorder = None
            self.sound_detect_service = None
        else:
            try:
                utils = Utils(ip=ip, port=port, mock_mode=False)
                self.get_logger().info(f"Connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                self.memory = self.session.service("ALMemory")
                self.audio_recorder = self.session.service("ALAudioRecorder")
                self.sound_detect_service = self.session.service("ALSoundDetection")
                self.sound_detect_service.setParameter("Sensitivity", 0.85)
                self.get_logger().info("Servizi audio Pepper pronti")
                self.get_logger().info(f"Nodo AUDIO attivo e connesso a Pepper! (default audio source: {self.DEFAULT_AUDIO_SOURCE})")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e



    def on_talking_changed(self, msg):
        self.get_logger().info(f"Stato talking: {msg.data}")
        self.is_pepper_talking = msg.data

    # ============================================================================
    # ACTION SERVER CALLBACKS
    # ============================================================================

    def listening_goal_callback(self, goal_request):
        self.get_logger().info("Goal listening ricevuto")
        return GoalResponse.ACCEPT

    def listening_cancel_callback(self, goal_handle):
        self.get_logger().info("Cancellazione listening richiesta")
        return CancelResponse.ACCEPT

    def listening_execute_callback(self, goal_handle):
        """SINCRONO: Esegue registrazione e trascrizione"""
        self.get_logger().warn("[LISTENING START] Goal ricevuto")
        self.get_logger().warn(f"   Audio source: {goal_handle.request.audio_source}")
        
        try:
            audio_source = goal_handle.request.audio_source or self.DEFAULT_AUDIO_SOURCE
            self.get_logger().warn(f"[STEP 1] Sorgente: {audio_source}")
            
            # STEP 1: Registrazione SINCRONA
            local_audio_file = self.record_audio_from_source(audio_source)
            self.get_logger().warn(f"[STEP 2] File: {local_audio_file}")
            
            if not local_audio_file or not os.path.exists(local_audio_file):
                self.get_logger().error(f"File non esiste: {local_audio_file}")
                result = Listening.Result()
                result.success = False
                result.transcription = ""
                result.error_message = "Registrazione fallita"
                goal_handle.abort()
                return result
            
            file_size = os.path.getsize(local_audio_file)
            self.get_logger().warn(f"[STEP 2 OK] Size: {file_size} bytes")
            
            # STEP 2: Trascrizione SINCRONA
            transcription = self.transcribe_audio_file(local_audio_file)
            self.get_logger().warn(f"[STEP 3] Trascrizione: '{transcription}'")
            
            if not transcription or transcription.strip() == "":
                self.get_logger().error(f"Trascrizione vuota!")
                result = Listening.Result()
                result.success = False
                result.transcription = ""
                result.error_message = "Trascrizione vuota"
                goal_handle.abort()
                return result
            
            # STEP 3: Pubblica e ritorna
            result = Listening.Result()
            result.success = True
            result.transcription = transcription
            result.error_message = ""
            
            msg = String()
            msg.data = transcription
            self.transcription_pub.publish(msg)
            self.get_logger().warn(f"[PUBLISHED] {transcription}")
            
            goal_handle.succeed()
            self.get_logger().warn("[LISTENING COMPLETE]")
            return result
            
        except Exception as e:
            self.get_logger().error(f"EXCEPTION: {type(e).__name__}: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            result = Listening.Result()
            result.success = False
            result.transcription = ""
            result.error_message = str(e)
            goal_handle.abort()
            return result

    # ============================================================================
    # REGISTRAZIONE AUDIO (SINCRONA)
    # ============================================================================

    def record_audio_from_source(self, audio_source: str) -> str:
        if audio_source == "pc" or audio_source == "external":
            return self.record_local_microphone()
        elif audio_source == "pepper":
            return self.record_from_pepper()
        else:
            self.get_logger().error(f"Sorgente sconosciuta: {audio_source}")
            return None

    def record_local_microphone(self, seconds=7, fs=16000) -> str:
        """Registra dal microfono locale"""
        if self.mock_mode:
            self.get_logger().warn("[MOCK] Registrazione PC simulata")
            time.sleep(2)
            mock_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
            mock_file.write(b"mock_audio_data_pc")
            mock_file.close()
            return mock_file.name
        
        self.get_logger().info(f"Registrazione PC per {seconds}s...")
        
        try:
            # SINCRONO: Blocca qui finché non finisce
            audio = sd.rec(int(seconds * fs), samplerate=fs, channels=1, dtype='int16')
            sd.wait()  # BLOCCA
            
            tmpfile = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
            wavfile.write(tmpfile.name, fs, audio)
            
            file_size = os.path.getsize(tmpfile.name)
            self.get_logger().info(f"Registrazione PC completata: {file_size} bytes")
            return tmpfile.name
        
        except Exception as e:
            self.get_logger().error(f"Errore registrazione PC: {e}")
            return None

    def record_from_pepper(self) -> str:
        """Registra dal microfono di Pepper"""
        if self.mock_mode:
            self.get_logger().warn("[MOCK] Registrazione Pepper simulata")
            time.sleep(2)
            mock_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
            mock_file.write(b"mock_audio_data_pepper")
            mock_file.close()
            return mock_file.name

        if not self.audio_recorder or not self.sound_detect_service or not self.memory:
            self.get_logger().error("Servizi Pepper non disponibili. Usa audio_source='pc'")
            return None

        if self.is_recording:
            self.get_logger().warn("Registrazione già in corso")
            return None

        self.is_recording = True

        try:
            channels = [1, 1, 1, 1]
            audio_format = "wav"
            sample_rate = 16000
            output_file_robot = "/tmp/risposta_utente.wav"

            self.get_logger().info(f"Registrazione Pepper: {output_file_robot}")

            try:
                self.audio_recorder.stopMicrophonesRecording()
            except:
                pass

            self.sound_detect_service.subscribe("Audio Detection")

            # Inizializzazione SoundDetected
            max_retries = 10
            retry_count = 0

            while retry_count < max_retries:
                try:
                    sound_data = self.memory.getData("SoundDetected")
                    if sound_data and len(sound_data) > 0 and len(sound_data[0]) > 1:
                        self.get_logger().info("SoundDetected OK")
                        break
                except:
                    pass
                time.sleep(0.5)
                retry_count += 1

            if retry_count >= max_retries:
                self.get_logger().error("Timeout SoundDetected")
                self.stop_recording()
                return None

            self.audio_recorder.startMicrophonesRecording(
                output_file_robot, audio_format, sample_rate, channels
            )
            self.get_logger().info("Registrazione Pepper avviata...")

            # Loop SINCRONO
            is_sound_detected = True
            last_sound_time = time.time()
            recording_start = time.time()
            max_recording_time = 7.0

            while is_sound_detected:
                time.sleep(0.5)  # BLOCCA

                elapsed = time.time() - recording_start
                if elapsed > max_recording_time:
                    self.get_logger().info(f"Timeout registrazione")
                    break

                try:
                    sound_status = self.memory.getData("SoundDetected")
                    if sound_status and len(sound_status) > 0:
                        last_sound_time = time.time()
                    else:
                        silence_duration = time.time() - last_sound_time
                        if silence_duration > 4.0:
                            is_sound_detected = False
                            self.get_logger().info(f"Silenzio rilevato")
                except:
                    pass

            self.stop_recording()
            time.sleep(0.5)

            # Download SFTP
            self.get_logger().info("Download da Pepper...")

            try:
                local_temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
                local_temp_file.close()
                local_path = local_temp_file.name

                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(self.ip, username='nao', password='nao', timeout=10)

                sftp = ssh.open_sftp()
                sftp.get(output_file_robot, local_path)
                sftp.remove(output_file_robot)
                sftp.close()
                ssh.close()

                file_size = os.path.getsize(local_path)
                self.get_logger().info(f"Download Pepper OK: {file_size} bytes")

                return local_path

            except Exception as e:
                self.get_logger().error(f"Errore SFTP: {e}")
                return None

        except Exception as e:
            self.get_logger().error(f"Errore Pepper: {e}")
            return None

        finally:
            self.is_recording = False

    # ============================================================================
    # TRASCRIZIONE (SINCRONA)
    # ============================================================================

    def transcribe_audio_file(self, file_path):
        """ SINCRONO: Trascrive con Whisper"""
        self.get_logger().info(f"🎤 [TRANSCRIBE] Inizio: {file_path}")
        
        if self.openai_client is None:
            self.get_logger().error("OpenAI client è None!")
            return ""
        
        try:
            with open(file_path, 'rb') as audio_file:
                file_size = os.path.getsize(file_path)
                self.get_logger().info(f"🎤 [TRANSCRIBE] Size: {file_size} bytes")
                
                # SINCRONO: 
                response = self.openai_client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    language="it"  # Italiano
                )
                
                transcription = response.text
                self.get_logger().info(f"🎤 [TRANSCRIBE OK] {transcription}")
                return transcription
                
        except Exception as e:
            self.get_logger().error(f"TRANSCRIBE ERROR: {type(e).__name__}: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return ""

    def stop_recording(self):
        try:
            if self.audio_recorder:
                self.audio_recorder.stopMicrophonesRecording()
            if self.sound_detect_service:
                self.sound_detect_service.unsubscribe("Audio Detection")
        except:
            pass
        self.is_recording = False

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QiUnipa2_audio()
        rclpy.spin(node)
    except ValueError as e:
        print(f"ERRORE CRITICO: {e}")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    exit(main())

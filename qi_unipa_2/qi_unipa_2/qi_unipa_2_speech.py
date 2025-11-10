import qi
import sys
import asyncio
import rclpy
import time
import os
import paramiko
from rclpy.node import Node
from std_msgs.msg import Bool, String
from qi_unipa_2_interfaces.msg import Tracker
from qi_unipa_2_interfaces.srv import SetPosture
from qi_unipa_2_interfaces.action import Talking, Browsing
from rclpy.action import ActionClient
import json
from qi_unipa_2.utils import Utils
from rclpy.action import ActionServer


class QiUnipa2_speech(Node):  
    def __init__(self):
        super().__init__('qi_unipa_2_speech')

        # Dichiarazione parametri con valori di default
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        # Lettura parametri
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Crea Utils con parametri
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        
        # Salva ip come attributo
        self.ip = ip

        # Connessione condizionale
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.memory = None
            self.animated_speech = None
            self.audio_service = None
            self.sound_detect_service = None
            self.configuration = None
            self.leds_service = None
            self.tg = None
            self.s1 = None
            self.last_index = 0
            self.is_recognizing = False
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                # ALService
                self.memory = self.session.service("ALMemory")
                self.animated_speech = self.session.service("ALAnimatedSpeech")
                self.audio_service = self.session.service("ALAudioRecorder")
                self.sound_detect_service = self.session.service("ALSoundDetection")
                self.sound_detect_service.setParameter("Sensitivity", 0.8)
                self.configuration = {"bodyLanguageMode": "contextual"}
                self.leds_service = self.session.service("ALLeds")
                self.tg = self.session.service('ALTactileGesture')
                self.s1 = self.tg.onGesture.connect(self.onTouched)
                
                # Variabili
                self.last_index = self.memory.getData("ALTextToSpeech/Status")[0]
                self.is_recognizing = False
                        
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.memory = None
                self.animated_speech = None
                self.audio_service = None
                self.sound_detect_service = None
                self.configuration = None
                self.leds_service = None
                self.tg = None
                self.s1 = None
                self.last_index = 0
                self.is_recognizing = False

        # Service
        self.set_posture_client = self.create_client(SetPosture, '~/set_posture')

        # Action
        self._action_server_talking = ActionServer(self, Talking, '~/talking', self.talking)
        self.browsing_client = ActionClient(self, Browsing, 'browsing')


        # Topic subscription
        self.record_sub = self.create_subscription(Bool, "~/record", self.record_callback, qos_reliable_10)
        self.record_no_mic = self.create_subscription(Bool, "~/record_no_mic", self.record_callback_no_mic, qos_reliable_10)
        self.end_sub = self.create_subscription(Bool, "~/end", self.ending_callback, qos_reliable_10)
        self.start_bdi_sub = self.create_subscription(Bool, "~/start_bdi", self.start_bdi_callback, qos_reliable_10)
        
        # Topic publisher
        self.tracking_pub = self.create_publisher(Tracker, "~/tracker",  qos_reliable_10)
        self.isTalking_pub = self.create_publisher(Bool, '~/is_speaking',  qos_reliable_10)
        self.isTalking_bdi_pub = self.create_publisher(Bool, '~/is_speaking_bdi', qos_reliable_10)
        self.start_pub = self.create_publisher(Bool, '~/start_conv', qos_reliable_10)
        self.show_pub = self.create_publisher(String, "~/show", qos_reliable_10)
        self.stt_pub = self.create_publisher(String, "~/stt", qos_reliable_10)
        self.stt_bdi_pub = self.create_publisher(String, "~/stt_bdi", qos_reliable_10)
        self.touched = self.create_publisher(Bool, "~/touched", qos_reliable_10)

        self.agente = "agente"      
        self.percept = "percept"
        self.start = False
        self.first = False
        self.tablet_on = True
        self.start_bdi = False
        self.create_timer(0.5, self.check_speaking)
        self.set_led(False)



    def call_set_posture_sync(self, posture_name, speed):#Permette chiamata sincrona ad una funzione async
        """
        Wrapper sincrono per chiamare set_posture senza await.
        Fire-and-forget: non aspetta la risposta.
        """
        if not self.set_posture_client.service_is_ready():
            self.get_logger().warn(f"[MOCK/Service non pronto] set_posture '{posture_name}' ignorato")
            return
        
        request = SetPosture.Request()
        request.posture_name = posture_name
        request.speed = speed
        self.set_posture_client.call_async(request)
        self.get_logger().debug(f"Chiamata set_posture: {posture_name} @ speed {speed}")

    
    def send_browsing(self, html_page: str, use_tablet: bool = False):
        if not self.browsing_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server /browsing non disponibile")
            return
        
        goal_msg = Browsing.Goal()
        goal_msg.html_page = html_page
        goal_msg.use_tablet = use_tablet
        
        self.get_logger().info(f"Invio action Browsing: page='{html_page}', use_tablet={use_tablet}")
        send_goal_future = self.browsing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.browsing_goal_response_callback)
    
    def browsing_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Action Browsing rifiutata")
            return
        self.get_logger().info("Action Browsing accettata")

    # modificata ending_callback che prima usava show_pub
    def ending_callback(self, msg):
        self.pub_tracker("Stop", 0.3)
        self.call_set_posture_sync("Stand", 0.5)
        self.tablet_on = True
        self.start = False
        self.set_led(False)
        # Invio action Browsing invece di show topic
        self.send_browsing("disattivo.html", use_tablet=False)

    # modifica di onTouched che prima usava show_pub
    def onTouched(self, value):
        self.get_logger().info(f"Tocco: {value}")
        if not self.start:
            self.start = True
            msg = Bool()
            if self.first:
                msg.data = True
            else:
                msg.data = False
                self.first = True
            self.start_pub.publish(msg)
            self.get_logger().info("Start")
        else:
            self.start = False
            self.set_led(False)
            self.call_set_posture_sync("Stand", 0.5)
            self.get_logger().info("Ending")

        if self.tablet_on:
            self.pub_tracker("Face", 0.8)
            self.send_browsing("attivo.html", use_tablet=False)
            self.tablet_on = False

            msg3 = Bool()
            msg3.data = True
            self.touched.publish(msg3)
        else:
            self.pub_tracker("Stop", 0.3)
            self.send_browsing("disattivo.html", use_tablet=False)
            self.tablet_on = True

            msg3 = Bool()
            msg3.data = False
            self.touched.publish(msg3)



    def pub_tracker(self, name, distance):
        msg = Tracker()
        msg.target_name = name
        msg.distance = distance
        self.tracking_pub.publish(msg)

    def is_valid_json(self, stringa):
        try:
            json.loads(stringa)
            return True
        except (json.JSONDecodeError, TypeError):
            return False


    def talking(self, goal_handle):
        message = goal_handle.request.message
        self.agente = goal_handle.request.agente
        self.percept = goal_handle.request.percept
        
        is_talking_feedback = Talking.Feedback()
        is_talking_feedback.is_talking = True
        goal_handle.publish_feedback(is_talking_feedback)
        
        is_talking = Bool()
        is_talking.data = True
        self.isTalking_pub.publish(is_talking)

        if self.animated_speech is None:
            self.get_logger().warn(f"[MOCK] Talking simulato: {message}")
            time.sleep(1)
        else:
            try:
                if self.is_valid_json(message):
                    self.animated_speech.say(message, self.configuration)
                else:
                    self.set_fade_led(False)
                    self.animated_speech.say(message, self.configuration)
            except Exception as e:
                self.get_logger().error(f"Errore talking: {e}")

        goal_handle.succeed()
        is_talking_feedback.is_talking = False
        goal_handle.publish_feedback(is_talking_feedback)
        
        result = Talking.Result()
        result.talking_complete = True
        
        is_talking.data = False
        self.isTalking_pub.publish(is_talking)
        self.isTalking_bdi_pub.publish(is_talking)
        self.get_logger().info("Pepper talking complete")

        self.call_set_posture_sync("Stand", 0.5) #Permette chiamata sincrona ad una funzione async
        
        return result

    def check_speaking(self):
        if self.memory is None:
            return
        try:
            msg = Bool()
            status = self.memory.getData("ALTextToSpeech/Status")
            if self.start:  
                if status[1] == "done" and status[0] != self.last_index:
                    msg.data = False
                    self.isTalking_pub.publish(msg)
                    self.get_logger().info("Pepper ha terminato")
                else:
                    msg.data = True
                    self.isTalking_pub.publish(msg)
                self.last_index = status[0]
            elif self.start_bdi:
                if status[1] == "done" and status[0] != self.last_index:
                    msg.data = False
                    self.isTalking_bdi_pub.publish(msg)
                    self.get_logger().info("BDI--------------Pepper ha terminato")
                else:
                    msg.data = True
                    self.isTalking_bdi_pub.publish(msg)
                self.last_index = status[0]
        except Exception as e:
            self.get_logger().error(f"Errore check_speaking: {e}")


    def record_callback(self, msg):
        if self.audio_service is None or self.sound_detect_service is None or self.memory is None:
            self.get_logger().warn("[MOCK] record_callback simulato")
            res = String()
            res.data = "/mock/path/recording.wav"
            if self.start:
                self.stt_pub.publish(res)
            return

        if getattr(self, "is_recording", False):
            self.get_logger().warn("Registrazione già in corso, callback ignorato.")
            return
        
        self.is_recording = True
        
        try:
            try:
                self.audio_service.stopMicrophonesRecording()
                self.get_logger().info("Registrazione precedente interrotta.")
            except Exception:
                self.get_logger().warn("Nessuna registrazione attiva da fermare.")
            
            channels = [1, 1, 1, 1]
            audio_format = "wav"
            sample_rate = 16000
            output_file_robot = "/home/nao/audio_record_unipa/recording.wav"
            
            self.sound_detect_service.subscribe("Audio Detection")
            
            max_retries = 10
            retry_count = 0
            sound_data = None
            
            while retry_count < max_retries:
                try:
                    sound_data = self.memory.getData("SoundDetected")
                    if sound_data is not None and len(sound_data) > 0 and len(sound_data[0]) > 1:
                        self.get_logger().info("Servizio SoundDetected inizializzato correttamente.")
                        break
                    else:
                        self.get_logger().info(f"Attesa inizializzazione SoundDetected... tentativo {retry_count + 1}")
                        time.sleep(0.5)
                        retry_count += 1
                except Exception as e:
                    self.get_logger().warn(f"Errore durante l'accesso a SoundDetected: {e}")
                    time.sleep(0.5)
                    retry_count += 1
            
            if sound_data is None:
                self.get_logger().error("Impossibile inizializzare il servizio SoundDetected. Annullamento registrazione.")
                self.sound_detect_service.unsubscribe("Audio Detection")
                self.is_recording = False
                return
            
            self.audio_service.startMicrophonesRecording(output_file_robot, audio_format, sample_rate, channels)
            self.get_logger().info("Avvio microfoni")
            self.set_led(True)
            time.sleep(0.3)

            self.is_recognizing = False
            while not self.is_recognizing:
                time.sleep(0.3)
                if self.memory.getData("SoundDetected")[0][1] == 1:
                    self.get_logger().info("Avvio registrazione...")        
                    self.is_recognizing = True
                    
            while self.is_recognizing:
                time.sleep(4)
                if self.memory.getData("SoundDetected")[0][1] == 0:
                    self.is_recognizing = False
                    self.audio_service.stopMicrophonesRecording()
                    self.set_led(False)
                    self.sound_detect_service.unsubscribe("Audio Detection")
                    self.get_logger().info(f"Registrazione terminata e salvata in: {output_file_robot}")

            path_ros_ws = os.path.join(os.path.abspath(__file__).split("/install")[0])
            local_output_file = os.path.join(path_ros_ws, "src/audio/recording.wav")
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.ip, username='nao', password='nao')

            sftp = ssh.open_sftp()
            sftp.get(output_file_robot, local_output_file)
            sftp.close()
            ssh.close()
            self.get_logger().info("File trasferito con successo!")
            
            res = String()
            res.data = local_output_file
            self.set_fade_led(True)
            
            if self.start:
                self.stt_pub.publish(res)
        except Exception as e:
            self.get_logger().error(f"Errore record_callback: {e}")
        finally:
            self.is_recording = False

    
    def start_bdi_callback(self, msg):
        self.start_bdi = msg.data

    def record_callback_no_mic(self, msg):
        res_bdi = String()
        messaggio_json = {
            "agente": self.agente,
            "percept": self.percept
        }
        res_bdi.data = json.dumps(messaggio_json)
        self.stt_bdi_pub.publish(res_bdi)

    def set_led(self, on):
        if self.leds_service is None:
            return
        
        try:
            names = [
                "Face/Led/Green/Left/0Deg/Actuator/Value",
                "Face/Led/Green/Left/45Deg/Actuator/Value",
                "Face/Led/Green/Left/90Deg/Actuator/Value",
                "Face/Led/Green/Left/135Deg/Actuator/Value",
                "Face/Led/Green/Left/180Deg/Actuator/Value",
                "Face/Led/Green/Left/225Deg/Actuator/Value",
                "Face/Led/Green/Left/270Deg/Actuator/Value",
                "Face/Led/Green/Left/315Deg/Actuator/Value",
                "Face/Led/Green/Right/0Deg/Actuator/Value",
                "Face/Led/Green/Right/45Deg/Actuator/Value",
                "Face/Led/Green/Right/90Deg/Actuator/Value",
                "Face/Led/Green/Right/135Deg/Actuator/Value",
                "Face/Led/Green/Right/180Deg/Actuator/Value",
                "Face/Led/Green/Right/225Deg/Actuator/Value",
                "Face/Led/Green/Right/270Deg/Actuator/Value",
                "Face/Led/Green/Right/315Deg/Actuator/Value"
            ]

            self.leds_service.createGroup("eyes", names)
            
            if on == True:
                self.leds_service.off("FaceLeds")
                self.leds_service.on("eyes")
            elif on == False:
                self.leds_service.off("eyes")
                self.leds_service.on("FaceLeds")
        except Exception as e:
            self.get_logger().error(f"Errore set_led: {e}")

    def set_fade_led(self, on):
        if self.leds_service is None:
            return
        
        try:
            names = [
                "Face/Led/Green/Left/0Deg/Actuator/Value",
                "Face/Led/Green/Left/45Deg/Actuator/Value",
                "Face/Led/Green/Left/90Deg/Actuator/Value",
                "Face/Led/Green/Left/135Deg/Actuator/Value",
                "Face/Led/Green/Left/180Deg/Actuator/Value",
                "Face/Led/Green/Left/225Deg/Actuator/Value",
                "Face/Led/Green/Left/270Deg/Actuator/Value",
                "Face/Led/Green/Left/315Deg/Actuator/Value",
                "Face/Led/Green/Right/0Deg/Actuator/Value",
                "Face/Led/Green/Right/45Deg/Actuator/Value",
                "Face/Led/Green/Right/90Deg/Actuator/Value",
                "Face/Led/Green/Right/135Deg/Actuator/Value",
                "Face/Led/Green/Right/180Deg/Actuator/Value",
                "Face/Led/Green/Right/225Deg/Actuator/Value",
                "Face/Led/Green/Right/270Deg/Actuator/Value",
                "Face/Led/Green/Right/315Deg/Actuator/Value"
            ]

            self.leds_service.createGroup("eyes", names)

            colors = [
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 1.0),
                (1.0, 0.0, 1.0),
                (1.0, 1.0, 1.0)
            ]

            duration = 0.8

            if on == True:
                self.leds_service.off("FaceLeds")
                for r, g, b in colors:
                    self.leds_service.fadeRGB("FaceLeds", r, g, b, duration)
            elif on == False:
                self.set_led(False)
        except Exception as e:
            self.get_logger().error(f"Errore set_fade_led: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_speech()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

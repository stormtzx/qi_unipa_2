import qi
import sys
import asyncio
import rclpy
import time
import os
import paramiko
from rclpy.node import Node
from std_msgs.msg import Bool, String
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
            self.get_logger().warn("Nodo SPEECH attivo in MOCK MODE")
            self.session = None
            self.memory = None
            self.animated_speech = None
            self.audio_service = None
            self.sound_detect_service = None
            self.configuration = None
            self.leds_service = None
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
                
                # Variabili
                self.is_recognizing = False                        
                self.get_logger().info("Nodo SPEECH attivo e connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e

        # Service
        self.set_posture_client = self.create_client(SetPosture, '/pepper/services/set_posture')


        # Action
        self._action_server_talking = ActionServer(self, Talking, '/pepper/actions/talking', self.talking)


        # Topic publisher
        self.isTalking_pub = self.create_publisher(Bool, '/pepper/topics/is_talking',  qos_reliable_10)


        self.set_led(False)


    
    # Sistema la postura prima di parlare
    def call_set_posture_sync(self, posture_name, speed):
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




    def talking(self, goal_handle):
        message = goal_handle.request.message
        self.agente = goal_handle.request.agente
        self.percept = goal_handle.request.percept
        
        is_talking_feedback = Talking.Feedback()
        is_talking_feedback.is_talking = True
        goal_handle.publish_feedback(is_talking_feedback)
        
        is_talking = Bool()
        is_talking.data = True
        self.isTalking_pub.publish(is_talking) #Il nodo audio rileverà inizio e fine parlato


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


        is_talking.data = False
        self.isTalking_pub.publish(is_talking) #Il nodo audio rileverà inizio e fine parlato
        
        goal_handle.succeed()
        is_talking_feedback.is_talking = False
        goal_handle.publish_feedback(is_talking_feedback)
        
        result = Talking.Result()
        result.talking_complete = True
        
        self.get_logger().info("Pepper talking complete")


        self.call_set_posture_sync("Stand", 0.5)
        
        return result
    


    # Valida la struttura del messaggio
    def is_valid_json(self, stringa):
        try:
            json.loads(stringa)
            return True
        except (json.JSONDecodeError, TypeError):
            return False





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

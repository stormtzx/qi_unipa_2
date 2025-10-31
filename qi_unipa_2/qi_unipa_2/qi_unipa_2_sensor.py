import qi
import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
from std_msgs.msg import String
from qi_unipa_2_interfaces.msg import Sonar, Bumper
from qi_unipa_2_interfaces.action import Talking
import time
from qi_unipa_2.utils import Utils


class QiUnipa2_sensor(Node):    
    def __init__(self):
        super().__init__('qi_unipa_2_sensor')
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        # Lettura parametri
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Crea Utils
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)


        # Connessione condizionale
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.sonar = None
            self.memory = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                # Connessione ai servizi di Pepper
                self.sonar = self.session.service("ALSonar")
                self.memory = self.session.service("ALMemory")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.sonar = None
                self.memory = None


        # Publishers
        self.sonar_pub = self.create_publisher(Sonar, "/sonar", qos_best_effort_10)
        self.bumper_pub = self.create_publisher(Bumper, "/bumper", qos_best_effort_10)
        self.speak_pub = self.create_publisher(String, "/speak", qos_best_effort_10)
        
        # Action Client per Talking
        self.talking_client = ActionClient(self, Talking, '/qi_unipa_2_speech/talking')
        
        self.pressed = False


        # Timers
        self.timer_sonar = self.create_timer(1.0, self.sonar_sub)
        self.timer_bumper = self.create_timer(0.3, self.bumper_sub)




    def sonar_sub(self):
        msg = Sonar()
        
        if self.sonar is None or self.memory is None:
            # MOCK: Pubblica dati simulati
            msg.front_sonar = 0.5
            msg.back_sonar = 0.5
            self.sonar_pub.publish(msg)
            return
        
        # Codice reale
        try:
            self.sonar.subscribe("Sonar_app")
            msg.front_sonar = self.memory.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")
            msg.back_sonar = self.memory.getData("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value")
            self.sonar_pub.publish(msg)
            
            # Unsubscribe se sottoscritto
            subscribers = self.sonar.getSubscribersInfo()
            if any("Sonar_app" in sub[0] for sub in subscribers):
                self.sonar.unsubscribe("Sonar_app")
        except Exception as e:
            self.get_logger().error(f"Errore sonar_sub: {e}")
        
    
    def bumper_sub(self):
        msg = Bumper()
        
        if self.memory is None:
            # MOCK: Pubblica dati simulati
            msg.left = 0.0
            msg.right = 0.0
            msg.back = 0.0
            self.bumper_pub.publish(msg)
            return
        
        # Codice reale
        try:
            msg.left = self.memory.getData("Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value")
            msg.right = self.memory.getData("Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value")
            msg.back = self.memory.getData("Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value")
            self.bumper_pub.publish(msg)


            if not self.pressed:
                # Determina il messaggio
                message = None
                if msg.left == 1.0:
                    message = "ho urtato a sinistra"
                elif msg.right == 1.0:
                    message = "ho urtato a destra"
                elif msg.back == 1.0:
                    message = "ho urtato dietro"
                
                # Se c'è un urto, invia action E pubblica
                if message:
                    # Pubblica su /speak (come prima)
                    string = String()
                    string.data = message
                    self.speak_pub.publish(string)
                    
                    # Invia action Talking
                    self.send_talking_action(message)
                    
                    self.pressed = True


            if msg.left == 0.0 and msg.right == 0.0 and msg.back == 0.0 and self.pressed:
                self.pressed = False
                
        except Exception as e:
            self.get_logger().error(f"Errore bumper_sub: {e}")


    def send_talking_action(self, text):
        """Invia action Talking per far parlare Pepper"""
        
        # Attende che l'action server sia disponibile (con timeout)
        if not self.talking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server /talking non disponibile")
            return
        
        # Crea goal
        goal_msg = Talking.Goal()
        goal_msg.message = text
        
        
        # Invia goal (async, non blocca il nodo)
        self.get_logger().info(f"Invio action Talking: '{text}'")
        send_goal_future = self.talking_client.send_goal_async(goal_msg)
        
        # Callback opzionale per verificare accettazione
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """Callback quando l'action viene accettato/rifiutato"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Action Talking rifiutato")
            return
        
        self.get_logger().info("Action Talking accettato")



def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_sensor()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

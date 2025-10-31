import qi
import rclpy
from rclpy.node import Node
from qi_unipa_2_interfaces.srv import SetPosture
from qi_unipa_2_interfaces.msg import Track
from std_msgs.msg import Bool, String
from qi_unipa_2.utils import Utils


class QiUnipa2_tracking(Node):  
    def __init__(self):
        super().__init__('qi_unipa_2_tracking')
        
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
            self.tracker = None
            self.motion = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                self.tracker = self.session.service("ALTracker")
                self.motion = self.session.service("ALMotion")
                self.tracker.setMode("Head")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.tracker = None
                self.motion = None

        # Subscription e client
        self.tracking_sub = self.create_subscription(Track, "/track", self.start_tracking, qos_profile=qos_best_effort_10)
        self.set_posture_client = self.create_client(SetPosture, '~/set_posture')

    
    #Permette chiamata sincrona ad una funzione async
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


    def start_tracking(self, msg_in):
        """Avvia il tracking del target specificato"""
        if self.tracker is None:
            self.get_logger().warn(f"[MOCK] Tracking simulato: {msg_in.target_name}")
            return
        
        try:
            # Chiama set_posture in modo sincrono
            self.call_set_posture_sync("Stand", 0.5) #Permette chiamata sincrona ad una funzione async
            
            # Prepara parametri in base al target
            if msg_in.target_name == "Face":
                params = msg_in.distance
            elif msg_in.target_name == "Sound":
                confidence = 0.5
                params = [msg_in.distance, confidence]
            elif msg_in.target_name == "Stop":
                self.stop_tracking()
                return
            elif msg_in.target_name == "People":
                params = [0]
            else:
                self.get_logger().warn(f"Received unknown target_name: {msg_in.target_name}")
                return
            
            # Avvia tracking reale
            self.tracker.registerTarget(msg_in.target_name, params)
            self.tracker.setRelativePosition([-2.0, 0.0, 0.0, 0.1, 0.1, 0.3])
            self.tracker.track(msg_in.target_name)
            
            self.get_logger().info(f"Tracking attivato per: {msg_in.target_name}")
        except Exception as e:
            self.get_logger().error(f"Errore start_tracking: {e}")


    def stop_tracking(self):
        """Ferma il tracking"""
        if self.tracker is None:
            self.get_logger().warn("[MOCK] Stop tracking simulato")
            return
        try:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()
            self.call_set_posture_sync("Stand", 0.5) #Permette chiamata sincrona ad una funzione async
            self.get_logger().info("Tracking fermato")
        except Exception as e:
            self.get_logger().error(f"Errore stop_tracking: {e}")


def main(args=None):
    rclpy.init(args=args) 
    node = QiUnipa2_tracking()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

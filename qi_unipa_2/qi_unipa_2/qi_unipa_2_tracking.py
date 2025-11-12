import qi
import rclpy
from rclpy.node import Node
from qi_unipa_2_interfaces.srv import SetPosture, GetTrackedObjCoordinates, MoveToTrackedObj
from qi_unipa_2_interfaces.msg import Tracker
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from qi_unipa_2.utils import Utils
from rclpy.action import ActionClient
from qi_unipa_2_interfaces.action import Navigating
import math
import time


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

        # Variabile per memorizzare target attivo
        self.active_target = None
        
        # Variabile per memorizzare ultima posizione oggetto tracciato (con timestamp)
        self.last_tracked_object = None

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

        # Publisher per coordinate oggetto tracciato (opzionale, continuo)
        self.tracked_coords_pub = self.create_publisher(PointStamped,'/pepper/topics/tracked_coordinates',10)
        
        # Timer per pubblicazione continua coordinate (10 Hz)
        self.create_timer(0.1, self.publish_tracked_coordinates)

        # Subscription topic tracker
        self.tracking_sub = self.create_subscription(Tracker,"/pepper/topics/tracker",self.start_tracking,qos_best_effort_10)
        
        # Action Client per Navigating
        self.navigating_client = ActionClient(self,Navigating,'/pepper/actions/navigating')
        
        # Service client SetPosture
        self.set_posture_client = self.create_client(SetPosture, '/pepper/services/set_posture')
        
        # Service GetTrackedObjCoordinates (query on-demand)
        self.get_coords_service = self.create_service(GetTrackedObjCoordinates,'/pepper/services/get_tracked_object_coordinates',
            self.get_tracked_object_coordinates)
        
        # Service MoveToTrackedObject (navigazione verso oggetto tracciato)
        self.move_to_tracked_service = self.create_service(MoveToTrackedObj,'/pepper/services/move_to_tracked_object',
            self.move_to_tracked_obj_callback)

    
    def call_set_posture_sync(self, posture_name, speed):
        """Wrapper sincrono per chiamare set_posture senza await"""
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
            self.active_target = msg_in.target_name
            return
        
        try:
            # Chiama set_posture in modo sincrono
            self.call_set_posture_sync("Stand", 0.5)
            
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
            
            # Memorizza target attivo
            self.active_target = msg_in.target_name
            
            self.get_logger().info(f"Tracking attivato per: {msg_in.target_name}")
        except Exception as e:
            self.get_logger().error(f"Errore start_tracking: {e}")


    def get_tracked_object_position(self):
        """
        Ottiene posizione 3D oggetto tracciato da ALTracker.
        Ritorna dict con coordinate o None se non disponibile.
        """
        if self.tracker is None or self.active_target is None:
            return None
        
        try:
            # Ottieni posizione target in frame TORSO (0)
            position = self.tracker.getTargetPosition(0)
            
            if not position or len(position) < 3:
                return None
            
            x, y, z = position[0], position[1], position[2]
            distance = math.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,
                "y": y,
                "z": z,
                "distance": distance,
                "frame": "TORSO",
                "target_name": self.active_target
            }
            
        except Exception as e:
            self.get_logger().debug(f"Errore get_tracked_object_position: {e}")
            return None


    def publish_tracked_coordinates(self):
        """
        Pubblicazione continua coordinate oggetto tracciato su topic (10 Hz).
        Aggiorna anche last_tracked_object.
        """
        if self.active_target is None:
            return
        
        position = self.get_tracked_object_position()
        
        if position is None:
            return
        
        # Aggiorna last_tracked_object con timestamp
        self.last_tracked_object = {
            "x": position["x"],
            "y": position["y"],
            "z": position["z"],
            "distance": position["distance"],
            "target_name": position["target_name"],
            "timestamp": self.get_clock().now()
        }
        
        # Crea messaggio PointStamped
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "torso"
        msg.point.x = position["x"]
        msg.point.y = position["y"]
        msg.point.z = position["z"]
        
        # Pubblica su topic
        self.tracked_coords_pub.publish(msg)
        
        self.get_logger().debug(
            f"Coordinate oggetto tracciato '{self.active_target}': "
            f"x={position['x']:.2f}, y={position['y']:.2f}, z={position['z']:.2f}, "
            f"dist={position['distance']:.2f}m"
        )


    def get_tracked_object_coordinates(self, request, response):
        """Service callback per ottenere coordinate oggetto tracciato on-demand"""
        
        if self.active_target is None:
            response.success = False
            response.message = "Nessun oggetto tracciato attivo"
            response.target_name = ""
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.distance = 0.0
            response.frame = ""
            return response
        
        # Mock mode
        if self.tracker is None:
            response.success = True
            response.message = f"[MOCK] Coordinate simulate per '{self.active_target}'"
            response.target_name = self.active_target
            response.x = 1.5
            response.y = 0.0
            response.z = 0.0
            response.distance = 1.5
            response.frame = "TORSO"
            return response
        
        # Ottieni posizione reale
        position = self.get_tracked_object_position()
        
        if position is None:
            response.success = False
            response.message = f"Impossibile ottenere posizione di '{self.active_target}'"
            response.target_name = self.active_target
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.distance = 0.0
            response.frame = ""
            return response
        
        # Successo
        response.success = True
        response.message = f"Coordinate oggetto '{self.active_target}' ottenute"
        response.target_name = self.active_target
        response.x = position["x"]
        response.y = position["y"]
        response.z = position["z"]
        response.distance = position["distance"]
        response.frame = position["frame"]
        
        self.get_logger().info(
            f"Service: Coordinate oggetto '{self.active_target}': "
            f"({position['x']:.2f}, {position['y']:.2f}, {position['z']:.2f}) "
            f"distanza={position['distance']:.2f}m"
        )
        
        return response


    def move_to_tracked_obj_callback(self, request, response):
        """Service callback per navigazione verso oggetto tracciato"""
        # LEttura distanza di avvicinamento (ossia quanto si deve fermare vicino all'oggetto (default 0.5 m)
        approach_distance = request.approach_distance if hasattr(request, 'approach_distance') else 0.5
        
        # Verifica presenza oggetto tracciato
        if self.last_tracked_object is None:
            response.success = False
            response.message = "Nessun oggetto tracciato rilevato"
            response.target_x = 0.0
            response.target_y = 0.0
            return response
        
        # Verifica recency (timeout 10 secondi)
        time_since_tracking = (self.get_clock().now() - self.last_tracked_object["timestamp"]).nanoseconds / 1e9
        if time_since_tracking > 10.0:
            response.success = False
            response.message = f"Ultimo tracking {time_since_tracking:.1f}s fa (timeout superato)"
            response.target_x = 0.0
            response.target_y = 0.0
            return response
        
        # Calcolo coordinate target con offset approach_distance
        target_x = self.last_tracked_object["x"]
        target_y = self.last_tracked_object["y"]
        
        distance = self.last_tracked_object["distance"]
        
        if distance > approach_distance:
            # Riduzione distanza per fermata a approach_distance dall'oggetto
            # in pratiica si ferma poco prima dell'obiettivo 
            scale = (distance - approach_distance) / distance
            target_x *= scale
            target_y *= scale
        else:
            response.success = False
            response.message = f"Oggetto troppo vicino ({distance:.2f}m)"
            response.target_x = 0.0
            response.target_y = 0.0
            return response
        
        self.get_logger().info(
            f"Navigazione verso '{self.last_tracked_object['target_name']}': "
            f"target=({target_x:.2f}, {target_y:.2f})"
        )
        
        # Verifica disponibilità action server navigating
        if not self.navigating_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Navigating action server non disponibile"
            response.target_x = 0.0
            response.target_y = 0.0
            return response
        
        # Invio goal navigating
        goal = Navigating.Goal()
        goal.target_x = target_x
        goal.target_y = target_y
        goal.use_map = False
        
        future = self.navigating_client.send_goal_async(goal)
        
        # Attesa accettazione goal (non blocca tutto il nodo)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response.success = True
            response.message = f"Navigazione avviata verso '{self.last_tracked_object['target_name']}'"
            response.target_x = target_x
            response.target_y = target_y
        else:
            response.success = False
            response.message = "Errore avvio navigazione"
            response.target_x = 0.0
            response.target_y = 0.0
        
        return response


    def stop_tracking(self):
        """Ferma il tracking"""
        if self.tracker is None:
            self.get_logger().warn("[MOCK] Stop tracking simulato")
            self.active_target = None
            return
        
        try:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()
            self.active_target = None
            self.call_set_posture_sync("Stand", 0.5)
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

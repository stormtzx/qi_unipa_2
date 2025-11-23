import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import cv2
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.srv import GetImage, GetCameraCoordinates, SetPosture, GetTrackedObjectCoordinates, MoveToTrackedObject
from qi_unipa_2_interfaces.msg import Emotion, Tracker
from qi_unipa_2_interfaces.action import Navigating
from geometry_msgs.msg import PointStamped
import math



# Costanti
K_QVGA = 1
K_VGA = 2
K_HD = 4
K_4VGA = 3
K_RGB = 13
K_BGR = 11



class QiUnipa2_vision(Node):
    
    def __init__(self):
        super().__init__('qi_unipa_2_vision')
    
        self.bridge = CvBridge()
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        self.declare_parameter('camera_index', 0)
        
        # Leggi parametri PRIMA di Utils
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        
        
        # Crea Utils DOPO aver letto mock_mode
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)

        # Variabile per memorizzare target attivo
        self.active_target = None
        
        # Variabile per memorizzare ultima posizione oggetto tracciato (con timestamp)
        self.last_tracked_object = None



        # Connessione
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.video_service = None
            self.memory = None
            self.tracker = None
            self.motion = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session 
                self.video_service = self.session.service("ALVideoDevice")
                self.memory = self.session.service("ALMemory")
                self.tracker = self.session.service("ALTracker")
                self.motion = self.session.service("ALMotion")
                self.tracker.setMode("Head")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.video_service = None
                self.memory = None
                self.tracker = None
                self.motion = None

        # SERVICES
        self.get_coords_service = self.create_service(GetCameraCoordinates,'/pepper/services/get_camera_coordinates',self.get_camera_coordinates)
        self.image_service = self.create_service(GetImage, '/pepper/services/get_image', self.get_image)
        self.set_posture_client = self.create_client(SetPosture, '/pepper/services/set_posture')
        self.get_coords_service = self.create_service(GetTrackedObjectCoordinates,'/pepper/services/get_tracked_object_coordinates',
            self.get_tracked_object_coordinates)
        
        self.move_to_tracked_service = self.create_service(MoveToTrackedObject,'/pepper/services/move_to_tracked_object',
            self.move_to_tracked_obj_callback)


        # TOPIC
        # Publisher per coordinate oggetto tracciato (opzionale, continuo)
        self.tracked_coords_pub = self.create_publisher(PointStamped,'/pepper/topics/tracked_coordinates',10)
        # Publisher per le immagini
        self.camera_pub = self.create_publisher(Image, '/pepper/topics/camera_call', 10)
        self.emotion_pub = self.create_publisher(Emotion, '/pepper/topics/emotion', 10)  # Riconoscimento emozioni
        
        # /get_video (UGUALE al vecchio /get_camera - cattura singola)
        self.camera_sub = self.create_subscription(Bool, "/pepper/topics/get_video", self.get_video, 10)
        self.tracking_sub = self.create_subscription(Tracker,"/pepper/topics/tracker",self.start_tracking,qos_best_effort_10)
        
        #ACTION
        self.navigating_client = ActionClient(self,Navigating,'/pepper/actions/navigating')

        

        self.get_logger().info("Nodo vision avviato")
        
        #Timer per pubblicare emozioni
        self.create_timer(0.5, self.publish_emotion)  # 2Hz

        # Timer per pubblicazione continua coordinate (10 Hz)
        self.create_timer(0.1, self.publish_tracked_coordinates)



    # ========================================
    # VIDEO/CAMERA (Topic - ALIAS DEL VECCHIO get_camera)
    # ========================================
    
    def get_video(self, msg):
        """Topic /get_video: cattura singole immagini (IDENTICO al vecchio get_camera)"""
        
        if self.video_service is None:
            # MOCK MODE
            self.get_logger().warn("[MOCK] Cattura video simulata")
            mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(mock_image, "MOCK VIDEO", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            
            img_msg = self.bridge.cv2_to_imgmsg(mock_image, encoding="bgr8")
            self.camera_pub.publish(img_msg)
            self.get_logger().info('\n##############\nMOCK Image Acquired')
            return
        
        # MODALITÀ REALE - ESATTAMENTE COME PRIMA
        video_client = None  # FIX 4: Inizializza variabile
        try:
            video_client = self.video_service.subscribeCamera(
                "Camera", self.camera_index, K_4VGA, K_RGB, 30
            )
            
            result = self.video_service.getImageRemote(video_client)
            
            if result is None:
                self.get_logger().info('\n##############\nUnable to acquire image')
                return
            
            width = result[0]
            height = result[1]
            channels = result[2]
            image_binary = result[6]
            
            # Converti l'immagine binaria in un array numpy
            image_array = np.frombuffer(image_binary, dtype=np.uint8)
            image_array = image_array.reshape((height, width, channels))
            
            # Pubblica l'immagine
            img_msg = self.bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
            self.camera_pub.publish(img_msg)
            self.get_logger().info('\n##############\nImage Acquired')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
            
        finally:
            if video_client is not None and self.video_service is not None:
                try:
                    self.video_service.unsubscribe(video_client)
                except:
                    pass



    # ========================================
    # FOTO SINGOLA (Service - NUOVO)
    # ========================================
    
    def get_image(self, request, response):
        """Service /get_image: foto singola con parametri personalizzabili"""
        
        camera_index = request.camera_index if request.camera_index in [0, 1] else 0
        resolution = request.resolution if request.resolution in [1, 2, 3, 4] else K_4VGA
        
        self.get_logger().info(f"Richiesta foto: camera={camera_index}, res={resolution}")
        
        if self.video_service is None:
            # MOCK MODE
            self.get_logger().warn("[MOCK] Foto simulata")
            mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(mock_image, "MOCK PHOTO", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            
            response.image = self.bridge.cv2_to_imgmsg(mock_image, encoding="bgr8")
            response.success = True
            response.message = "MOCK MODE - Foto simulata"
            return response
        
        # MODALITÀ REALE
        video_client = None
        try:
            video_client = self.video_service.subscribeCamera(
                "SinglePhotoClient", 
                camera_index, 
                resolution, 
                K_RGB, 
                30
            )
            
            # Stabilizzazione
            time.sleep(0.3)
            
            result = self.video_service.getImageRemote(video_client)
            
            if result is None:
                raise RuntimeError("Nessun dato dalla camera")
            
            width = result[0]
            height = result[1]
            channels = result[2]
            image_binary = result[6]
            
            image_array = np.frombuffer(image_binary, dtype=np.uint8).reshape((height, width, channels))
            image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
            
            response.image = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            response.success = True
            response.message = f"Foto acquisita: {width}x{height}"  # FIX 5: Completa messaggio
            
            self.get_logger().info(f"✓ {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            self.get_logger().error(response.message)
            
        finally:
            if video_client is not None and self.video_service is not None:
                try:
                    self.video_service.unsubscribe(video_client)
                except:
                    pass
        
        return response
    


    def publish_emotion(self):
        """Pubblica emozione corrente"""
        
        # MOCK MODE
        if self.memory is None:
            # MOCK MODE - Non pubblica nulla
            return
        
        try:
            # Leggi emozione da ALMemory
            emotion_data = self.memory.getData("EmotionDetection/Emotion")
            # Esempio: Returns: {"emotion": "happy", "confidence": 0.85}
            
            msg = Emotion()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.emotion = emotion_data["emotion"]
            msg.confidence = emotion_data["confidence"]
            
            self.emotion_pub.publish(msg)
            
        except Exception as e:
            # Nessuna emozione rilevata (silenzioso)
            pass


    def get_camera_coordinates(self, request, response):
        """
        Service per ottenere coordinate 3D da depth camera.
        Converte pixel (x, y) in coordinate robot frame (x, y, z).
        """
        pixel_x = request.pixel_x
        pixel_y = request.pixel_y
        camera_index = request.camera_index if request.camera_index in [0, 1] else 0
        
        self.get_logger().info(f"Get coordinates: pixel=({pixel_x}, {pixel_y}), camera={camera_index}")
        
        # MOCK MODE
        if self.video_service is None:
            self.get_logger().warn("[MOCK] Coordinate simulate")
            
            response.success = True
            response.message = "MOCK: Coordinate simulate"
            response.x = 2.0 + (pixel_x - 320) * 0.001
            response.y = 0.3
            response.z = 0.0
            response.distance = 2.1
            response.confidence = 0.8
            return response
        
        # ESECUZIONE REALE
        video_client = None
        try:
            # Sottoscrizione camera DEPTH (type=11 = kDepthColorSpace)
            video_client = self.video_service.subscribeCamera(
                "DepthClient",
                camera_index,
                K_QVGA,  # Risoluzione depth map
                11,      # kDepthColorSpace
                30
            )
            
            time.sleep(0.2)
            
            # Ottieni depth map
            depth_data = self.video_service.getImageRemote(video_client)
            
            if depth_data is None:
                raise RuntimeError("Nessun dato depth dalla camera")
            
            # ESTRAZIONE DIMENSIONI REALI
            width = depth_data[0]
            height = depth_data[1]
            num_layers = depth_data[2]  # Per depth dovrebbe essere 1
            depth_binary = depth_data[6]
            
            # CALCOLO DIMENSIONE ATTESA
            # Depth è uint16 (2 byte per pixel)
            expected_size = width * height * 2  # 2 byte per pixel uint16
            actual_size = len(depth_binary)
            
            # LOGGING DEBUG
            self.get_logger().info(
                f"Depth image: {width}x{height}x{num_layers}, "
                f"expected_bytes={expected_size}, actual_bytes={actual_size}"
            )
            
            # VALIDAZIONE
            if actual_size != expected_size:
                # Prova a inferire dimensioni corrette
                num_pixels = actual_size // 2  # Dividi per 2 (uint16)
                self.get_logger().warn(
                    f"Size mismatch! Inferring dimensions from {num_pixels} pixels"
                )
                
                # Prova dimensioni comuni
                if num_pixels == 57600:  # 240x240
                    width, height = 240, 240
                elif num_pixels == 76800:  # 320x240
                    width, height = 320, 240
                elif num_pixels == 115200:  # 480x240
                    width, height = 480, 240
                else:
                    raise ValueError(
                        f"Cannot infer dimensions from {num_pixels} pixels "
                        f"(bytes={actual_size})"
                    )
                
                self.get_logger().info(f"Using inferred dimensions: {width}x{height}")
            
            # RESHAPE CON DIMENSIONI CORRETTE
            depth_array = np.frombuffer(depth_binary, dtype=np.uint16).reshape((height, width))
            
            # Verifica bounds pixel
            if pixel_x < 0 or pixel_x >= width or pixel_y < 0 or pixel_y >= height:
                raise ValueError(f"Pixel ({pixel_x}, {pixel_y}) fuori bounds (max {width}x{height})")
            
            # Leggi depth al pixel richiesto (in mm)
            depth_mm = depth_array[pixel_y, pixel_x]
            depth_m = depth_mm / 1000.0
            
            # Verifica validità depth
            if depth_mm == 0 or depth_m > 10.0:
                raise ValueError(f"Depth non valido: {depth_mm}mm")
            
            # PARAMETRI CAMERA PEPPER
            FOV_H = 58.0 * (np.pi / 180.0)
            FOV_V = 45.0 * (np.pi / 180.0)
            
            cx = width / 2.0
            cy = height / 2.0
            
            focal_length = width / (2.0 * np.tan(FOV_H / 2.0))
            
            # Offset pixel dal centro
            dx = pixel_x - cx
            dy = pixel_y - cy
            
            # Calcola angoli
            angle_h = np.arctan(dx / focal_length)
            angle_v = np.arctan(dy / focal_length)
            
            # Coordinate camera frame
            camera_z = depth_m * np.cos(angle_v) * np.cos(angle_h)
            camera_x = depth_m * np.cos(angle_v) * np.sin(angle_h)
            camera_y = depth_m * np.sin(angle_v)
            
            # Trasformazione camera → robot frame
            robot_x = camera_z
            robot_y = -camera_x
            robot_z = -camera_y
            
            distance = np.sqrt(robot_x**2 + robot_y**2 + robot_z**2)
            
            # Confidence
            if depth_m < 0.5:
                confidence = 0.3
            elif depth_m > 4.0:
                confidence = 0.5
            else:
                confidence = 0.9
            
            # Risposta
            response.success = True
            response.message = "Coordinate ottenute da depth"
            response.x = float(robot_x)
            response.y = float(robot_y)
            response.z = float(robot_z)
            response.distance = float(distance)
            response.confidence = float(confidence)
            
            self.get_logger().info(
                f"✓ x={robot_x:.2f}m, y={robot_y:.2f}m, z={robot_z:.2f}m, "
                f"dist={distance:.2f}m, conf={confidence:.2f}"
            )
            
        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.distance = 0.0
            response.confidence = 0.0
            self.get_logger().error(response.message)
            
        finally:
            if video_client is not None and self.video_service is not None:
                try:
                    self.video_service.unsubscribe(video_client)
                except:
                    pass
        
        return response



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
    node = QiUnipa2_vision()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()

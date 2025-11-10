import qi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import cv2
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.srv import GetImage, GetCoordinates
from qi_unipa_2_interfaces.msg import Emotion


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


        # Connessione
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.video_service = None
            self.memory = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session 
                self.video_service = self.session.service("ALVideoDevice")
                self.memory = self.session.service("ALMemory")  # ← FIX 1: Aggiungi ALMemory
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.video_service = None
                self.memory = None

        # Services
        self.get_coords_service = self.create_service(GetCoordinates,'/get_coordinates',self.get_coordinates)
            
        # Publisher per le immagini
        self.camera_pub = self.create_publisher(Image, '/camera_call', 10)
        self.emotion_pub = self.create_publisher(Emotion, '/emotion', 10)  # Riconoscimento emozioni
        
        # Topic: /get_video (UGUALE al vecchio /get_camera - cattura singola)
        self.camera_sub = self.create_subscription(Bool, "/get_video", self.get_video, 10)
        
        self.image_service = self.create_service(GetImage, '/get_image', self.get_image)
        
        self.get_logger().info("Nodo vision avviato")
        
        #Timer per pubblicare emozioni
        self.create_timer(0.5, self.publish_emotion)  # 2Hz


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

    #Funzione per stimare le coordinate tramite la camera 3d di pepper
    def get_coordinates(self, request, response):
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
                K_QVGA,  # Depth map è sempre QVGA (320x240)
                11,      # kDepthColorSpace
                30
            )
            
            time.sleep(0.2)
            
            # Ottieni depth map
            depth_data = self.video_service.getImageRemote(video_client)
            
            if depth_data is None:
                raise RuntimeError("Nessun dato depth dalla camera")
            
            width = depth_data[0]
            height = depth_data[1]
            depth_binary = depth_data[6]
            
            # Converte depth binary in array numpy (depth in mm, uint16)
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



def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_vision()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()

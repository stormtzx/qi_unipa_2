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
from qi_unipa_2_interfaces.srv import GetImage


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
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session 
                self.video_service = self.session.service("ALVideoDevice")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.video_service = None
                    
        # Publisher per le immagini
        self.camera_pub = self.create_publisher(Image, '/camera_call', 10)
        
        # Topic: /get_video (UGUALE al vecchio /get_camera - cattura singola)
        self.camera_sub = self.create_subscription(Bool, "/get_video", self.get_video, 10)
        

        self.image_service = self.create_service(GetImage, '/get_image', self.get_image_callback)
        
        self.get_logger().info("Nodo vision avviato")


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
            if self.video_service is not None:
                try:
                    self.video_service.unsubscribe(video_client)
                except:
                    pass


    # ========================================
    # FOTO SINGOLA (Service - NUOVO)
    # ========================================
    
    def get_image_callback(self, request, response):
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
            response.message = f"Foto acquisita:"# {width}x{height}"
            
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


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_vision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

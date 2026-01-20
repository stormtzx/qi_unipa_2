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
from qi_unipa_2_interfaces.msg import TrackingState
from qi_unipa_2_interfaces.srv import GetImage, GetEmotion, GetCameraCoordinates, SetPosture, GetTrackedSubjectCoordinates, SetJointAngles, StartTracking, StopTracking
from qi_unipa_2_interfaces.action import Navigating
from geometry_msgs.msg import PointStamped
import math
import os
from datetime import datetime



# ========================================
# COSTANTI GLOBALI
# ========================================

# Risoluzioni camera
K_QVGA = 1   # 320x240
K_VGA = 2    # 640x480
K_4VGA = 3   # 640x480 (alternativo)
K_HD = 4     # 1280x960

# Colorspace RGB/BGR (per foto normali)
K_RGB = 13
K_BGR = 11

# Colorspace DEPTH (per depth camera)
K_DEPTH = 17        # kRawDepthColorSpace (raw uint16)
K_DEPTH_MM = 21     # kDepthColorSpace (millimetri, più compatibile)




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

        # Connessione
        if mock_mode:
            self.get_logger().warn("Nodo VISION attivo in MOCK MODE")
            self.session = None
            self.vision_device = None
            self.memory = None
            self.tracker = None
            self.motion = None

        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session 
                self.vision_device = self.session.service("ALVideoDevice")
                self.memory = self.session.service("ALMemory")
                self.tracker = self.session.service("ALTracker")
                self.motion = self.session.service("ALMotion")
                self.tracker.setMode("Head")
                self.get_logger().info("Nodo VISION attivo e connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e


        # Services
        self.start_tracking_service = self.create_service(
            StartTracking,
            '/pepper/services/start_tracking',
            self.start_tracking
        )

        self.stop_tracking_service = self.create_service(
            StopTracking,
            '/pepper/services/stop_tracking',
            self.stop_tracking
        )

        self.set_posture_client = self.create_client(
            SetPosture, 
            '/pepper/services/set_posture'
        )

        self.set_joint_angles_client = self.create_client(
            SetJointAngles,
            '/pepper/services/set_joint_angles'
            )
        
        self.image_service = self.create_service(
            GetImage, 
            '/pepper/services/get_image', 
            self.get_image
        )

        self.get_camera_coords_service = self.create_service(
            GetCameraCoordinates,
            '/pepper/services/get_camera_coordinates',
            self.get_camera_coordinates
        )


        self.get_tracked_subject_coords_service = self.create_service(
            GetTrackedSubjectCoordinates,
            '/pepper/services/get_tracked_subject_coordinates',
            self.get_tracked_subject_coordinates
        )

        self.get_emotion = self.create_service(
            GetEmotion,
            '/pepper/services/get_emotion',
            self.get_emotion
        )

        # Publishers

        # Publisher per stato tracking
        self.tracking_state_pub = self.create_publisher(
            TrackingState,
            '/pepper/topics/tracking_state',
            10
        )
       


        
        
        # TOPIC
        # Publisher per le immagini
        self.camera_pub = self.create_publisher(Image, '/pepper/topics/camera_call', 10)
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Bool, 
            "/pepper/topics/get_video", 
            self.get_video, 
            10
        ) 
        
        

        # ACTION
        self.navigating_client = ActionClient(self, Navigating, '/pepper/actions/navigating')

        


    # ========================================
    # VIDEO/CAMERA (Topic)
    # ========================================
    
    def get_video(self, msg):
        """Topic /get_video: cattura singole immagini"""
        
        if self.vision_device is None:
            # MOCK MODE
            self.get_logger().warn("[MOCK] Cattura video simulata")
            mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(mock_image, "MOCK VIDEO", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            
            img_msg = self.bridge.cv2_to_imgmsg(mock_image, encoding="bgr8")
            self.camera_pub.publish(img_msg)
            self.get_logger().info('\n##############\nMOCK Image Acquired')
            return
        
        # MODALITÀ REALE
        vision_client = None
        try:
            vision_client = self.vision_device.subscribeCamera(
                "Camera", self.camera_index, K_4VGA, K_RGB, 30
            )
            
            result = self.vision_device.getImageRemote(vision_client)
            
            if result is None:
                self.get_logger().info('\n##############\nUnable to acquire image')
                return
            
            width = result[0]
            height = result[1]
            channels = result[2]
            image_binary = result[6]
            
            image_array = np.frombuffer(image_binary, dtype=np.uint8)
            image_array = image_array.reshape((height, width, channels))
            
            img_msg = self.bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
            self.camera_pub.publish(img_msg)
            self.get_logger().info('\n##############\nImage Acquired')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
            
        finally:
            if vision_client is not None and self.vision_device is not None:
                try:
                    self.vision_device.unsubscribe(vision_client)
                except:
                    pass




    # ================================================
    # FOTO SINGOLA (Service)
    # ================================================
    def get_image(self, request, response):
        """Service /get_image: foto singola salvata su disco"""

        camera_index = request.camera_index if request.camera_index in [0, 1] else 0
        resolution = request.resolution if request.resolution in [1, 2, 3, 4] else K_4VGA

        save_dir = os.environ.get("ROS2_CAPTURES_PATH", "/home/daniele/Scrivania/ros2_ws/src/qi_unipa_2/pepper_captures")
        os.makedirs(save_dir, exist_ok=True)

        if hasattr(request, 'file_name') and request.file_name.strip():
            filename = request.file_name.strip()
            if not filename.lower().endswith('.jpg'):
                filename += '.jpg'
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"pepper_photo_{timestamp}_cam{camera_index}.jpg"

        filepath = os.path.join(save_dir, filename)

        self.get_logger().info(f"Richiesta foto: camera={camera_index}, res={resolution}, salvataggio: {filepath}")

        if self.vision_device is None:
            self.get_logger().warn("[MOCK] Foto simulata")
            mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(mock_image, "MOCK PHOTO", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            cv2.imwrite(filepath, mock_image)
            response.success = True
            response.message = f"[MOCK] Foto salvata: {filename}"
            return response

        vision_client = None
        try:
            vision_client = self.vision_device.subscribeCamera(
                "SinglePhotoClient",
                camera_index,
                resolution,
                K_RGB,
                30
            )

            time.sleep(0.3)

            result = self.vision_device.getImageRemote(vision_client)

            if result is None:
                raise RuntimeError("Nessun dato dalla camera")

            width = result[0]
            height = result[1]
            channels = result[2]
            image_binary = result[6]

            image_array = np.frombuffer(image_binary, dtype=np.uint8).reshape((height, width, channels))
            
            cv2.imwrite(filepath, image_array)

            response.success = True
            response.message = f"Foto acquisita: {width}x{height}, salvata in {filename}"

            self.get_logger().info(f"✓ Foto salvata: {filepath}")

        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            self.get_logger().error(response.message)

        finally:
            if vision_client is not None and self.vision_device is not None:
                try:
                    self.vision_device.unsubscribe(vision_client)
                except:
                    pass

        return response



    
    def call_set_joint_angles_sync(self, names, angles, speed):
        """Chiama service SetJointAngles per muovere articolazioni"""
        req = SetJointAngles.Request()
        req.names = names
        req.angles = angles
        req.speed = speed
        self.set_joint_angles_client.call_async(req)

        


    def call_set_posture_sync(self, posture_name, speed):
        """Wrapper sincrono per chiamare set_posture"""
        if not self.set_posture_client.service_is_ready():
            self.get_logger().warn(f"[MOCK/Service non pronto] set_posture '{posture_name}' ignorato")
            return
        
        request = SetPosture.Request()
        request.posture_name = posture_name
        request.speed = speed
        self.set_posture_client.call_async(request) 
        self.get_logger().debug(f"Chiamata set_posture: {posture_name} @ speed {speed}")


    # ========================================
    # TRACKING - FUNZIONI SEMPLICI
    # ========================================

    def start_tracking(self, request, response):
        """
        Service per avviare tracking.
        """
        SUPPORTED_TARGETS = ["Face", "People", "Sound"]
        
        # Validazione
        if request.target_name not in SUPPORTED_TARGETS:
            response.success = False
            response.message = f"Target '{request.target_name}' non supportato. Usa: {SUPPORTED_TARGETS}"
            response.active_target = self.active_target or ""
            return response
        
        # MOCK MODE
        if self.tracker is None:
            self.get_logger().warn(f"[MOCK] Tracking simulato: {request.target_name}")
            self.active_target = request.target_name
            self.tracking_state_pub.publish(TrackingState(is_tracking_on=True))
            response.success = True
            response.message = f"[MOCK] Tracking {request.target_name} attivato"
            response.active_target = self.active_target
            return response
        
        # REAL MODE
        try:
            self.call_set_posture_sync("Stand", 0.5)
            self.call_set_joint_angles_sync(["HeadPitch"], [0.15], 0.15)
            
            # Parametri target-specific
            if request.target_name == "Face":
                params = 0.15
            elif request.target_name == "People":
                params = [0]
            elif request.target_name == "Sound":
                params = [3.0, 0.5]
            
            distance = request.distance if request.distance > 0 else 2.0
            
            self.tracker.registerTarget(request.target_name, params)
            self.tracker.setRelativePosition([
                -distance, 0.0, 0.0,
                0.1, 0.1, 0.2
            ])
            self.tracker.track(request.target_name)
            self.active_target = request.target_name
            
            # Pubblica stato tracking
            self.tracking_state_pub.publish(TrackingState(is_tracking_on=True))
            
            response.success = True
            response.message = f"Tracking attivato: {request.target_name} (distanza: {distance:.1f}m)"
            response.active_target = self.active_target
            
            self.get_logger().info(f"✓ {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            response.active_target = self.active_target or ""
            self.active_target = None
            self.tracking_state_pub.publish(TrackingState(is_tracking_on=False))
            self.get_logger().error(f"❌ start_tracking: {e}")
        
        return response


    def stop_tracking(self, request, response):
        """
        Service per fermare tracking.
        """
        # MOCK MODE
        if self.tracker is None:
            self.get_logger().warn("[MOCK] Stop tracking simulato")
            self.active_target = None
            self.tracking_state_pub.publish(TrackingState(is_tracking_on=False))
            response.success = True
            response.message = "[MOCK] Tracking fermato"
            return response
        
        # REAL MODE
        try:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()
            self.active_target = None
            self.call_set_posture_sync("Stand", 0.5)
            
            # Pubblica stato tracking
            self.tracking_state_pub.publish(TrackingState(is_tracking_on=False))
            
            response.success = True
            response.message = "Tracking fermato"
            self.get_logger().info("✓ Tracking fermato")
            
        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            self.get_logger().error(f"❌ stop_tracking: {e}")
        
        return response


    # ========================================
    # GET EMOTION (con tracking come interruttore)
    # ========================================

    def get_emotion(self, request, response):
        """
        Service per rilevare emozione.
        Usa tracking come interruttore: avvia se spento, spegne alla fine.
        """
        tracking_was_off = False
        
        # MOCK MODE
        if self.memory is None:
            time.sleep(1.5)
            response.emotion = "happy"
            response.confidence = 0.85
            self.get_logger().info("[MOCK] Emozione simulata: happy (0.85)")
            return response
        
        try:
            # 1. AVVIA TRACKING (se spento)
            tracking_was_off = (self.active_target is None)
            
            if tracking_was_off:
                self.get_logger().info("Avvio tracking Face per rilevare emozione...")
                
                # Chiama start_tracking internamente
                req = StartTracking.Request()
                req.target_name = "Face"
                req.distance = 2.0
                
                res = StartTracking.Response()
                self.start_tracking(req, res)
                
                if not res.success:
                    raise RuntimeError(f"Impossibile avviare tracking: {res.message}")
                
                time.sleep(1.0)  # Attendi stabilizzazione
            
            # 2. RILEVA EMOZIONE
            time.sleep(2.0)  # Stabilizzazione emotion detection
            emotion_data = self.memory.getData("EmotionDetection/Emotion")
            
            response.emotion = str(emotion_data.get("emotion", "neutral"))
            response.confidence = float(emotion_data.get("confidence", 0.0))
            
            self.get_logger().info(f"✓ Emozione: {response.emotion} ({response.confidence:.2f})")
            
        except Exception as e:
            response.emotion = "error"
            response.confidence = 0.0
            self.get_logger().error(f"❌ Errore get_emotion: {e}")
        
        finally:
            # 3. SPEGNI TRACKING (se era spento prima)
            if tracking_was_off:
                req_stop = StopTracking.Request()
                res_stop = StopTracking.Response()
                self.stop_tracking(req_stop, res_stop)
                self.get_logger().debug("✓ Tracking fermato (interruttore)")
        
        return response


    # ========================================
    # GET TRACKED SUBJECT COORDINATES (con tracking come interruttore)
    # ========================================
    def get_tracked_subject_coordinates(self, request, response):
        """
        Service per ottenere coordinate persona dal tracker NAOqi.
        Avvia tracking → legge coordinate → ferma tracking.
        """
        tracking_was_off = False
        
        try:
            tracking_was_off = (self.active_target is None)
            
            self.get_logger().info(f"🔍 get_coordinates: active_target={self.active_target}")
            
            # MOCK MODE
            if self.tracker is None:
                self.get_logger().info("🎭 MOCK MODE")
                time.sleep(1.0)
                response.success = True
                response.message = "[MOCK] Coordinate simulate"
                response.target_name = "People"
                response.x = 1.5
                response.y = 0.0
                response.z = 0.0
                response.distance = 1.5
                response.theta = 0.0
                response.frame = "ROBOT"
                return response
            
            # 1. AVVIA TRACKING (se spento)
            if tracking_was_off:
                self.get_logger().info("🚀 Avvio tracking People...")
                
                req = StartTracking.Request()
                req.target_name = "People"
                req.distance = 2.0
                
                res = StartTracking.Response()
                self.start_tracking(req, res)
                
                if not res.success:
                    raise RuntimeError(f"Impossibile avviare tracking: {res.message}")
                
                self.get_logger().info(f"✅ Tracking avviato!")
                time.sleep(2.0)
            
            # 2. DEBUG: Verifica tracker state
            self.get_logger().info(f"🔍 Tracker object: {self.tracker}")
            self.get_logger().info(f"🔍 Active target: {self.active_target}")
            
            # 3. LEGGI COORDINATE
            self.get_logger().info(f"📍 Inizio lettura coordinate (5 tentativi)...")
            
            position = None
            valid_position = None
            
            for attempt in range(5):
                try:
                    # ✅ Chiamata corretta
                    position = self.tracker.getTargetPosition()
                    
                    # ✅ LOG RAW - FONDAMENTALE
                    self.get_logger().info(f"🔍 Tentativo {attempt+1}/5:")
                    self.get_logger().info(f"   position={position}")
                    self.get_logger().info(f"   type={type(position)}")
                    
                    if position is None:
                        self.get_logger().info(f"   ❌ Position è None")
                    elif not isinstance(position, (list, tuple)):
                        self.get_logger().info(f"   ❌ Position non è lista/tuple")
                    elif len(position) < 2:
                        self.get_logger().info(f"   ❌ Position ha len < 2: {len(position)}")
                    else:
                        # Tenta conversione
                        try:
                            theta = float(position[0])
                            distance = float(position[1])
                            
                            self.get_logger().info(f"   ✅ theta={theta:.3f}, distance={distance:.3f}m")
                            
                            # Validazione
                            if 0.2 <= distance <= 10.0:
                                self.get_logger().info(f"   ✅ VALIDO! Uso questo.")
                                valid_position = position
                                break
                            else:
                                self.get_logger().info(f"   ❌ Distance fuori range (0.2-10.0m)")
                        
                        except (ValueError, TypeError) as e:
                            self.get_logger().info(f"   ❌ Errore conversione: {e}")
                
                except Exception as e:
                    self.get_logger().info(f"   ❌ Exception: {e}")
                
                time.sleep(0.5)
            
            if valid_position is None:
                raise RuntimeError("Nessuna coordinata valida dopo 5 tentativi")
            
            # 4. CONVERTI IN COORDINATE CARTESIANE
            theta = float(valid_position[0])
            distance = float(valid_position[1])
            
            x = distance * math.cos(theta)
            y = distance * math.sin(theta)
            z = 0.0
            
            response.success = True
            response.message = f"Coordinate rilevate (tentativo {attempt+1}/5)"
            response.target_name = self.active_target or "People"
            response.x = float(x)
            response.y = float(y)
            response.z = float(z)
            response.distance = float(distance)
            response.theta = float(theta)
            response.frame = "ROBOT"
            
            self.get_logger().info(
                f"✓ SUCCESSO! Coordinate: x={x:.2f}m, y={y:.2f}m, "
                f"dist={distance:.2f}m, theta={theta:.2f}rad"
            )
            
        except Exception as e:
            response.success = False
            response.message = f"Errore: {str(e)}"
            response.target_name = ""
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.distance = 0.0
            response.theta = 0.0
            response.frame = ""
            self.get_logger().error(f"❌ get_tracked_subject_coordinates: {e}")
            
            import traceback
            self.get_logger().error(f"❌ {traceback.format_exc()}")
        
        finally:
            # SPEGNI TRACKING (se era spento prima)
            if tracking_was_off:
                self.get_logger().info(f"🛑 Stop tracking...")
                
                try:
                    req_stop = StopTracking.Request()
                    res_stop = StopTracking.Response()
                    self.stop_tracking(req_stop, res_stop)
                    self.get_logger().info(f"🛑 Tracking fermato")
                except Exception as e:
                    self.get_logger().error(f"❌ Errore stop: {e}")
        
        self.get_logger().info(f"🏁 FINE get_coordinates")
        
        return response






    # Purtroppo sembra non funzionare correttamente
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
        if self.vision_device is None:
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
        vision_client = None
        try:
            # Sottoscrizione depth camera con colorspace 21
            vision_client = self.vision_device.subscribeCamera(
                "DepthClient",
                camera_index,
                K_QVGA,
                K_DEPTH_MM,  # Colorspace 21
                30
            )
            
            # Flush buffer
            time.sleep(0.5)
            for i in range(3):
                depth_data = self.vision_device.getImageRemote(vision_client)
                if i < 2:
                    time.sleep(0.1)
            
            if depth_data is None:
                raise RuntimeError("Nessun dato depth dalla camera")
            
            # ESTRAZIONE DIMENSIONI
            width = depth_data[0]
            height = depth_data[1]
            num_layers = depth_data[2]
            depth_binary = depth_data[6]
            actual_size = len(depth_binary)
            
            self.get_logger().info(
                f"Depth image: {width}x{height}x{num_layers}, bytes={actual_size}"
            )
            
            # ✅ CALCOLA LAYER REALI (ignora header se sbagliato)
            num_elements = actual_size // 2  # Numero di uint16
            num_pixels_per_frame = width * height
            calculated_layers = num_elements // num_pixels_per_frame
            
            if calculated_layers != num_layers:
                self.get_logger().warn(
                    f"⚠️ Header: {num_layers} layer, dati: {calculated_layers} layer → uso {calculated_layers}"
                )
                num_layers = calculated_layers
            
            # RESHAPE
            if num_layers == 1:
                depth_array = np.frombuffer(depth_binary, dtype=np.uint16).reshape((height, width))
                self.get_logger().info("✓ Depth monocanale")
            elif num_layers == 2:
                full_array = np.frombuffer(depth_binary, dtype=np.uint16).reshape((height, width, 2))
                depth_array = full_array[:, :, 0]  # Solo depth (primo layer)
                self.get_logger().info("✓ Estratto depth da array 2-layer")
            else:
                raise ValueError(f"Layer inatteso: {num_layers}")
            
            # Verifica bounds pixel
            if pixel_x < 0 or pixel_x >= width or pixel_y < 0 or pixel_y >= height:
                raise ValueError(f"Pixel fuori bounds: ({pixel_x}, {pixel_y})")
            
            # ✅ DIAGNOSTICA GLOBALE
            depth_flat = depth_array.flatten()
            depth_nonzero = depth_flat[depth_flat > 0]
            
            if len(depth_nonzero) > 0:
                median_depth = np.median(depth_nonzero)
                min_depth = np.min(depth_nonzero)
                max_depth = np.max(depth_nonzero)
                
                self.get_logger().info(
                    f"📊 Depth stats: min={min_depth}mm, median={median_depth:.0f}mm, max={max_depth}mm"
                )
                
                # Verifica se depth sembra corrotto
                if median_depth > 20000:
                    raise RuntimeError(
                        f"Depth corrotta: mediana globale {median_depth}mm > 20m"
                    )
            else:
                raise RuntimeError("Tutti i pixel depth sono zero")
            
            # FUNZIONE VALIDAZIONE
            def is_valid_depth(depth_mm):
                if depth_mm == 0:
                    return False
                depth_m = depth_mm / 1000.0
                return 0.3 <= depth_m <= 4.0
            
            # TENTATIVO DEPTH CON FALLBACK
            depth_mm = None
            attempts_log = []
            
            # Tentativo 1: Pixel centro
            depth_mm_center = depth_array[pixel_y, pixel_x]
            attempts_log.append(f"center={depth_mm_center}mm")
            
            if is_valid_depth(depth_mm_center):
                depth_mm = depth_mm_center
                self.get_logger().info(f"✓ Depth valido: {depth_mm}mm")
            else:
                # Tentativo 2: Finestra 3x3
                self.get_logger().warn(f"⚠️ Centro invalido ({depth_mm_center}mm), provo 3x3...")
                y_start, y_end = max(0, pixel_y-1), min(height, pixel_y+2)
                x_start, x_end = max(0, pixel_x-1), min(width, pixel_x+2)
                window = depth_array[y_start:y_end, x_start:x_end]
                valid = [d for d in window.flatten() if is_valid_depth(d)]
                
                if len(valid) > 0:
                    depth_mm = int(np.median(valid))
                    attempts_log.append(f"median_3x3={depth_mm}mm")
                    self.get_logger().info(f"✓ Mediana 3x3: {depth_mm}mm ({len(valid)}/9)")
                else:
                    # Tentativo 3: Finestra 5x5
                    self.get_logger().warn("⚠️ Provo 5x5...")
                    y_start, y_end = max(0, pixel_y-2), min(height, pixel_y+3)
                    x_start, x_end = max(0, pixel_x-2), min(width, pixel_x+3)
                    window = depth_array[y_start:y_end, x_start:x_end]
                    valid = [d for d in window.flatten() if is_valid_depth(d)]
                    
                    if len(valid) > 0:
                        depth_mm = int(np.median(valid))
                        attempts_log.append(f"median_5x5={depth_mm}mm")
                        self.get_logger().info(f"✓ Mediana 5x5: {depth_mm}mm ({len(valid)}/25)")
                    else:
                        raise ValueError(
                            f"Depth invalido: centro={depth_mm_center}mm, "
                            f"nessun pixel valido in 5x5"
                        )
            
            # CALCOLO COORDINATE 3D
            depth_m = depth_mm / 1000.0
            
            # Parametri camera Pepper
            FOV_H = 58.0 * (np.pi / 180.0)
            cx = width / 2.0
            cy = height / 2.0
            focal_length = width / (2.0 * np.tan(FOV_H / 2.0))
            
            # Calcola angoli
            angle_h = np.arctan((pixel_x - cx) / focal_length)
            angle_v = np.arctan((pixel_y - cy) / focal_length)
            
            # Coordinate camera frame
            camera_z = depth_m * np.cos(angle_v) * np.cos(angle_h)
            camera_x = depth_m * np.cos(angle_v) * np.sin(angle_h)
            camera_y = depth_m * np.sin(angle_v)
            
            # Trasformazione camera → robot frame
            robot_x = camera_z
            robot_y = -camera_x
            robot_z = -camera_y
            distance = np.sqrt(robot_x**2 + robot_y**2 + robot_z**2)
            
            # CONFIDENCE
            if depth_mm == depth_mm_center:
                confidence = 0.95 if 0.5 <= depth_m <= 3.0 else 0.7
            else:
                confidence = 0.75 if 0.5 <= depth_m <= 3.0 else 0.5
            
            # RESPONSE
            response.success = True
            response.message = f"Depth={depth_mm}mm ({', '.join(attempts_log)})"
            response.x = float(robot_x)
            response.y = float(robot_y)
            response.z = float(robot_z)
            response.distance = float(distance)
            response.confidence = float(confidence)
            
            self.get_logger().info(
                f"✓ Coords: ({robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f})m, "
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
            self.get_logger().error(f"❌ get_camera_coordinates: {response.message}")
            
        finally:
            if vision_client is not None and self.vision_device is not None:
                try:
                    self.vision_device.unsubscribe(vision_client)
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
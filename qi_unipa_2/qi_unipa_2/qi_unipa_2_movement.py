import qi
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from qi_unipa_2.utils import Utils #type: ignore
from qi_unipa_2_interfaces.srv import SetState, SetPosture, SetJointAngles, SetHand, GetTrackedObjectCoordinates, PlayAnimation #type: ignore
from qi_unipa_2_interfaces.action import Walking, Navigating, Seeking #type: ignore
from qi_unipa_2_interfaces.msg import Tracker
import math
import time




class QiUnipa2_movement(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_movement')
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        # Lettura parametri
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self._localization_active = False
        self._current_map_name = None


        # Variabili di stato per Seeking
        self._tracker_data = None
        self._tracking_active = False
        self._last_tracking_time = 0



        # Crea Utils
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        qos_best_effort_5 = utils.get_QoS('best_effort', 5)



        # Connessione condizionale
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.motion = None
            self.posture = None
            self.tracker = None
            self.navigation = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                self.navigation = self.session.service("ALNavigation")
                self.motion = self.session.service("ALMotion")
                self.posture = self.session.service("ALRobotPosture")
                self.tracker = self.session.service("ALTracker")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.motion = None
                self.posture = None
                self.navigation = None
                self.tracker = None




        # Dichiarazione service
        self.create_service(SetState, '/pepper/services/set_state', self.set_state, qos_profile=qos_reliable_10)
        self.create_service(SetJointAngles, '/pepper/services/set_joint_angles', self.set_joint_angles, qos_profile=qos_reliable_10)
        self.create_service(SetPosture, '/pepper/services/set_posture', self.set_posture, qos_profile=qos_reliable_10)
        self.create_service(SetHand, '/pepper/services/set_hand', self.set_hand, qos_profile=qos_reliable_10)
        self.create_service(PlayAnimation, '/pepper/services/play_animation', self.play_animation, qos_profile=qos_reliable_10)
        
        # Dichiarazione action
        self._action_server_walking = ActionServer(self, Walking, '/pepper/actions/walking', self.walking)
        # ActionServer Navigating (NAVIGAZIONE AUTONOMA CON OBSTACLE AVOIDANCE)
        self.navigating_server = ActionServer(self, Navigating, '/pepper/actions/navigating', self.navigating)
        # ActionServer Seeking (SEEKING CON TRACKING E NAVIGAZIONE CONTINUA CON OBSTACLE AVOIDANCE)
        self.seeking_server = ActionServer(self, Seeking, '/pepper/actions/seeking', self.seeking)


        # Subscription a tracker data
        self.create_subscription(Tracker, '/pepper/topics/tracker', self.on_tracker_data, qos_best_effort_5)





    # ========== CALLBACKS TRACKER DATA ==========
    def on_tracker_data(self, msg):
        """Riceve dati di tracking da tracking.py"""
        self._tracker_data = msg
        self._last_tracking_time = time.time()



    def get_tracked_position(self):
        """Estrae coordinate 3D dal tracker data ricevuto"""
        if self._tracker_data is None:
            return None
        
        try:
            # Il messaggio Tracker contiene le coordinate
            x = self._tracker_data.x
            y = self._tracker_data.y
            z = self._tracker_data.z
            return {'x': x, 'y': y, 'z': z}
        except:
            return None



    def request_tracked_object_coordinates(self):
        """Chiama service di tracking per ottenere coordinate attuali"""
        if self.motion is None:
            return None
        
        try:
            # Verifica se esiste il service
            client = self.create_client(GetTrackedObjectCoordinates, '/pepper/services/get_tracked_object_coordinates')
            
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            
            request = GetTrackedObjectCoordinates.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            result = future.result()
            if result and result.success:
                return {
                    'x': result.x,
                    'y': result.y,
                    'z': result.z,
                    'distance': result.distance
                }
        except:
            pass
        
        return None



    #Callback per il service: SetState
    def set_state(self, request, response):
        if self.motion is None:
            self.get_logger().warn("[MOCK] set_state chiamato")
            response.success = True
            response.message = "MOCK: State change simulato"
            return response
        
        state = request.state
        try:
            if state == 0:
                self.motion.wakeUp()
                response.message = "Robot woke up"
            else:
                self.motion.rest()
                response.message = "Robot rested"
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Errore set_state: {e}")
            response.success = False
            response.message = str(e)
        return response
    
 
    #Callback per il service: SetJointAngles
    def set_joint_angles(self, request, response):
        if self.motion is None:
            self.get_logger().warn(f"[MOCK] set_joint_angles chiamato: {request.names}")
            response.success = True
            response.message = "MOCK: Joint movement simulato"
            return response
        
        names = request.names
        angles = request.angles.tolist() if hasattr(request.angles, 'tolist') else list(request.angles)
        speed = request.speed
        
        #  VALIDAZIONE PARAMETRI
        # 1. Verifica che ci siano nomi e angoli
        if not names or not angles:
            self.get_logger().error(" Errore: nomi o angoli vuoti")
            response.success = False
            response.message = "Nomi o angoli vuoti"
            return response
        
        # 2. Verifica lunghezza uguale
        if len(names) != len(angles):
            self.get_logger().error(f" Lunghezza diversa: {len(names)} nomi vs {len(angles)} angoli")
            response.success = False
            response.message = f"Lunghezza diversa: {len(names)} nomi, {len(angles)} angoli"
            return response
        
        # 3. Verifica velocità valida
        if speed <= 0.0 or speed > 1.0:
            self.get_logger().error(f" Velocità non valida: {speed} (deve essere 0 < speed <= 1.0)")
            response.success = False
            response.message = f"Velocità non valida: {speed}"
            return response
        
        # 4. Verifica nomi articolazioni validi
        valid_joints = [
            "HeadYaw", "HeadPitch","HipRoll", "HipPitch", "KneePitch"
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
            "LHand", "RHand",
            
        ]
        
        for name in names:
            if name not in valid_joints:
                self.get_logger().error(f" Articolazione non valida: {name}")
                response.success = False
                response.message = f"Articolazione '{name}' non valida"
                return response
        # 5. Verifica range angoli (in radianti)
        # Range approssimativi per sicurezza
        angle_limits = {
            "HeadYaw": (-2.0857, 2.0857), # Rotazione testa: diminuendo-> testa gira a sinistra / aumentando-> testa gira a destra
            "HeadPitch": (-0.7068, 0.4451), # Inclinazione testa: diminuendo-> testa va in alto / aumentando-> testa va in basso
            "LShoulderPitch": (-2.0857, 2.0857), # Inclinazione spalla sinistra: diminuendo-> braccio sx va in alto / aumentando-> braccio sx va in basso
            "RShoulderPitch": (-2.0857, 2.0857), # Inclinazione spalla destra: diminuendo-> braccio dx va in alto / aumentando-> braccio dx va in basso
            "LShoulderRoll": (0.0087, 1.5620), # Rotazione spalla sinistra: aumentando-> braccio sx si allontana dal corpo (apertura laterale)
            "RShoulderRoll": (-1.5620, -0.0087), # Rotazione spalla destra: diminuendo-> braccio dx si allontana dal corpo (apertura laterale)
            "LElbowYaw": (-2.0857, 2.0857), # Rotazione avambraccio sinistro: diminuendo-> rotazione interna / aumentando-> rotazione esterna
            "RElbowYaw": (-2.0857, 2.0857), # Rotazione avambraccio destro: diminuendo-> rotazione interna / aumentando-> rotazione esterna
            "LElbowRoll": (-1.5620, -0.0087), # Piegamento gomito sinistro: diminuendo-> gomito si piega (mano verso spalla)
            "RElbowRoll": (0.0087, 1.5620), # Piegamento gomito destro: aumentando-> gomito si piega (mano verso spalla)
            "LWristYaw": (-1.8238, 1.8238), # Rotazione polso sinistro: diminuendo-> polso ruota verso interno / aumentando-> polso ruota verso esterno
            "RWristYaw": (-1.8238, 1.8238), # Rotazione polso destro: diminuendo-> polso ruota verso interno / aumentando-> polso ruota verso esterno
            "HipRoll": (-0.5149, 0.5149), # Inclinazione laterale bacino: diminuendo-> corpo si inclina a sinistra / aumentando-> corpo si inclina a destra
            "HipPitch": (-1.0385, 1.0385), # Inclinazione avanti/indietro bacino: diminuendo-> busto si piega indietro / aumentando-> busto si piega in avanti
            "KneePitch": (-0.5149, 0.5149), # Piegamento ginocchia: diminuendo-> ginocchia si estendono / aumentando-> ginocchia si flettono
        }

        for i, (name, angle) in enumerate(zip(names, angles)):
            if name in angle_limits:
                min_angle, max_angle = angle_limits[name]
                if angle < min_angle or angle > max_angle:
                    self.get_logger().warn(
                        f" Angolo fuori range per {name}: {angle:.2f} "
                        f"(range: [{min_angle:.2f}, {max_angle:.2f}])"
                    )
                    # Clamp al range valido
                    angles[i] = max(min_angle, min(angle, max_angle))
                    self.get_logger().warn(f"   → Clamped a: {angles[i]:.2f}")
        
        #  LOG DETTAGLIATO PRIMA DELLA CHIAMATA
        self.get_logger().info(f" set_joint_angles:")
        self.get_logger().info(f"   Names: {names}")
        self.get_logger().info(f"   Angles: {[f'{a:.3f}' for a in angles]}")
        self.get_logger().info(f"   Speed: {speed}")
        
        try:
            self.motion.angleInterpolationWithSpeed(names, angles, speed)
            response.success = True
            response.message = "Joint movement executed"
            self.get_logger().info(" Movimento completato")
        except Exception as e:
            self.get_logger().error(f" Errore set_joint_angles: {e}")
            response.success = False
            response.message = str(e)
        
        return response



    #Callback per il service: SetPosture
    def set_posture(self, request, response):
        if self.posture is None:
            self.get_logger().warn(f"[MOCK] set_posture chiamato: {request.posture_name}")
            response.success = True
            response.message = "MOCK: Postura simulata"
            return response
        
        pose_name = request.posture_name
        speed = request.speed
        
        try:
            self.posture.goToPosture(pose_name, speed)
            response.success = True
            response.message = "Posture set correctly"
        except Exception as e:
            self.get_logger().error(f"Errore set_posture: {e}")
            response.success = False
            response.message = str(e)
        return response




    #Callback per il service: SetHand
    def set_hand(self, request, response):
        if self.motion is None:
            self.get_logger().warn(f"[MOCK] set_hand chiamato: {request.hand}, fun={request.fun}")
            response.success = True
            response.message = "MOCK: Hand movement simulato"
            return response
        
        hand = request.hand
        fun = request.fun
        
        try:
            if hand == "Hands":
                if fun == 0:
                    self.motion.openHand("RHand")
                    self.motion.openHand("LHand")
                    response.message = "Right and left hand opened"
                else:
                    self.motion.closeHand("RHand")
                    self.motion.closeHand("LHand")
                    response.message = "Right and left hand closed"
            else:
                if fun == 0:
                    self.motion.openHand(hand)
                    response.message = f"{hand} hand opened"
                else:
                    self.motion.closeHand(hand)
                    response.message = f"{hand} hand closed"
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Errore set_hand: {e}")
            response.success = False
            response.message = str(e)
        return response



    #Callback per il service: PlayAnimation
    def play_animation(self, request, response):
        """Service per eseguire animazioni predefinite"""
        
        if self.motion is None:
            self.get_logger().warn(f"[MOCK] Animation '{request.animation_type}' simulata")
            response.success = True
            response.message = f"MOCK: Animation {request.animation_type} eseguita"
            return response
        
        animation_type = request.animation_type
        
        try:
            if animation_type == "dancing":
                self._dancing(request.repetitions or 3)
            elif animation_type == "waving":
                self._waving(request.side or "right")
            elif animation_type == "nodding":
                self._nodding(request.repetitions or 3)
            elif animation_type == "shaking_head":
                self._shaking_head(request.repetitions or 3)
            elif animation_type == "celebrating":
                self._celebrating()
            elif animation_type == "pointing":
                self._pointing(request.direction or "front")
            elif animation_type == "shrugging":
                self._shrugging()
            elif animation_type == "bowing":
                self._bowing()
            elif animation_type == "thinking":
                self._thinking(request.duration or 3.0)
            elif animation_type == "clapping":
                self._clapping(request.repetitions or 5)
            else:
                response.success = False
                response.message = f"Animazione '{animation_type}' non supportata"
                return response
            
            response.success = True
            response.message = f"Animazione {animation_type} completata"
        
        except Exception as e:
            self.get_logger().error(f"Errore animazione: {e}")
            response.success = False
            response.message = str(e)
        
        return response



    # ========== METODI PRIVATI PER ANIMAZIONI ==========
    
    def _dancing(self, cycles):
        """Pepper balla"""
        for i in range(cycles):
            self.motion.angleInterpolationWithSpeed(
                ["HipRoll", "LShoulderPitch", "RShoulderPitch"],
                [-0.3, -1.0, -1.0], 0.5
            )
            time.sleep(0.4)
            self.motion.angleInterpolationWithSpeed(
                ["HipRoll", "LShoulderPitch", "RShoulderPitch"],
                [0.3, -1.0, -1.0], 0.5
            )
            time.sleep(0.4)
        self.posture.goToPosture("Stand", 0.5)


    def _waving(self, side):
        """Pepper saluta"""
        if side == "right":
            shoulder, elbow, wrist = "RShoulderPitch", "RElbowRoll", "RWristYaw"
            elbow_value = 1.0
        else:
            shoulder, elbow, wrist = "LShoulderPitch", "LElbowRoll", "LWristYaw"
            elbow_value = -1.0
        
        self.motion.angleInterpolationWithSpeed([shoulder, elbow], [-1.5, elbow_value], 0.5)
        for _ in range(3):
            self.motion.angleInterpolationWithSpeed([wrist], [0.5], 0.3)
            time.sleep(0.2)
            self.motion.angleInterpolationWithSpeed([wrist], [-0.5], 0.3)
            time.sleep(0.2)
        self.posture.goToPosture("Stand", 0.5)


    def _nodding(self, times):
        """Pepper annuisce"""
        for _ in range(times):
            self.motion.angleInterpolationWithSpeed(["HeadPitch"], [0.4], 0.5)
            time.sleep(0.3)
            self.motion.angleInterpolationWithSpeed(["HeadPitch"], [-0.2], 0.5)
            time.sleep(0.3)
        self.motion.angleInterpolationWithSpeed(["HeadPitch"], [0.0], 0.3)


    def _shaking_head(self, times):
        """Pepper scuote la testa"""
        for _ in range(times):
            self.motion.angleInterpolationWithSpeed(["HeadYaw"], [0.6], 0.4)
            time.sleep(0.2)
            self.motion.angleInterpolationWithSpeed(["HeadYaw"], [-0.6], 0.4)
            time.sleep(0.2)
        self.motion.angleInterpolationWithSpeed(["HeadYaw"], [0.0], 0.3)


    def _celebrating(self):
        """Pepper celebra"""
        self.motion.angleInterpolationWithSpeed(
            ["LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll"],
            [-1.5, -1.5, 0.3, -0.3], 0.7
        )
        time.sleep(1.5)
        self.posture.goToPosture("Stand", 0.5)


    def _pointing(self, direction):
        """Pepper indica una direzione"""
        if direction == "right":
            self.motion.angleInterpolationWithSpeed(
                ["RShoulderPitch", "RShoulderRoll", "RElbowRoll"],
                [0.0, -0.3, 0.0], 0.5
            )
        elif direction == "left":
            self.motion.angleInterpolationWithSpeed(
                ["LShoulderPitch", "LShoulderRoll", "LElbowRoll"],
                [0.0, 0.3, 0.0], 0.5
            )
        else:
            self.motion.angleInterpolationWithSpeed(
                ["RShoulderPitch", "RElbowRoll"],
                [-0.5, 0.0], 0.5
            )
        time.sleep(2.0)
        self.posture.goToPosture("Stand", 0.5)


    def _shrugging(self):
        """Pepper alza le spalle"""
        self.motion.angleInterpolationWithSpeed(
            ["LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll"],
            [-0.5, -0.5, 0.5, -0.5], 0.5
        )
        self.motion.angleInterpolationWithSpeed(["HeadPitch"], [0.2], 0.3)
        time.sleep(1.5)
        self.posture.goToPosture("Stand", 0.5)


    def _bowing(self):
        """Pepper fa un inchino"""
        self.motion.angleInterpolationWithSpeed(
            ["HeadPitch", "HipPitch"],
            [0.5, -0.5], 0.6
        )
        time.sleep(1.0)
        self.motion.angleInterpolationWithSpeed(
            ["HeadPitch", "HipPitch"],
            [0.0, 0.0], 0.6
        )


    def _thinking(self, duration):
        """Pepper assume posizione pensierosa"""
        self.motion.angleInterpolationWithSpeed(
            ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"],
            [-0.5, -0.1, 1.0, 1.5], 0.5
        )
        self.motion.angleInterpolationWithSpeed(["HeadPitch", "HeadYaw"], [0.2, 0.3], 0.3)
        time.sleep(duration)
        self.posture.goToPosture("Stand", 0.5)


    def _clapping(self, times):
        """Pepper applaude"""
        self.motion.angleInterpolationWithSpeed(
            ["LShoulderPitch", "RShoulderPitch", "LElbowRoll", "RElbowRoll"],
            [0.5, 0.5, -1.5, 1.5], 0.5
        )
        for _ in range(times):
            self.motion.openHand("LHand")
            self.motion.openHand("RHand")
            time.sleep(0.15)
            self.motion.closeHand("LHand")
            self.motion.closeHand("RHand")
            time.sleep(0.15)
        self.posture.goToPosture("Stand", 0.5)




    #Riferito all'action: Walking
    def walking(self, goal_handle):
        x = goal_handle.request.x
        y = goal_handle.request.y
        theta = goal_handle.request.theta
        
        self.get_logger().info(f"Walking to ({x}, {y}, {theta})")



        # Feedback iniziale
        walking_feedback = Walking.Feedback()
        walking_feedback.current_status = "Walking..."
        goal_handle.publish_feedback(walking_feedback)



        if self.motion is None:
            self.get_logger().warn(f"[MOCK] Walking simulato: x={x}, y={y}, theta={theta}")
            # Simula un po' di feedback
            walking_feedback.current_status = "MOCK: Walking in progress..."
            goal_handle.publish_feedback(walking_feedback)
            
            # Risultato mock
            goal_handle.succeed()
            result = Walking.Result()
            result.success = True
            walking_feedback.current_status = "MOCK: Walking completato"
            return result



        # Esecuzione reale
        try:
            self.motion.moveTo(x, y, theta)
            
            # Quando il movimento è terminato:
            goal_handle.succeed()
            result = Walking.Result()
            result.success = True
            walking_feedback.current_status = "Walking completato"
            return result
        except Exception as e:
            self.get_logger().error(f"Errore walking: {e}")
            goal_handle.abort()
            result = Walking.Result()
            result.success = False
            walking_feedback.current_status = str(e)
            return result
        




    # Riferito all'action: Navigating
    # ATTENZIONE: NAVIGATING FUNZIONA FINO AD UNA DISTANZA MASSIMA DI 3 METRI valutare una funzione wrap di loop per dist maggiori
    def navigating(self, goal_handle): 
        """Action callback per navigazione autonoma con obstacle avoidance"""
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        
        # Parametri opzionali (default se non specificati)
        use_map = getattr(goal_handle.request, 'use_map', False)
        map_name = getattr(goal_handle.request, 'map_name', 'default')
        
        self.get_logger().info(f"Navigating to ({target_x}, {target_y}), use_map={use_map}")



        # Feedback iniziale
        feedback = Navigating.Feedback()
        feedback.status = "planning"
        feedback.current_x = 0.0
        feedback.current_y = 0.0
        feedback.distance_remaining = 0.0
        goal_handle.publish_feedback(feedback)



        # ===== MOCK MODE =====
        if self.motion is None or self.navigation is None:
            self.get_logger().warn(f"[MOCK] Navigating simulato")
            
            feedback.status = "moving"
            feedback.current_x = target_x * 0.5
            feedback.current_y = target_y * 0.5
            feedback.distance_remaining = math.sqrt((target_x - feedback.current_x)**2 + 
                                                    (target_y - feedback.current_y)**2)
            goal_handle.publish_feedback(feedback)
            
            goal_handle.succeed()
            result = Navigating.Result()
            result.success = True
            result.status = "reached"
            result.final_x = target_x
            result.final_y = target_y
            result.distance_to_target = 0.0
            return result



        # ===== ESECUZIONE REALE =====
        try:
            # GESTIONE MAPPING (inline, se richiesto)
            if use_map:
                map_path = f"/home/nao/maps/{map_name}.explo"
                
                # Carica mappa solo se diversa da quella corrente
                if not (self._localization_active and self._current_map_name == map_name):
                    self.get_logger().info(f"Caricamento mappa: {map_path}")
                    
                    feedback.status = "loading_map"
                    goal_handle.publish_feedback(feedback)
                    
                    # Ferma localization precedente
                    if self._localization_active:
                        self.navigation.stopLocalization()
                    
                    # Carica e relocalizza
                    try:
                        self.navigation.loadExploration(map_path)
                        self.navigation.relocalizeInMap([0.0, 0.0])
                        self.navigation.startLocalization()
                        
                        self._localization_active = True
                        self._current_map_name = map_name
                        self.get_logger().info(f"Mappa '{map_name}' caricata")
                        
                    except Exception as e:
                        self.get_logger().error(f"Errore caricamento mappa: {e}")
                        goal_handle.abort()
                        
                        result = Navigating.Result()
                        result.success = False
                        result.status = "map_not_found"
                        result.final_x = 0.0
                        result.final_y = 0.0
                        result.distance_to_target = 0.0
                        return result
            
            # Ottieni posizione iniziale
            if use_map:
                start_pose = self.navigation.getRobotPositionInMap()
            else:
                start_pose = self.motion.getRobotPosition(False)
            
            start_x, start_y = start_pose[0], start_pose[1]
            initial_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
            
            self.get_logger().info(f"Start: ({start_x:.2f}, {start_y:.2f}), distanza: {initial_distance:.2f}m")
            
            # Warning per distanze > 3m in modalità relativa
            if not use_map and initial_distance > 3.0:
                self.get_logger().warn(f"Target oltre 3m ({initial_distance:.2f}m),consigliato uso di use_map=true")
            
            # Feedback moving
            feedback.status = "moving"
            feedback.current_x = start_x
            feedback.current_y = start_y
            feedback.distance_remaining = initial_distance
            goal_handle.publish_feedback(feedback)
            
            # NAVIGAZIONE (bloccante)
            if use_map:
                success = self.navigation.navigateToInMap([target_x, target_y, 0.0])
            else:
                success = self.navigation.navigateTo(target_x, target_y)
            
            # Posizione finale
            if use_map:
                final_pose = self.navigation.getRobotPositionInMap()
            else:
                final_pose = self.motion.getRobotPosition(False)
            
            final_x, final_y = final_pose[0], final_pose[1]
            distance_to_target = math.sqrt((target_x - final_x)**2 + (target_y - final_y)**2)
            
            # Risultato
            result = Navigating.Result()
            result.success = success
            result.final_x = final_x
            result.final_y = final_y
            result.distance_to_target = distance_to_target
            
            if success:
                result.status = "reached"
                self.get_logger().info(f"Target raggiunto! Final: ({final_x:.2f}, {final_y:.2f})")
                goal_handle.succeed()
            else:
                result.status = "blocked"
                self.get_logger().warn(f"Bloccato. Final: ({final_x:.2f}, {final_y:.2f})")
                goal_handle.abort()
            
            return result




            
        except Exception as e:
            self.get_logger().error(f"Errore: {e}")
            goal_handle.abort()
            
            result = Navigating.Result()
            result.success = False
            result.status = "error"
            
            try:
                if use_map:
                    error_pose = self.navigation.getRobotPositionInMap()
                else:
                    error_pose = self.motion.getRobotPosition(False)
                
                result.final_x = error_pose[0]
                result.final_y = error_pose[1]
                result.distance_to_target = math.sqrt((target_x - error_pose[0])**2 + 
                                                    (target_y - error_pose[1])**2)
            except:
                result.final_x = 0.0
                result.final_y = 0.0
                result.distance_to_target = 0.0
            
            return result





    # ========== SEEKING ACTION ==========
    # Aggregazione di Tracking + Navigazione continua con obstacle avoidance per seguire un target
    def seeking(self, goal_handle):
        """
        Action callback per SEEKING: segue un target utilizzando ALTracker + ALNavigation
        
        Goal:
            - target_type: "Face", "People", "LandMark"
            - approach_distance: distanza a cui fermarsi dal target (default: 1.0m)
        
        Feedback:
            - status_message: stato attuale ("searching", "found", "following", "approaching")
            - distance_remaining: distanza corrente dal target
            - current_x, current_y: coordinate attuali del target
        
        Result:
            - success: True se target raggiunto, False altrimenti
            - status: "reached", "lost", "timeout", "blocked"
            - final_distance: distanza finale dal target
        """
        
        target_type = goal_handle.request.target_type
        approach_distance = goal_handle.request.approach_distance if goal_handle.request.approach_distance > 0 else 1.0
        
        self.get_logger().info(f"Seeking started: target_type={target_type}, approach_distance={approach_distance}m")
        
        # Feedback iniziale
        feedback = Seeking.Feedback()
        feedback.status_message = "searching"
        feedback.distance_remaining = 0.0
        feedback.current_x = 0.0
        feedback.current_y = 0.0
        goal_handle.publish_feedback(feedback)
        
        # ===== MOCK MODE =====
        if self.motion is None or self.tracker is None or self.navigation is None:
            self.get_logger().warn("[MOCK] Seeking simulato")
            
            feedback.status_message = "MOCK: found"
            feedback.distance_remaining = 0.5
            feedback.current_x = 1.0
            feedback.current_y = 0.5
            goal_handle.publish_feedback(feedback)
            
            goal_handle.succeed()
            result = Seeking.Result()
            result.success = True
            result.status = "reached"
            result.final_distance = 0.0
            return result
        
        # ===== ESECUZIONE REALE =====
        try:
            # Step 1: Setup postura
            self.get_logger().info("Setting up posture...")
            try:
                self.posture.goToPosture("Stand", 0.5)
            except:
                pass
            
            time.sleep(1.0)
            
            # Step 2: Avvia tracking
            self.get_logger().info(f"Starting tracking for {target_type}...")
            try:
                # Registra target
                if target_type == "Face":
                    self.tracker.registerTarget("Face", 0.15)
                elif target_type == "People":
                    self.tracker.registerTarget("People", 0.3)
                elif target_type == "LandMark":
                    self.tracker.registerTarget("LandMark", 0.08)
                else:
                    self.get_logger().error(f"Target type {target_type} non supportato")
                    goal_handle.abort()
                    result = Seeking.Result()
                    result.success = False
                    result.status = "invalid_target"
                    result.final_distance = 0.0
                    return result
                
                # Attiva tracking - SOLO TESTA (non "Move")
                # Perché il corpo lo muove navigating
                self.tracker.setMode("Head")
                self.tracker.track(target_type)
                
            except Exception as e:
                self.get_logger().error(f"Errore avvio tracking: {e}")
                goal_handle.abort()
                result = Seeking.Result()
                result.success = False
                result.status = "tracking_error"
                result.final_distance = 0.0
                return result
            
            self._tracking_active = True
            
            # Step 3: Loop principale di seeking con navigating
            timeout_counter = 0
            max_timeout = 20  # 20 secondi di perdita tracking (1s per loop)
            loop_count = 0
            max_loops = 120  # Timeout massimo 2 minuti
            
            while loop_count < max_loops:
                loop_count += 1
                
                # Ottieni posizione target
                target_position = self.get_tracked_position()
                
                if target_position is None:
                    # Prova service call come backup
                    target_position = self.request_tracked_object_coordinates()
                
                if target_position is None:
                    # Tracking perso
                    timeout_counter += 1
                    
                    feedback.status_message = "lost"
                    feedback.distance_remaining = -1.0
                    goal_handle.publish_feedback(feedback)
                    
                    self.get_logger().warn(f"Tracking lost ({timeout_counter}/{max_timeout})")
                    
                    if timeout_counter >= max_timeout:
                        self.get_logger().error("Target perso definitivamente")
                        goal_handle.abort()
                        
                        result = Seeking.Result()
                        result.success = False
                        result.status = "lost"
                        result.final_distance = -1.0
                        return result
                    
                    time.sleep(1.0)
                    continue
                
                # Target trovato
                timeout_counter = 0
                
                x = target_position['x']
                y = target_position['y']
                z = target_position.get('z', 0.0)
                distance = math.sqrt(x**2 + y**2 + z**2)
                
                feedback.current_x = x
                feedback.current_y = y
                feedback.distance_remaining = distance
                
                # Verifica se raggiunto
                if distance < approach_distance:
                    feedback.status_message = "reached"
                    goal_handle.publish_feedback(feedback)
                    
                    self.get_logger().info(f"Target raggiunto! Distance: {distance:.2f}m")
                    
                    # Stop tracking
                    self._tracking_active = False
                    try:
                        self.tracker.stopTracker()
                        self.tracker.unregisterAllTargets()
                    except:
                        pass
                    
                    # Successo
                    goal_handle.succeed()
                    result = Seeking.Result()
                    result.success = True
                    result.status = "reached"
                    result.final_distance = distance
                    return result
                
                # Naviga verso target con obstacle avoidance
                feedback.status_message = "approaching"
                goal_handle.publish_feedback(feedback)
                
                # Calcola target intermedio (non tutto il percorso)
                # Cosi possiamo aggiornare piu spesso
                max_step = 0.5  # Max 50cm per step
                
                if distance > max_step:
                    # Normalizza e scala a max_step
                    factor = max_step / distance
                    nav_x = x * factor
                    nav_y = y * factor
                else:
                    # Se vicino, vai diretto
                    nav_x = x
                    nav_y = y
                
                # Usa navigating con coordinate relative
                self.get_logger().info(f"Navigating step: distance={distance:.2f}m, target=({nav_x:.2f}, {nav_y:.2f})")
                
                try:
                    # navigateTo e bloccante MA con obstacle avoidance
                    success = self.navigation.navigateTo(nav_x, nav_y)
                    
                    if not success:
                        # Bloccato da ostacolo
                        self.get_logger().warn("Navigazione bloccata da ostacolo")
                        
                        # Prova a girare attorno
                        time.sleep(0.5)
                        
                        # Riprova nella prossima iterazione con nuove coordinate
                        continue
                        
                except Exception as e:
                    self.get_logger().error(f"Errore navigazione: {e}")
                    goal_handle.abort()
                    
                    result = Seeking.Result()
                    result.success = False
                    result.status = "blocked"
                    result.final_distance = distance
                    return result
                
                # Piccola pausa prima di aggiornare target
                # (navigateTo e bloccante, quindi arriviamo qui solo quando finito)
                time.sleep(0.2)
            
            # Timeout massimo raggiunto
            self.get_logger().error("Timeout massimo seeking raggiunto")
            goal_handle.abort()
            
            result = Seeking.Result()
            result.success = False
            result.status = "timeout"
            result.final_distance = -1.0
            return result
        
        except Exception as e:
            self.get_logger().error(f"Errore seeking: {e}")
            goal_handle.abort()
            
            result = Seeking.Result()
            result.success = False
            result.status = "error"
            result.final_distance = -1.0
            return result
        
        finally:
            # Cleanup
            self._tracking_active = False
            try:
                self.tracker.stopTracker()
                self.tracker.unregisterAllTargets()
            except:
                pass




def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_movement()
    rclpy.spin(node)
    rclpy.shutdown()
    



if __name__ == '__main__':
    main()

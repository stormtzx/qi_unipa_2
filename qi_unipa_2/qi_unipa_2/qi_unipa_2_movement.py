import qi
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from qi_unipa_2.utils import Utils #type: ignore
from qi_unipa_2_interfaces.srv import SetState, SetPosture, SetJointAngles, SetHand, PlayAnimation #type: ignore
from qi_unipa_2_interfaces.action import Moving, Walking, Navigating #type: ignore
from qi_unipa_2_interfaces.msg import Sonar
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



        # Creazione Utils
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)





        # Variabili sonar (AGGIUNTO)
        self.front_sonar = 0.0
        self.back_sonar = 0.0




        # Crea Utils
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        qos_best_effort_5 = utils.get_QoS('best_effort', 5)




        # Connessione condizionale
        if mock_mode:
            self.get_logger().warn("Nodo MOVEMENT attivo in MOCK MODE")
            self.session = None
            self.motion = None
            self.posture = None
            self.navigation = None
            self.animation_player = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                self.navigation = self.session.service("ALNavigation")
                self.motion = self.session.service("ALMotion")
                self.posture = self.session.service("ALRobotPosture")
                self.animation_player = self.session.service("ALAnimationPlayer")
                self.get_logger().info("Nodo MOVEMENT attivo e connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e





        # Dichiarazione service
        self.create_service(SetState, '/pepper/services/set_state', self.set_state, qos_profile=qos_reliable_10)
        self.create_service(SetJointAngles, '/pepper/services/set_joint_angles', self.set_joint_angles, qos_profile=qos_reliable_10)
        self.create_service(SetPosture, '/pepper/services/set_posture', self.set_posture, qos_profile=qos_reliable_10)
        self.create_service(SetHand, '/pepper/services/set_hand', self.set_hand, qos_profile=qos_reliable_10)
        self.create_service(PlayAnimation, '/pepper/services/play_animation', self.play_animation, qos_profile=qos_reliable_10)
        
        # Dichiarazione action
        self._action_server_moving = ActionServer(self, Moving, '/pepper/actions/moving', self.moving)
        # ActionServer Navigating (NAVIGAZIONE AUTONOMA CON OBSTACLE AVOIDANCE)
        self.navigating_server = ActionServer(self, Navigating, '/pepper/actions/navigating', self.navigating)
        # ActionServer Walking (ossia moving+obstacle avoidance semplificato)
        self.walking_server = ActionServer(self, Walking, '/pepper/actions/walking', self.walking)

        # Subscription a sonar data (CORRETTO: era self.node.create_subscription)
        self.sonar_sub = self.create_subscription(
            Sonar,
            '/pepper/topics/sonar',
            self.sonar_handler,
            qos_best_effort_10
        )






    # ========== CALLBACKS SONAR DATA (AGGIUNTO) ==========
    def sonar_handler(self, msg):
        """Callback per dati sonar dal nodo reference"""
        self.front_sonar = msg.front_sonar
        self.back_sonar = msg.back_sonar







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
            "HeadYaw", "HeadPitch","HipRoll", "HipPitch", "KneePitch",
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
            "LHand", "RHand",
            
        ]
        
        for name in names:
            if name.strip() not in valid_joints:
                self.get_logger().error(f"Articolazione non valida: {name}")
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
        """
        Service per eseguire animazioni NATIVE di Pepper.
        Solo set essenziale. Se animazione non disponibile, ritorna errore.
        """
        
        # ===== MOCK MODE: motion o animation_player non disponibili =====
        if self.motion is None or self.animation_player is None:
            mock_reason = "motion" if self.motion is None else "animation_player"
            self.get_logger().warn(
                f"[MOCK] play_animation chiamato: {request.animation_type} "
                f"({mock_reason} non disponibile)"
            )
            response.success = True
            response.message = f"MOCK: Animation {request.animation_type} simulata"
            return response
        
        animation_type = request.animation_type.lower()
        
        # ========== SET MINIMO VITALE (13 animazioni) ==========
        native_animations = {
            # Saluti base
            "wave": "animations/Stand/Gestures/Hey_1",
            "hello": "animations/Stand/Gestures/Hello_1",
            
            # Conferme/Negazioni
            "yes": "animations/Stand/Gestures/Yes_1",
            "nod": "animations/Stand/Gestures/Yes_1",
            "no": "animations/Stand/Gestures/No_1",
            
            # Emozioni base
            "happy": "animations/Stand/Gestures/Happy_1",
            "thinking": "animations/Stand/Gestures/Think_1",
            
            # Comunicazione
            "explain": "animations/Stand/Gestures/Explain_1",
            "shrug": "animations/Stand/Gestures/IDontKnow_1",
            
            # Cortesia
            "bow": "animations/Stand/Gestures/BowShort_1",
            
            # Indicare avanti (generico dritto)
            "point": "animations/Stand/Gestures/ShowTablet_1",
            
            # Ballo e Festeggiamento
            "dance": "animations/Stand/Gestures/Enthusiastic_5",
            "celebrate": "animations/Stand/Emotions/Positive/Excited_1",
        }
        
        # Verifica se animazione supportata
        if animation_type not in native_animations:
            available = ", ".join(sorted(native_animations.keys()))
            self.get_logger().error(
                f" Animazione '{request.animation_type}' non supportata.\n"
                f"   Disponibili: {available}"
            )
            response.success = False
            response.message = (
                f"Animazione '{request.animation_type}' non supportata. "
                f"Disponibili: {available}"
            )
            return response
        
        # Esegui animazione (arrivati qui motion e animation_player sono disponibili)
        animation_path = native_animations[animation_type]
        
        try:
            self.get_logger().info(f" Esecuzione: {animation_path}")
            
            # Esegui (bloccante)
            self.animation_player.run(animation_path, _async=False)
            
            response.success = True
            response.message = f"Animazione '{request.animation_type}' completata"
            self.get_logger().info(f" '{request.animation_type}' completata")
            
        except Exception as e:
            self.get_logger().error(f" Errore animazione '{animation_path}': {e}")
            response.success = False
            response.message = f"Errore esecuzione: {str(e)}"
        
        return response


    #Riferito all'action: moving
    def moving(self, goal_handle):
        x = goal_handle.request.x
        y = goal_handle.request.y
        theta = goal_handle.request.theta
        
        self.get_logger().info(f"moving to ({x}, {y}, {theta})")




        # Feedback iniziale
        moving_feedback = Moving.Feedback()
        moving_feedback.current_status = "moving..."
        goal_handle.publish_feedback(moving_feedback)




        if self.motion is None:
            self.get_logger().warn(f"[MOCK] Moving simulato: x={x}, y={y}, theta={theta}")
            # Simula un po' di feedback
            moving_feedback.current_status = "MOCK: moving in progress..."
            goal_handle.publish_feedback(moving_feedback)
            
            # Risultato mock
            goal_handle.succeed()
            result = Moving.Result()
            result.success = True
            moving_feedback.current_status = "MOCK: moving completato"
            return result




        # Esecuzione reale
        try:
            self.motion.moveTo(x, y, theta)
            
            # Quando il movimento è terminato:
            goal_handle.succeed()
            result = Moving.Result()
            result.success = True
            moving_feedback.current_status = "Moving completato"
            return result
        except Exception as e:
            self.get_logger().error(f"Errore moving: {e}")
            goal_handle.abort()
            result = Moving.Result()
            result.success = False
            moving_feedback.current_status = str(e)
            return result
        


    # ========== ACTION: WALKING (CON OBSTACLE AVOIDANCE) ==========
    def walking(self, goal_handle):
        """
        Action Walking - moveTo incrementale con obstacle avoidance via sonar
        
        Parametri:
        - Soglia sonar: 0.10m (10cm) hardcoded
        - Step forward: 0.25m
        - Step recovery: 0.20m (alternanza destra/sinistra)
        - Timeout: 300 sec (5 minuti)
        - Feedback: ogni 1 secondo
        """
        
        # ===== PARAMETRI GOAL =====
        x = goal_handle.request.x
        y = goal_handle.request.y
        theta = goal_handle.request.theta
        
        # ===== COSTANTI HARDCODED =====
        SONAR_THRESHOLD = 0.10  # 10cm
        FORWARD_STEP = 0.50     # 50cm per step
        RECOVERY_STEP = 0.20    # 20cm per evasione
        MAX_TIMEOUT = 300.0     # 5 minuti
        FEEDBACK_INTERVAL = 1.0 # 1 secondo
        MAX_RECOVERY_ATTEMPTS = 20  # Max tentativi evasione prima di abort
        
        # ===== VARIABILI DI STATO =====
        start_time = time.time()
        last_feedback_time = start_time
        
        distance_covered = 0.0
        obstacles_detected = 0
        recovery_attempts = 0
        
        # Calcola distanza target totale
        total_distance = math.sqrt(x**2 + y**2)

        if total_distance < 0.01:
            self.get_logger().error("Walking: distanza target troppo piccola (<1cm), aborting")
            goal_handle.abort()
            result = Walking.Result()
            result.success = False
            result.reason = "target_too_close"
            return result
        
        self.get_logger().info(
            f"╔═══════════════════════════════════════════════════════════════╗\n"
            f"║ WALKING START                                                 ║\n"
            f"╠═══════════════════════════════════════════════════════════════╣\n"
            f"║ Target: x={x:.2f}m, y={y:.2f}m, theta={theta:.3f}rad          ║\n"
            f"║ Total distance: {total_distance:.2f}m                         ║\n"
            f"║ Sonar threshold: {SONAR_THRESHOLD}m                           ║\n"
            f"║ Forward step: {FORWARD_STEP}m                                 ║\n"
            f"║ Recovery step: {RECOVERY_STEP}m                               ║\n"
            f"║ Timeout: {MAX_TIMEOUT}s                                       ║\n"
            f"╚═══════════════════════════════════════════════════════════════╝"
        )
        
        # ===== FEEDBACK INIZIALE =====
        feedback = Walking.Feedback()
        feedback.current_status = "initializing"
        feedback.distance_covered = 0.0
        feedback.remaining_distance = total_distance
        feedback.front_sonar_distance = self.front_sonar if self.motion is not None else 2.5
        feedback.obstacles_detected = 0
        feedback.recovery_attempts = 0
        feedback.elapsed_time = 0.0
        goal_handle.publish_feedback(feedback)
        
        # ===== MOCK MODE =====
        if self.motion is None:
            self.get_logger().warn(
                f"[MOCK] Walking simulato: x={x:.2f}m, y={y:.2f}m, theta={theta:.3f}rad\n"
                f"       Total distance: {total_distance:.2f}m"
            )
            
            # Simula rotazione iniziale se richiesta
            if abs(theta) > 0.05:
                feedback.current_status = "rotating"
                feedback.elapsed_time = 0.5
                goal_handle.publish_feedback(feedback)
                time.sleep(0.5)
            
            # Simula movimento progressivo
            mock_steps = max(3, int(total_distance / FORWARD_STEP))
            for step in range(mock_steps):
                # Calcola progresso
                progress = (step + 1) / mock_steps
                mock_distance = total_distance * progress
                
                feedback.current_status = "moving_forward"
                feedback.distance_covered = mock_distance
                feedback.remaining_distance = total_distance - mock_distance
                feedback.front_sonar_distance = 2.5  # Mock sonar sempre libero
                feedback.obstacles_detected = 0
                feedback.recovery_attempts = 0
                feedback.elapsed_time = time.time() - start_time
                goal_handle.publish_feedback(feedback)
                
                time.sleep(0.3)  # Simula durata step
            
            # Completamento mock
            feedback.current_status = "completed"
            feedback.distance_covered = total_distance
            feedback.remaining_distance = 0.0
            feedback.front_sonar_distance = 2.5
            feedback.obstacles_detected = 0
            feedback.recovery_attempts = 0
            feedback.elapsed_time = time.time() - start_time
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(
                f" [MOCK] Walking completato!\n"
                f"       Distanza: {total_distance:.2f}m, Tempo: {feedback.elapsed_time:.1f}s"
            )
            
            goal_handle.succeed()
            result = Walking.Result()
            result.success = True
            result.reason = "reached"
            result.distance_covered = total_distance
            result.obstacles_detected = 0
            result.recovery_attempts = 0
            return result
        
        # ===== STEP 1: ROTAZIONE INIZIALE (se richiesta) =====
        if abs(theta) > 0.05:  # Solo se rotazione > ~3°
            feedback.current_status = "rotating"
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f"⟲ Rotazione iniziale: {theta:.3f} rad ({math.degrees(theta):.1f}°)")
            
            try:
                self.motion.moveTo(0.0, 0.0, theta)
                time.sleep(0.5)  # Stabilizzazione
            except Exception as e:
                self.get_logger().error(f" Errore rotazione: {e}")
                goal_handle.abort()
                result = Walking.Result()
                result.success = False
                result.reason = "error"
                result.distance_covered = 0.0
                result.obstacles_detected = obstacles_detected
                result.recovery_attempts = recovery_attempts
                return result
        
        # ===== STEP 2: MOVIMENTO INCREMENTALE CON OBSTACLE AVOIDANCE =====
        try:
            while distance_covered < total_distance:
                
                # ===== CHECK TIMEOUT =====
                elapsed_time = time.time() - start_time
                if elapsed_time > MAX_TIMEOUT:
                    self.get_logger().warn(f"⏱️ TIMEOUT raggiunto ({MAX_TIMEOUT}s)")
                    feedback.current_status = "timeout"
                    goal_handle.publish_feedback(feedback)
                    
                    goal_handle.abort()
                    result = Walking.Result()
                    result.success = False
                    result.reason = "timeout"
                    result.distance_covered = distance_covered
                    result.obstacles_detected = obstacles_detected
                    result.recovery_attempts = recovery_attempts
                    return result
                
                # ===== PUBLISH FEEDBACK (ogni secondo) =====
                if time.time() - last_feedback_time >= FEEDBACK_INTERVAL:
                    feedback.current_status = "moving_forward"
                    feedback.distance_covered = distance_covered
                    feedback.remaining_distance = max(0.0, total_distance - distance_covered)
                    feedback.front_sonar_distance = self.front_sonar
                    feedback.obstacles_detected = obstacles_detected
                    feedback.recovery_attempts = recovery_attempts
                    feedback.elapsed_time = elapsed_time
                    goal_handle.publish_feedback(feedback)
                    last_feedback_time = time.time()
                
                # ===== CHECK SONAR =====
                current_sonar = self.front_sonar
                
                if current_sonar < SONAR_THRESHOLD:
                    # ===== OSTACOLO RILEVATO =====
                    obstacles_detected += 1
                    
                    self.get_logger().warn(
                        f"️  OSTACOLO RILEVATO!\n"
                        f"    Sonar: {current_sonar:.3f}m < {SONAR_THRESHOLD}m\n"
                        f"    Tentativo evasione #{recovery_attempts + 1}"
                    )
                    
                    feedback.current_status = "obstacle_detected"
                    feedback.front_sonar_distance = current_sonar
                    feedback.obstacles_detected = obstacles_detected
                    goal_handle.publish_feedback(feedback)
                    
                    # ===== LOOP RECOVERY (alternanza destra/sinistra) =====
                    recovery_success = False
                    local_recovery_attempts = 0
                    
                    while not recovery_success and local_recovery_attempts < 10:
                        
                        # Check se troppi tentativi totali
                        if recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                            self.get_logger().error(
                                f" Max recovery attempts raggiunto ({MAX_RECOVERY_ATTEMPTS})"
                            )
                            feedback.current_status = "obstacle_blocked"
                            goal_handle.publish_feedback(feedback)
                            
                            goal_handle.abort()
                            result = Walking.Result()
                            result.success = False
                            result.reason = "obstacle_blocked"
                            result.distance_covered = distance_covered
                            result.obstacles_detected = obstacles_detected
                            result.recovery_attempts = recovery_attempts
                            return result
                        
                        # DETERMINA DIREZIONE (alternanza)
                        if recovery_attempts % 2 == 0:
                            # DESTRA
                            recovery_y = -RECOVERY_STEP
                            direction = "DESTRA"
                        else:
                            # SINISTRA
                            recovery_y = +RECOVERY_STEP
                            direction = "SINISTRA"
                        
                        self.get_logger().info(
                            f"↔️  Evasione {direction}: spostamento laterale di {abs(recovery_y):.2f}m"
                        )
                        
                        feedback.current_status = "evading_obstacle"
                        feedback.recovery_attempts = recovery_attempts
                        goal_handle.publish_feedback(feedback)
                        
                        # ESEGUI SPOSTAMENTO LATERALE
                        try:
                            self.motion.moveTo(0.0, recovery_y, 0.0)
                            time.sleep(0.5)  # Stabilizzazione
                            recovery_attempts += 1
                            local_recovery_attempts += 1
                        except Exception as e:
                            self.get_logger().error(f" Errore evasione: {e}")
                            goal_handle.abort()
                            result = Walking.Result()
                            result.success = False
                            result.reason = "error"
                            result.distance_covered = distance_covered
                            result.obstacles_detected = obstacles_detected
                            result.recovery_attempts = recovery_attempts
                            return result
                        
                        # RICONTROLLA SONAR
                        time.sleep(0.3)  # Attendi lettura sonar aggiornata
                        new_sonar = self.front_sonar
                        
                        self.get_logger().info(
                            f"   Sonar dopo evasione: {new_sonar:.3f}m "
                            f"({' LIBERO' if new_sonar >= SONAR_THRESHOLD else ' BLOCCATO'})"
                        )
                        
                        if new_sonar >= SONAR_THRESHOLD:
                            #  PERCORSO LIBERO
                            recovery_success = True
                            feedback.current_status = "obstacle_cleared"
                            feedback.front_sonar_distance = new_sonar
                            goal_handle.publish_feedback(feedback)
                            
                            self.get_logger().info(
                                f" Ostacolo superato dopo {local_recovery_attempts} tentativi"
                            )
                            break
                        else:
                            #  ANCORA BLOCCATO - Riprova dall'altro lato
                            self.get_logger().warn(
                                f"   Ancora bloccato, riprovo dall'altro lato..."
                            )
                            time.sleep(0.2)
                    
                    # Se esaurito loop recovery senza successo
                    if not recovery_success:
                        self.get_logger().error(
                            f" Impossibile superare ostacolo dopo {local_recovery_attempts} tentativi"
                        )
                        feedback.current_status = "obstacle_blocked"
                        goal_handle.publish_feedback(feedback)
                        
                        goal_handle.abort()
                        result = Walking.Result()
                        result.success = False
                        result.reason = "obstacle_blocked"
                        result.distance_covered = distance_covered
                        result.obstacles_detected = obstacles_detected
                        result.recovery_attempts = recovery_attempts
                        return result
                
                else:
                    # ===== PERCORSO LIBERO - AVANZA =====
                    remaining = total_distance - distance_covered
                    step_distance = min(FORWARD_STEP, remaining)
                    
                    # Normalizza step proporzionale a x e y
                    if total_distance > 0:
                        step_x = (x / total_distance) * step_distance
                        step_y = (y / total_distance) * step_distance
                    else:
                        step_x = 0.0
                        step_y = 0.0
                    
                    self.get_logger().info(
                        f"→  Avanzando: {step_distance:.2f}m "
                        f"(x={step_x:.2f}, y={step_y:.2f}) | "
                        f"Sonar: {current_sonar:.2f}m"
                    )
                    
                    feedback.current_status = "moving_forward"
                    goal_handle.publish_feedback(feedback)
                    
                    # ESEGUI MOVIMENTO
                    try:
                        self.motion.moveTo(step_x, step_y, 0.0)
                        time.sleep(0.4)  # Stabilizzazione
                        distance_covered += step_distance
                    except Exception as e:
                        self.get_logger().error(f" Errore movimento: {e}")
                        goal_handle.abort()
                        result = Walking.Result()
                        result.success = False
                        result.reason = "error"
                        result.distance_covered = distance_covered
                        result.obstacles_detected = obstacles_detected
                        result.recovery_attempts = recovery_attempts
                        return result
                
                # Breve pausa prima del prossimo check
                time.sleep(0.1)
        
        except Exception as e:
            self.get_logger().error(f" Errore walking loop: {e}")
            goal_handle.abort()
            result = Walking.Result()
            result.success = False
            result.reason = "error"
            result.distance_covered = distance_covered
            result.obstacles_detected = obstacles_detected
            result.recovery_attempts = recovery_attempts
            return result
        
        # ===== COMPLETAMENTO =====
        elapsed_time = time.time() - start_time
        success = distance_covered >= (total_distance * 0.8)  # 80% = successo
        
        self.get_logger().info(
            f"╔═══════════════════════════════════════════════════════════════╗\n"
            f"║ WALKING COMPLETED                                             ║\n"
            f"╠═══════════════════════════════════════════════════════════════╣\n"
            f"║ Success: {success}                                            ║\n"
            f"║ Distance covered: {distance_covered:.2f}m / {total_distance:.2f}m ({(distance_covered/total_distance*100):.1f}%) ║\n"
            f"║ Obstacles detected: {obstacles_detected}                      ║\n"
            f"║ Recovery attempts: {recovery_attempts}                        ║\n"
            f"║ Elapsed time: {elapsed_time:.1f}s                             ║\n"
            f"╚═══════════════════════════════════════════════════════════════╝"
        )
        
        feedback.current_status = "completed" if success else "failed"
        feedback.distance_covered = distance_covered
        feedback.remaining_distance = 0.0
        feedback.front_sonar_distance = self.front_sonar
        feedback.obstacles_detected = obstacles_detected
        feedback.recovery_attempts = recovery_attempts
        feedback.elapsed_time = elapsed_time
        goal_handle.publish_feedback(feedback)
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        result = Walking.Result()
        result.success = success
        result.reason = "reached" if success else "incomplete"
        result.distance_covered = distance_covered
        result.obstacles_detected = obstacles_detected
        result.recovery_attempts = recovery_attempts
        
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






def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_movement()
    rclpy.spin(node)
    rclpy.shutdown()
    




if __name__ == '__main__':
    main()

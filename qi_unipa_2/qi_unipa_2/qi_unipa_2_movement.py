import qi
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.srv import SetState, SetPosture, SetJointAngles, SetHand
from qi_unipa_2_interfaces.action import Walking, Navigating
import math


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
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                self.navigation = self.session.service("ALNavigation")
                self.motion = self.session.service("ALMotion")
                self.posture = self.session.service("ALRobotPosture")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.motion = None
                self.posture = None
                self.navigation = None


        # Dichiarazione service
        self.create_service(SetState, '~/set_state', self.set_state, qos_profile=qos_reliable_10)
        self.create_service(SetJointAngles, '~/set_joint_angles', self.set_joint_angles, qos_profile=qos_reliable_10)
        self.create_service(SetPosture, '~/set_posture', self.set_posture, qos_profile=qos_reliable_10)
        self.create_service(SetHand, '~/set_hand', self.set_hand, qos_profile=qos_reliable_10)
        
        # Dichiarazione action
        self._action_server_walking = ActionServer(self, Walking, '~/walking', self.walking)
          # ActionServer Navigating (NAVIGAZIONE AUTONOMA CON OBSTACLE AVOIDANCE)
        self.navigating_server = ActionServer(self,Navigating,'navigating',self.navigating)



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
        angles = request.angles.tolist()
        speed = request.speed
        
        try:
            self.motion.angleInterpolationWithSpeed(names, angles, speed)
            response.success = True
            response.message = "Joint movement executed"
        except Exception as e:
            self.get_logger().error(f"Errore set_joint_angles: {e}")
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
            walking_feedback.current_status  = "Walking completato"
            return result
        except Exception as e:
            self.get_logger().error(f"Errore walking: {e}")
            goal_handle.abort()
            result = Walking.Result()
            result.success = False
            walking_feedback.current_status = str(e)
            return result

    # Riferito all'action: Navigating
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

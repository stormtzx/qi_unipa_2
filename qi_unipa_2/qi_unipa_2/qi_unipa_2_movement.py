import qi
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from qi_unipa_2.utils import Utils
from qi_unipa_2_interfaces.srv import SetState, SetPosture, SetJointAngles, SetHand, GetPosition
from qi_unipa_2_interfaces.action import Walking

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
                
                self.motion = self.session.service("ALMotion")
                self.posture = self.session.service("ALRobotPosture")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.motion = None
                self.posture = None

        # Dichiarazione service
        self.create_service(SetState, '~/set_state', self.set_state, qos_profile=qos_reliable_10)
        self.create_service(SetJointAngles, '~/set_joint_angles', self.set_joint_angles, qos_profile=qos_reliable_10)
        self.create_service(SetPosture, '~/set_posture', self.set_posture, qos_profile=qos_reliable_10)
        self.create_service(SetHand, '~/set_hand', self.set_hand, qos_profile=qos_reliable_10)
        self.create_service(GetPosition, '~/get_position', self.get_position, qos_profile=qos_reliable_10)
        
        # Dichiarazione action
        self._action_server_walking = ActionServer(self, Walking, '~/walking', self.walking)



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


    # Callback per il service: GetPosition
    def get_position(self, request, response):
        if self.motion is None:
            self.get_logger().warn("[MOCK] get_position chiamato")
            # Ritorna posizione fittizia
            response.x = 0.0
            response.y = 0.0
            response.theta = 0.0
            return response
        
        try:
            pose = self.motion.getRobotPosition(False)
            response.x = pose[0]
            response.y = pose[1]
            response.theta = pose[2]
        except Exception as e:
            self.get_logger().error(f"Errore get_position: {e}")
            response.x = 0.0
            response.y = 0.0
            response.theta = 0.0
        return response


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_movement()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

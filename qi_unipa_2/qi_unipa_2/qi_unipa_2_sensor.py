import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from qi_unipa_2_interfaces.msg import Bumper, HeadTouch, HandTouch, Battery
from qi_unipa_2_interfaces.action import Talking
from qi_unipa_2.utils import Utils


class QiUnipa2_sensor(Node):    
    def __init__(self):
        super().__init__('qi_unipa_2_sensor')

        self.declare_parameter('reaction_mode', 'Autonomous')
        self.reaction_mode = self.get_parameter('reaction_mode').get_parameter_value().string_value
        self.get_logger().info(f"Reaction mode impostato su: {self.reaction_mode}")

        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)

        self.mock_mode = mock_mode
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO")
            self.session = None
            self.memory = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")                
                self.memory = self.session.service("ALMemory")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.memory = None
                self.mock_mode = True

        self.bumper_pub = self.create_publisher(Bumper, "/pepper/topics/bumper", qos_best_effort_10)
        self.head_touch_pub = self.create_publisher(HeadTouch, "/pepper/topics/head_touch", qos_best_effort_10)
        self.hand_touch_pub = self.create_publisher(HandTouch, "/pepper/topics/hand_touch", qos_best_effort_10)
        self.battery_pub = self.create_publisher(Battery, "/pepper/topics/battery", qos_best_effort_10)
    
        # ========== SOGLIE DI CAMBIO ==========
        self.bumper_threshold = 0.05
        self.touch_threshold = 0.05
        self.battery_threshold = 0.5  # % di cambio
        
        # ========== FLAG PRIMA PUBBLICAZIONE ==========
        self.bumper_published = False
        self.head_published = False
        self.hand_published = False
        self.battery_published = False
        
        # ========== STATO PRECEDENTE PER RILEVARE TRANSIZIONI E CAMBI ==========
        self.prev_bumper_state = {"left": 0.0, "right": 0.0, "back": 0.0}
        self.prev_head_state = {"front": 0.0, "middle": 0.0, "rear": 0.0}
        self.prev_hand_state = {"left": 0.0, "right": 0.0}
        self.prev_battery_state = {"charge": 0.0, "current": 0.0, "temp": 0.0}

        # Timers per polling
        self.timer_sensors = self.create_timer(0.1, self.poll_sensors)  # 100ms = 10Hz
        self.timer_battery = self.create_timer(5.0, self.battery_sub)
        
        self.talking_client = ActionClient(self, Talking, '/pepper/actions/talking')


    def poll_sensors(self):
        """Legge i sensori ogni 100ms e pubblica solo se cambiano o è la prima volta"""
        if self.mock_mode or self.memory is None:
            self.publish_mock_state()
            return
        
        try:
            # ========== BUMPER ==========
            msg_bumper = Bumper()
            msg_bumper.left = float(self.memory.getData("Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value"))
            msg_bumper.right = float(self.memory.getData("Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value"))
            msg_bumper.back = float(self.memory.getData("Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value"))
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            bumper_should_publish = (
                not self.bumper_published or
                abs(self.prev_bumper_state["left"] - msg_bumper.left) > self.bumper_threshold or
                abs(self.prev_bumper_state["right"] - msg_bumper.right) > self.bumper_threshold or
                abs(self.prev_bumper_state["back"] - msg_bumper.back) > self.bumper_threshold
            )
            
            if bumper_should_publish:
                self.bumper_pub.publish(msg_bumper)
                self.bumper_published = True
                self.prev_bumper_state = {
                    "left": msg_bumper.left,
                    "right": msg_bumper.right,
                    "back": msg_bumper.back
                }
            
            #  REAZIONI IMMEDIATE (NON DIPENDONO DA SOGLIA) - Sempre controllate
            self.check_bumper_transition(msg_bumper)
            
            # ========== HEAD TOUCH ==========
            msg_head = HeadTouch()
            msg_head.front = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value"))
            msg_head.middle = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value"))
            msg_head.rear = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value"))
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            head_should_publish = (
                not self.head_published or
                abs(self.prev_head_state["front"] - msg_head.front) > self.touch_threshold or
                abs(self.prev_head_state["middle"] - msg_head.middle) > self.touch_threshold or
                abs(self.prev_head_state["rear"] - msg_head.rear) > self.touch_threshold
            )
            
            if head_should_publish:
                self.head_touch_pub.publish(msg_head)
                self.head_published = True
                self.prev_head_state = {
                    "front": msg_head.front,
                    "middle": msg_head.middle,
                    "rear": msg_head.rear
                }
            
            #  REAZIONI IMMEDIATE (NON DIPENDONO DA SOGLIA) - Sempre controllate
            self.check_head_touch_transition(msg_head)
            
            # ========== HAND TOUCH ==========
            msg_hand = HandTouch()
            msg_hand.left_hand = float(self.memory.getData("Device/SubDeviceList/LHand/Touch/Back/Sensor/Value"))
            msg_hand.right_hand = float(self.memory.getData("Device/SubDeviceList/RHand/Touch/Back/Sensor/Value"))
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            hand_should_publish = (
                not self.hand_published or
                abs(self.prev_hand_state["left"] - msg_hand.left_hand) > self.touch_threshold or
                abs(self.prev_hand_state["right"] - msg_hand.right_hand) > self.touch_threshold
            )
            
            if hand_should_publish:
                self.hand_touch_pub.publish(msg_hand)
                self.hand_published = True
                self.prev_hand_state = {
                    "left": msg_hand.left_hand,
                    "right": msg_hand.right_hand
                }
            
            #  REAZIONI IMMEDIATE (NON DIPENDONO DA SOGLIA) - Sempre controllate
            self.check_hand_touch_transition(msg_hand)
            
        except Exception as e:
            self.get_logger().error(f"Errore poll_sensors: {e}")


    def check_bumper_transition(self, msg: Bumper):
        """Rileva transizioni bumper (0 -> 1) per trigger reactions - INDIPENDENTE DA SOGLIA"""
        if self.reaction_mode != "Autonomous":
            return
        
        # Transizione Left: 0 -> 1
        if self.prev_bumper_state["left"] == 0.0 and msg.left > 0.5:
            self.send_talking_action("ho urtato a sinistra")
        
        # Transizione Right: 0 -> 1
        if self.prev_bumper_state["right"] == 0.0 and msg.right > 0.5:
            self.send_talking_action("ho urtato a destra")
        
        # Transizione Back: 0 -> 1
        if self.prev_bumper_state["back"] == 0.0 and msg.back > 0.5:
            self.send_talking_action("ho urtato dietro")


    def check_head_touch_transition(self, msg: HeadTouch):
        """Rileva transizioni head touch (0 -> 1) per trigger reactions - INDIPENDENTE DA SOGLIA"""
        if self.reaction_mode != "Autonomous":
            return
        
        # Transizione Front: 0 -> 1
        if self.prev_head_state["front"] == 0.0 and msg.front > 0.5:
            self.send_talking_action("mi hanno toccato la fronte")
        
        # Transizione Middle: 0 -> 1
        if self.prev_head_state["middle"] == 0.0 and msg.middle > 0.5:
            self.send_talking_action("mi hanno toccato la testa")
        
        # Transizione Rear: 0 -> 1
        if self.prev_head_state["rear"] == 0.0 and msg.rear > 0.5:
            self.send_talking_action("mi hanno toccato dietro la testa")


    def check_hand_touch_transition(self, msg: HandTouch):
        """Rileva transizioni hand touch (0 -> 1) per trigger reactions - INDIPENDENTE DA SOGLIA"""
        if self.reaction_mode != "Autonomous":
            return
        
        # Transizione Left: 0 -> 1
        if self.prev_hand_state["left"] == 0.0 and msg.left_hand > 0.5:
            self.send_talking_action("mi hanno toccato la mano sinistra")
        
        # Transizione Right: 0 -> 1
        if self.prev_hand_state["right"] == 0.0 and msg.right_hand > 0.5:
            self.send_talking_action("mi hanno toccato la mano destra")


    def publish_mock_state(self):
        """Pubblica stato mock - PRIMA VOLTA SEMPRE, POI SOLO SE CAMBIA"""
        msg_bumper = Bumper()
        msg_bumper.left = 0.0
        msg_bumper.right = 0.0
        msg_bumper.back = 0.0
        
        if not self.bumper_published:
            self.bumper_pub.publish(msg_bumper)
            self.bumper_published = True
            self.prev_bumper_state = {"left": 0.0, "right": 0.0, "back": 0.0}
        
        msg_head = HeadTouch()
        msg_head.front = 0.0
        msg_head.middle = 0.0
        msg_head.rear = 0.0
        
        if not self.head_published:
            self.head_touch_pub.publish(msg_head)
            self.head_published = True
            self.prev_head_state = {"front": 0.0, "middle": 0.0, "rear": 0.0}
        
        msg_hand = HandTouch()
        msg_hand.left_hand = 0.0
        msg_hand.right_hand = 0.0
        
        if not self.hand_published:
            self.hand_touch_pub.publish(msg_hand)
            self.hand_published = True
            self.prev_hand_state = {"left": 0.0, "right": 0.0}


    def battery_sub(self):
        """Pubblica batteria - PRIMA VOLTA SEMPRE, POI SOLO SE CAMBIA OLTRE SOGLIA"""
        msg = Battery()
        
        if self.memory is None:
            # Mock mode
            charge_val = 85.0
            current_val = 0.5
            temp_val = 30.0
        else:
            try:
                charge_val = round(float(self.memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")) * 100, 2)
                current_val = round(float(self.memory.getData("Device/SubDeviceList/Battery/Current/Sensor/Value")), 2)
                temp_val = round(float(self.memory.getData("Device/SubDeviceList/Battery/Temperature/Sensor/Value")), 2)
            except Exception as e:
                self.get_logger().error(f"Errore battery_sub: {e}")
                charge_val = 85.0
                current_val = 0.5
                temp_val = 30.0
        
        msg.charge_percent = f"{charge_val:.2f}%"
        msg.current_ampere = f"{current_val:.2f}A"
        msg.temperature = f"{temp_val:.2f}°C"
        msg.charging = current_val >= 0
        
        #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
        #  POI SOLO SE CAMBIA OLTRE SOGLIA
        battery_should_publish = (
            not self.battery_published or
            abs(self.prev_battery_state["charge"] - charge_val) > self.battery_threshold or
            abs(self.prev_battery_state["current"] - current_val) > self.battery_threshold or
            abs(self.prev_battery_state["temp"] - temp_val) > self.battery_threshold
        )
        
        if battery_should_publish:
            self.battery_pub.publish(msg)
            self.battery_published = True
            self.prev_battery_state = {
                "charge": charge_val,
                "current": current_val,
                "temp": temp_val
            }


    def send_talking_action(self, text):
        if not self.talking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server /talking non disponibile")
            return
        
        goal_msg = Talking.Goal()
        goal_msg.message = text
        
        self.get_logger().info(f"Invio: '{text}'")
        send_goal_future = self.talking_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Action Talking rifiutato")
            return
        self.get_logger().info("Action Talking accettato")


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_sensor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

        self.declare_parameter('reaction_mode', 'Autonomous')  # Default Autonomous
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

        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
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
                
                self.setup_bumper_events()
                self.setup_headTouch_events()
                self.setup_handTouch_events()
                
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self.memory = None

        self.bumper_pub = self.create_publisher(Bumper, "/pepper/topics/bumper", qos_best_effort_10)
        self.head_touch_pub = self.create_publisher(HeadTouch, "/pepper/topics/head_touch", qos_best_effort_10)
        self.battery_pub = self.create_publisher(Battery, "/pepper/topics/battery", qos_best_effort_10)
    
        self.bumper_pressed = False

        self.timer_battery = self.create_timer(5.0, self.battery_sub)
        self.talking_client = ActionClient(self, Talking, '/pepper/actions/talking')

    def setup_bumper_events(self):
        try:
            self.memory.subscribeToEvent(
                "LeftBumperPressed",
                self.get_name(),
                lambda event, value, msg: self.on_bumper_event("front_left", value)
            )
            self.memory.subscribeToEvent(
                "RightBumperPressed",
                self.get_name(),
                lambda event, value, msg: self.on_bumper_event("front_right", value)
            )
            self.memory.subscribeToEvent(
                "BackBumperPressed",
                self.get_name(),
                lambda event, value, msg: self.on_bumper_event("back", value)
            )
            self.get_logger().info("Eventi bumper sottoscritti con successo")
        except Exception as e:
            self.get_logger().error(f"Errore sottoscrizione eventi bumper: {e}")

    def setup_headTouch_events(self):
        try:
            self.memory.subscribeToEvent(
                "FrontTactilTouched",
                self.get_name(),
                lambda event, value, msg: self.on_head_touch_event("front", value)
            )
            self.memory.subscribeToEvent(
                "MiddleTactilTouched",
                self.get_name(),
                lambda event, value, msg: self.on_head_touch_event("middle", value)
            )
            self.memory.subscribeToEvent(
                "RearTactilTouched",
                self.get_name(),
                lambda event, value, msg: self.on_head_touch_event("rear", value)
            )
            self.get_logger().info("Eventi touch testa sottoscritti con successo")
        except Exception as e:
            self.get_logger().error(f"Errore sottoscrizione eventi touch testa: {e}")

    def setup_handTouch_events(self):
        try:
            self.memory.subscribeToEvent(
                "HandLeftBackTouched",
                self.get_name(),
                lambda event, value, msg: self.on_hand_touch_event("left", value))
            self.memory.subscribeToEvent(
                "HandRightBackTouched",
                self.get_name(),
                lambda event, value, msg: self.on_hand_touch_event("right", value))
            self.get_logger().info("Eventi touch mani sottoscritti con successo")
        except Exception as e:
            self.get_logger().error(f"Errore sottoscrizione eventi hand touch: {e}")

    def on_hand_touch_event(self, hand: str, value: float):
        self.get_logger().info(f"Hand touch {hand} event: {value}")
        msg = HandTouch()
        try:
            msg.left_hand = self.memory.getData("Device/SubDeviceList/LHand/Touch/Back/Sensor/Value")
            msg.right_hand = self.memory.getData("Device/SubDeviceList/RHand/Touch/Back/Sensor/Value")
            self.hand_touch_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore lettura hand touch: {e}")

    def on_bumper_event(self, position: str, value: float):
        self.get_logger().info(f"Bumper {position} event: {value}")
        
        msg = Bumper()
        try:
            msg.left = self.memory.getData("Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value")
            msg.right = self.memory.getData("Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value")
            msg.back = self.memory.getData("Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value")
            self.bumper_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore lettura bumper: {e}")
            return

        if self.reaction_mode != "Autonomous":
            self.get_logger().debug("Reaction disabilitata, salto reazione bumper")
            return
        
        if value > 0.5 and not self.bumper_pressed:
            message = None
            if position == "front_left":
                message = "ho urtato a sinistra"
            elif position == "front_right":
                message = "ho urtato a destra"
            elif position == "back":
                message = "ho urtato dietro"
            
            if message:
                self.send_talking_action(message)
                self.bumper_pressed = True
        
        if msg.left == 0.0 and msg.right == 0.0 and msg.back == 0.0:
            self.bumper_pressed = False

    def on_head_touch_event(self, position: str, value: float):
        self.get_logger().info(f"Head touch {position} event: {value}")

        msg = HeadTouch()
        try:
            msg.front = self.memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
            msg.middle = self.memory.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
            msg.rear = self.memory.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
            self.head_touch_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore lettura head touch: {e}")
            return

        if self.reaction_mode != "Autonomous":
            self.get_logger().debug("Reaction disabilitata, salto reazione head touch")
            return
        
        if value > 0.5:
            message = None
            if position == "front":
                message = "mi hanno toccato la fronte"
            elif position == "middle":
                message = "mi hanno toccato la testa"
            elif position == "rear":
                message = "mi hanno toccato dietro la testa"
            
            if message:
                self.send_talking_action(message)

    def battery_sub(self):
        msg = Battery()
        
        if self.memory is None:
            msg.charge_percent = 85.0
            msg.current_ampere = 0.5
            msg.temperature = 30.0
            msg.charging = False
            self.battery_pub.publish(msg)
            return
        
        try:
            msg.charge_percent = self.memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
            msg.current_ampere = self.memory.getData("Device/SubDeviceList/Battery/Current/Sensor/Value")
            msg.temperature = self.memory.getData("Device/SubDeviceList/Battery/Temperature/Sensor/Value")
            msg.charging = msg.current_ampere < 0            
            self.battery_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore battery_sub: {e}")

    def send_talking_action(self, text):
        if not self.talking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server /talking non disponibile")
            return
        
        goal_msg = Talking.Goal()
        goal_msg.message = text
        
        self.get_logger().info(f"Invio action Talking: '{text}'")
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

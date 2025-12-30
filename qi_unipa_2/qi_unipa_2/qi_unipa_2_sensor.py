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

        # ===== Parametri =====
        self.declare_parameter('reaction_mode', 'Autonomous')
        self.reaction_mode = self.get_parameter('reaction_mode').value

        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)

        mock_mode = self.get_parameter('mock_mode').value
        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value

        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)

        self.mock_mode = mock_mode
        if mock_mode:
            self.session = None
            self.memory = None
            self.get_logger().warn("Nodo SENSOR attivo in MOCK MODE")
        else:
            try:
                self.session = utils.session
                self.memory = self.session.service("ALMemory")
                self.get_logger().info("Nodo SENSOR attivo e connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e

                

        # ===== Publisher =====
        self.bumper_pub = self.create_publisher(Bumper, "/pepper/topics/bumper", qos_best_effort_10)
        self.head_touch_pub = self.create_publisher(HeadTouch, "/pepper/topics/head_touch", qos_best_effort_10)
        self.hand_touch_pub = self.create_publisher(HandTouch, "/pepper/topics/hand_touch", qos_best_effort_10)
        self.battery_pub = self.create_publisher(Battery, "/pepper/topics/battery", qos_best_effort_10)

        # ===== Soglie =====
        self.bumper_threshold = 0.05
        self.touch_threshold = 0.05
        self.battery_threshold = 0.5

        # ===== Flag prima pubblicazione =====
        self.bumper_published = False
        self.head_published = False
        self.hand_published = False
        self.battery_published = False

        # ===== Stati precedenti per pubblicazione =====
        self.prev_bumper_pub = {"left": 0.0, "right": 0.0, "back": 0.0}
        self.prev_head_pub = {"front": 0.0, "middle": 0.0, "rear": 0.0}
        self.prev_hand_pub = {"left": 0.0, "right": 0.0}

        # Stato precedente batteria (percentuale, corrente, temperatura, charging)
        self.prev_battery_state = {
            "charge": None,
            "current": None,
            "temp": None,
            "charging": None,
        }

        # ===== Stati precedenti per DETEZIONE TRANSIZIONI =====
        self.prev_bumper_tr = {"left": 0.0, "right": 0.0, "back": 0.0}
        self.prev_head_tr = {"front": 0.0, "middle": 0.0, "rear": 0.0}
        self.prev_hand_tr = {"left": 0.0, "right": 0.0}

        # ===== Timers =====
        self.timer_sensors = self.create_timer(0.1, self.poll_sensors)
        self.timer_battery = self.create_timer(5.0, self.battery_sub)

        # ===== Action client =====
        self.talking_client = ActionClient(self, Talking, "/pepper/actions/talking")

    # =====================================================================
    #                               SENSOR POLLING
    # =====================================================================
    def poll_sensors(self):
        if self.mock_mode or self.memory is None:
            self.publish_mock_state()
            return

        try:
            # ========================= BUMPER =========================
            msg_bumper = Bumper()
            msg_bumper.left = float(self.memory.getData("Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value"))
            msg_bumper.right = float(self.memory.getData("Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value"))
            msg_bumper.back = float(self.memory.getData("Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value"))

            # ----- Check transizioni (0 → 1) -----
            self.check_transition(
                sensor="bumper",
                prev=self.prev_bumper_tr,
                cur={"left": msg_bumper.left, "right": msg_bumper.right, "back": msg_bumper.back},
                mapping={
                    "left": "ho urtato a sinistra",
                    "right": "ho urtato a destra",
                    "back": "ho urtato dietro"
                }
            )

            # Update transition states
            self.prev_bumper_tr = {
                "left": msg_bumper.left, "right": msg_bumper.right, "back": msg_bumper.back
            }

            # ----- Pubblicazione se cambia -----
            if (not self.bumper_published or
                abs(self.prev_bumper_pub["left"] - msg_bumper.left) > self.bumper_threshold or
                abs(self.prev_bumper_pub["right"] - msg_bumper.right) > self.bumper_threshold or
                abs(self.prev_bumper_pub["back"] - msg_bumper.back) > self.bumper_threshold):

                self.bumper_pub.publish(msg_bumper)
                self.bumper_published = True
                self.prev_bumper_pub = {
                    "left": msg_bumper.left,
                    "right": msg_bumper.right,
                    "back": msg_bumper.back
                }

            # ========================= HEAD TOUCH =========================
            msg_head = HeadTouch()
            msg_head.front = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value"))
            msg_head.middle = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value"))
            msg_head.rear = float(self.memory.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value"))

            self.check_transition(
                sensor="head",
                prev=self.prev_head_tr,
                cur={"front": msg_head.front, "middle": msg_head.middle, "rear": msg_head.rear},
                mapping={
                    "front": "mi hanno toccato la fronte",
                    "middle": "mi hanno toccato la testa",
                    "rear": "mi hanno toccato dietro la testa"
                }
            )

            self.prev_head_tr = {
                "front": msg_head.front, "middle": msg_head.middle, "rear": msg_head.rear
            }

            if (not self.head_published or
                abs(self.prev_head_pub["front"] - msg_head.front) > self.touch_threshold or
                abs(self.prev_head_pub["middle"] - msg_head.middle) > self.touch_threshold or
                abs(self.prev_head_pub["rear"] - msg_head.rear) > self.touch_threshold):

                self.head_touch_pub.publish(msg_head)
                self.head_published = True
                self.prev_head_pub = {
                    "front": msg_head.front,
                    "middle": msg_head.middle,
                    "rear": msg_head.rear
                }

            # ========================= HAND TOUCH =========================
            msg_hand = HandTouch()
            msg_hand.left_hand = float(self.memory.getData("Device/SubDeviceList/LHand/Touch/Back/Sensor/Value"))
            msg_hand.right_hand = float(self.memory.getData("Device/SubDeviceList/RHand/Touch/Back/Sensor/Value"))

            self.check_transition(
                sensor="hand",
                prev=self.prev_hand_tr,
                cur={"left": msg_hand.left_hand, "right": msg_hand.right_hand},
                mapping={
                    "left": "mi hanno toccato la mano sinistra",
                    "right": "mi hanno toccato la mano destra"
                }
            )

            self.prev_hand_tr = {
                "left": msg_hand.left_hand, "right": msg_hand.right_hand
            }

            if (not self.hand_published or
                abs(self.prev_hand_pub["left"] - msg_hand.left_hand) > self.touch_threshold or
                abs(self.prev_hand_pub["right"] - msg_hand.right_hand) > self.touch_threshold):

                self.hand_touch_pub.publish(msg_hand)
                self.hand_published = True
                self.prev_hand_pub = {
                    "left": msg_hand.left_hand,
                    "right": msg_hand.right_hand
                }

        except Exception as e:
            self.get_logger().error(f"Errore poll_sensors: {e}")

    # =====================================================================
    #                       GENERIC TRANSITION CHECK
    # =====================================================================
    def check_transition(self, sensor, prev, cur, mapping):
        if self.reaction_mode != "Autonomous":
            return

        for key, value in cur.items():
            if prev[key] == 0.0 and value > 0.5:  # Transizione 0→1
                self.send_talking_action(mapping[key])

    # =====================================================================
    #                              TALKING ACTION
    # =====================================================================
    def send_talking_action(self, text):
        if not self.talking_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Talking action server non disponibile")
            return

        goal = Talking.Goal()
        goal.message = text
        self.talking_client.send_goal_async(goal)

    # =====================================================================
    #                             MOCK STATE
    # =====================================================================
    def publish_mock_state(self):
        msg = Bumper()
        msg.left = msg.right = msg.back = 0.0

        if not self.bumper_published:
            self.bumper_published = True
            self.prev_bumper_pub = {"left": 0.0, "right": 0.0, "back": 0.0}
            self.bumper_pub.publish(msg)

    # =====================================================================
    #                               BATTERY
    # =====================================================================
    def battery_sub(self):
        msg = Battery()

        if self.memory is None:
            charge = 85.0
            current = 0.5
            temp = 30.0
        else:
            try:
                charge = round(float(self.memory.getData(
                    "Device/SubDeviceList/Battery/Charge/Sensor/Value"
                )) * 100, 2)
                current = round(float(self.memory.getData(
                    "Device/SubDeviceList/Battery/Current/Sensor/Value"
                )), 2)
                temp = round(float(self.memory.getData(
                    "Device/SubDeviceList/Battery/Temperature/Sensor/Value"
                )), 2)
            except Exception as e:
                self.get_logger().warn(f"Errore lettura batteria, uso valori di fallback: {e}")
                charge = 85.0
                current = 0.5
                temp = 30.0

        charging = current >= 0

        msg.charge_percent = f"{charge:.2f}%"
        msg.current_ampere = f"{current:.2f}A"
        msg.temperature = f"{temp:.2f}°C"
        msg.charging = charging

        # ===== LOGICA: prima volta sempre, poi solo se cambia oltre soglia =====
        # Usa self.battery_threshold in punti percentuali (es. 0.5 = 0.5%)
        should_publish = (
            not self.battery_published or
            self.prev_battery_state["charge"] is None or
            abs(self.prev_battery_state["charge"] - charge) > self.battery_threshold or
            self.prev_battery_state["current"] != current or
            self.prev_battery_state["temp"] != temp or
            self.prev_battery_state["charging"] != charging
        )

        if should_publish:
            self.battery_pub.publish(msg)
            self.battery_published = True
            self.prev_battery_state = {
                "charge": charge,
                "current": current,
                "temp": temp,
                "charging": charging,
            }



# =====================================================================
#                                MAIN
# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_sensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import qi
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qi_unipa_2_interfaces.msg import Sonar, IMU #,Infrared
from qi_unipa_2_interfaces.srv import GetPosition #type: ignore
from qi_unipa_2.utils import Utils #type: ignore


class QiUnipa2_reference(Node):    
    def __init__(self):
        super().__init__('qi_unipa_2_reference')
        
        # Dichiarazione parametri
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        # Lettura parametri
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Creazione Utils
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        qos_reliable_10 = utils.get_QoS('reliable', 10)
        qos_best_effort_10 = utils.get_QoS('best_effort', 10)

        # Inizializzazione valori MOCK
        self.mock_mode = mock_mode
        if mock_mode:
            self.mock_imu = {
                'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,
                'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
                'angle_x': 0.0, 'angle_y': 0.0
            }
           # self.mock_infrared = {'left_ir': 0.5, 'right_ir': 0.5}
            self.mock_sonar = {'front_sonar': 0.5, 'back_sonar': 0.5}
            self.mock_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # Connessione condizionale a Pepper
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self.memory = None
            self.motion = None
            self.sonar = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                # Connessione ai servizi di Pepper
                self.memory = self.session.service("ALMemory")
                self.motion = self.session.service("ALMotion")
                self.sonar = self.session.service("ALSonar")
                self.sonar.subscribe("Sonar_nav")
                self.get_logger().info("Sonar attivato")
                
                self.get_logger().info("Connesso a Pepper - Servizi navigation attivi")
                
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.mock_mode = True
                self.session = None
                self.memory = None
                self.motion = None
                self.sonar = None
                
                # Inizializza valori mock anche in caso di fallback
                self.mock_imu = {
                    'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,
                    'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
                    'angle_x': 0.0, 'angle_y': 0.0
                }
               # self.mock_infrared = {'left_ir': 0.5, 'right_ir': 0.5}
                self.mock_sonar = {'front_sonar': 0.5, 'back_sonar': 0.5}
                self.mock_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # Inizializzazione publishers
        self.sonar_pub = self.create_publisher(Sonar, "/pepper/topics/sonar", qos_best_effort_10)
        self.imu_pub = self.create_publisher(IMU, "/pepper/topics/imu", qos_best_effort_10)
      #  self.infrared_pub = self.create_publisher(Infrared, "/pepper/topics/infrared", qos_best_effort_10)
        
        # Service GetPosition
        self.get_position_srv = self.create_service(GetPosition,'/pepper/services/get_position',self.get_position_callback,qos_profile=qos_reliable_10)

        # Timers per sensori continui
        self.timer_imu = self.create_timer(0.1, self.imu_callback)           # 10Hz
        self.timer_sonar = self.create_timer(1.0, self.sonar_callback)       # 1Hz
       # self.timer_infrared = self.create_timer(1.0, self.infrared_callback) # 1Hz


    def get_position_callback(self, request, response):
        """Service GetPosition per ottenere posizione robot on-demand"""
        if self.mock_mode:
            self.get_logger().info("[MOCK] get_position chiamato")
            response.x = self.mock_position['x']
            response.y = self.mock_position['y']
            response.theta = self.mock_position['theta']
            return response
        
        try:
            # getRobotPosition(False) = frame robot locale
            pose = self.motion.getRobotPosition(False)
            response.x = pose[0]
            response.y = pose[1]
            response.theta = pose[2]
            
        except Exception as e:
            self.get_logger().error(f"Errore get_position_callback: {e}")
            response.x = 0.0
            response.y = 0.0
            response.theta = 0.0
        
        return response


    def imu_callback(self):
        """Pubblicazione dati IMU completi a 10Hz"""
        msg = IMU()
        
        if self.mock_mode:
            msg.accel_x = self.mock_imu['accel_x']
            msg.accel_y = self.mock_imu['accel_y']
            msg.accel_z = self.mock_imu['accel_z']
            msg.gyro_x = self.mock_imu['gyro_x']
            msg.gyro_y = self.mock_imu['gyro_y']
            msg.gyro_z = self.mock_imu['gyro_z']
            msg.angle_x = self.mock_imu['angle_x']
            msg.angle_y = self.mock_imu['angle_y']
            self.imu_pub.publish(msg)
            return
        
        try:
            # Lettura accelerometro da ALMemory
            msg.accel_x = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/AccelerometerX/Sensor/Value"))
            msg.accel_y = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/AccelerometerY/Sensor/Value"))
            msg.accel_z = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/AccelerometerZ/Sensor/Value"))
            
            # Lettura giroscopio da ALMemory
            msg.gyro_x = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/GyroscopeX/Sensor/Value"))
            msg.gyro_y = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/GyroscopeY/Sensor/Value"))
            msg.gyro_z = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/GyroscopeZ/Sensor/Value"))
            
            # Lettura angoli orientamento da ALMemory
            msg.angle_x = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/AngleX/Sensor/Value"))
            msg.angle_y = float(self.memory.getData("Device/SubDeviceList/InertialSensorBase/AngleY/Sensor/Value"))
            
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore imu_callback: {e}")

    # NB: SONAR è un tipo di hardware ATTIVO e particolare ed ha necessità di essere terminato a fine uso
    # per questo motivo è gestito in destroy_node() a chiusura del nodo
    def sonar_callback(self):
        """Pubblicazione dati sonar a 1Hz"""
        msg = Sonar()
        
        if self.mock_mode:
            msg.front_sonar = self.mock_sonar['front_sonar']
            msg.back_sonar = self.mock_sonar['back_sonar']
            self.sonar_pub.publish(msg)
            return
        
        try:
            msg.front_sonar = self.memory.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")
            msg.back_sonar = self.memory.getData("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value")
            self.sonar_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore sonar_callback: {e}")

    '''
    def infrared_callback(self):
        """Pubblicazione dati sensori infrarossi a 1Hz"""
        msg = Infrared()
        
        if self.mock_mode:
            msg.left_ir = self.mock_infrared['left_ir']
            msg.right_ir = self.mock_infrared['right_ir']
            self.infrared_pub.publish(msg)
            return
        
        try:
            # Lettura sensori infrarossi da ALMemory
            msg.left_ir = self.memory.getData("Device/SubDeviceList/Platform/InfraredSensor/Left/Sensor/Value")
            msg.right_ir = self.memory.getData("Device/SubDeviceList/Platform/InfraredSensor/Right/Sensor/Value")
            
            self.infrared_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore infrared_callback: {e}")
    '''

    def destroy_node(self):
        """Cleanup quando nodo viene terminato"""
        self.get_logger().info("Chiusura nodo navigation...")
        
        if not self.mock_mode and self.sonar is not None:
            try:
                self.sonar.unsubscribe("Sonar_nav")
                self.get_logger().info("Sonar disattivato")
            except Exception as e:
                self.get_logger().warn(f"Errore disattivazione sonar: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa2_reference()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main()

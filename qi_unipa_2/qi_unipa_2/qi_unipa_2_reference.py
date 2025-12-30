import qi
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qi_unipa_2_interfaces.msg import Sonar, IMU 
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
            self.mock_sonar = {'front_sonar': 0.5, 'back_sonar': 0.5}
            self.mock_position = {'x': 1.0, 'y': 2.0, 'theta': 3.0}

        # Connessione condizionale a Pepper
        if mock_mode:
            self.get_logger().warn("Nodo REFERENCE attivo in MOCK MODE")
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
                self.get_logger().info("Nodo REFERENCE attivo e connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().error("TERMINAZIONE FORZATA - Connessione fallita")
                # Solleva eccezione per terminare il nodo
                raise RuntimeError(
                    f"Connessione a Pepper fallita ({ip}:{port}). "
                    f"Verifica che Pepper sia acceso e raggiungibile. "
                ) from e



        # ========== VALORI MOCK MODE ==========
        self.mock_imu = {
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'angle_x': 0.0, 'angle_y': 0.0
        }
        self.mock_sonar = {'front_sonar': 0.5, 'back_sonar': 0.5}
        self.mock_position = {'x': 1.0, 'y': 2.0, 'theta': 3.0}


        # ========== SOGLIE DI CAMBIO ==========
        self.imu_accel_threshold = 0.05      # m/s² di cambio
        self.imu_gyro_threshold = 0.05       # rad/s di cambio
        self.imu_angle_threshold = 0.05      # rad di cambio
        self.sonar_threshold = 0.02          # metri di cambio
        
        # ========== FLAG PRIMA PUBBLICAZIONE ==========
        self.imu_published = False
        self.sonar_published = False
        
        # ========== STATO PRECEDENTE PER RILEVARE CAMBI ==========
        self.prev_imu_state = {
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'angle_x': 0.0, 'angle_y': 0.0
        }
        self.prev_sonar_state = {'front_sonar': 0.5, 'back_sonar': 0.5}

        # Inizializzazione publishers
        self.sonar_pub = self.create_publisher(Sonar, "/pepper/topics/sonar", qos_best_effort_10)
        self.imu_pub = self.create_publisher(IMU, "/pepper/topics/imu", qos_best_effort_10)
        
        # Service GetPosition
        self.get_position_srv = self.create_service(GetPosition, '/pepper/services/get_position', self.get_position_callback, qos_profile=qos_reliable_10)

        # Timers per sensori continui
        self.timer_imu = self.create_timer(0.1, self.imu_callback)           # 10Hz
        self.timer_sonar = self.create_timer(1.0, self.sonar_callback)       # 1Hz
   
    


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
            response.theta = 20.0
        
        return response


    def imu_callback(self):
        """Pubblicazione dati IMU - PRIMA VOLTA SEMPRE, POI SOLO SE CAMBIANO"""
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
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            imu_should_publish = (
                not self.imu_published or
                abs(self.prev_imu_state['accel_x'] - msg.accel_x) > self.imu_accel_threshold or
                abs(self.prev_imu_state['accel_y'] - msg.accel_y) > self.imu_accel_threshold or
                abs(self.prev_imu_state['accel_z'] - msg.accel_z) > self.imu_accel_threshold or
                abs(self.prev_imu_state['gyro_x'] - msg.gyro_x) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['gyro_y'] - msg.gyro_y) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['gyro_z'] - msg.gyro_z) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['angle_x'] - msg.angle_x) > self.imu_angle_threshold or
                abs(self.prev_imu_state['angle_y'] - msg.angle_y) > self.imu_angle_threshold
            )
            
            if imu_should_publish:
                self.imu_pub.publish(msg)
                self.imu_published = True
                self.prev_imu_state = {
                    'accel_x': msg.accel_x,
                    'accel_y': msg.accel_y,
                    'accel_z': msg.accel_z,
                    'gyro_x': msg.gyro_x,
                    'gyro_y': msg.gyro_y,
                    'gyro_z': msg.gyro_z,
                    'angle_x': msg.angle_x,
                    'angle_y': msg.angle_y
                }
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
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            imu_should_publish = (
                not self.imu_published or
                abs(self.prev_imu_state['accel_x'] - msg.accel_x) > self.imu_accel_threshold or
                abs(self.prev_imu_state['accel_y'] - msg.accel_y) > self.imu_accel_threshold or
                abs(self.prev_imu_state['accel_z'] - msg.accel_z) > self.imu_accel_threshold or
                abs(self.prev_imu_state['gyro_x'] - msg.gyro_x) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['gyro_y'] - msg.gyro_y) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['gyro_z'] - msg.gyro_z) > self.imu_gyro_threshold or
                abs(self.prev_imu_state['angle_x'] - msg.angle_x) > self.imu_angle_threshold or
                abs(self.prev_imu_state['angle_y'] - msg.angle_y) > self.imu_angle_threshold
            )
            
            if imu_should_publish:
                self.imu_pub.publish(msg)
                self.imu_published = True
                self.prev_imu_state = {
                    'accel_x': msg.accel_x,
                    'accel_y': msg.accel_y,
                    'accel_z': msg.accel_z,
                    'gyro_x': msg.gyro_x,
                    'gyro_y': msg.gyro_y,
                    'gyro_z': msg.gyro_z,
                    'angle_x': msg.angle_x,
                    'angle_y': msg.angle_y
                }
            
        except Exception as e:
            self.get_logger().error(f"Errore imu_callback: {e}")


    def sonar_callback(self):
        """Pubblicazione dati sonar - PRIMA VOLTA SEMPRE, POI SOLO SE CAMBIANO"""
        msg = Sonar()
        
        if self.mock_mode:
            msg.front_sonar = self.mock_sonar['front_sonar']
            msg.back_sonar = self.mock_sonar['back_sonar']
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            sonar_should_publish = (
                not self.sonar_published or
                abs(self.prev_sonar_state['front_sonar'] - msg.front_sonar) > self.sonar_threshold or
                abs(self.prev_sonar_state['back_sonar'] - msg.back_sonar) > self.sonar_threshold
            )
            
            if sonar_should_publish:
                self.sonar_pub.publish(msg)
                self.sonar_published = True
                self.prev_sonar_state = {
                    'front_sonar': msg.front_sonar,
                    'back_sonar': msg.back_sonar
                }
            return
        
        try:
            msg.front_sonar = float(self.memory.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value"))
            msg.back_sonar = float(self.memory.getData("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value"))
            
            #  PUBBLICA SEMPRE ALLA PRIMA VOLTA
            #  POI SOLO SE CAMBIA OLTRE SOGLIA
            sonar_should_publish = (
                not self.sonar_published or
                abs(self.prev_sonar_state['front_sonar'] - msg.front_sonar) > self.sonar_threshold or
                abs(self.prev_sonar_state['back_sonar'] - msg.back_sonar) > self.sonar_threshold
            )
            
            if sonar_should_publish:
                self.sonar_pub.publish(msg)
                self.sonar_published = True
                self.prev_sonar_state = {
                    'front_sonar': msg.front_sonar,
                    'back_sonar': msg.back_sonar
                }
            
        except Exception as e:
            self.get_logger().error(f"Errore sonar_callback: {e}")


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

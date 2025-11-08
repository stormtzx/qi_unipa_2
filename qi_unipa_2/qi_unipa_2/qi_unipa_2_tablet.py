import qi
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Bool, String
import time
import threading
from qi_unipa_2.utils import Utils


# --- Selenium + Chrome ---
from selenium import webdriver
from selenium.webdriver.chrome.service import Service as ChromeService
from selenium.webdriver.chrome.options import Options
from webdriver_manager.chrome import ChromeDriverManager

class QiUnipa2_tablet(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_tablet')
        
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        utils = Utils(ip=ip, port=port, mock_mode=mock_mode)
        
        # Unifica MOCK e browser locale
        self.use_local_browser = mock_mode
        self.session = None
        self._tablet = None

        if not mock_mode:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                self._tablet = self.session.service("ALTabletService")
                self._tablet.resetTablet()
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a BROWSER LOCALE")
                self.use_local_browser = True

        # Sottoscrizioni ROS
        self.show_sub = self.create_subscription(String, "/show", self.show, 10)
        self.show_tablet_sub = self.create_subscription(String, "/show_tablet", self.show_tablet, 10)
        self.hide_sub = self.create_subscription(Bool, "/hide", self.hide, 10)
        self.ip_sub = self.create_subscription(String, "/ip", self.set_ip, 10)
        self.ip_tablet_sub = self.create_subscription(String, "/ip_tablet", self.set_ip_tablet, 10)
        self.image_sub = self.create_subscription(Bool, "/image_tablet", self.show_tablet_stream_callback, 10)

        # Stato e blocchi
        self.webserver_ip = ""
        self.tablet_ip = ""
        self.tablet_lock = threading.Lock()
        self.last_url = ""
        self.driver = None

        # Configura Chrome options
        self.options = Options()
        self.options.add_argument("--no-sandbox")
        self.options.add_argument("--disable-dev-shm-usage")
        self.options.add_argument("--start-maximized")
        self.options.add_argument("--disable-gpu")
        self.options.add_argument("--disable-infobars")
        self.options.add_argument("--disable-extensions")
        #self.options.add_argument("--headless")

        if self.use_local_browser:
            self.setup_browser()

        self.get_logger().info("Nodo tablet avviato correttamente")

    def setup_browser(self):
        try:
            self.get_logger().info("Avvio browser locale (Chrome) con Selenium...")
            self.driver = webdriver.Chrome(
                service=ChromeService(ChromeDriverManager().install()),
                options=self.options
            )
            self.get_logger().info("Browser locale (Chrome) avviato con successo.")
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio di Chrome: {e}")
            self.driver = None

    def set_ip(self, msg):
        self.webserver_ip = msg.data
        self.get_logger().debug(f"IP webserver impostato: {self.webserver_ip}")

    def set_ip_tablet(self, msg):
        self.tablet_ip = msg.data

    def show(self, msg):
        with self.tablet_lock:
            try:
                webPage = msg.data
                full_url = f"{self.webserver_ip}/{webPage}"
                self.get_logger().info(f"Richiesta visualizzazione: {full_url}")

                if not self.use_local_browser and self._tablet is not None:
                    if full_url == self.last_url:
                        self.get_logger().info(f"Pagina già aperta: {full_url}, salto reload")
                        return
                    self._tablet.showWebview(full_url)
                    self.last_url = full_url
                else:
                    if self.driver:
                        self.driver.get(full_url)
                        self.last_url = full_url
                    else:
                        self.get_logger().warn("Nessun browser disponibile per visualizzare la pagina")
            except Exception as e:
                self.get_logger().error(f"Errore durante show: {e}")

    def show_tablet(self, msg):
        with self.tablet_lock:
            try:
                webPage = msg.data
                full_url = f"{self.tablet_ip}/{webPage}"

                self.last_url = full_url
                if self._tablet is not None:
                    self._tablet.showWebview(full_url)
                else:
                    self.get_logger().warn("Tablet service non disponibile")
            except Exception as e:
                self.get_logger().error(f"Errore durante show_tablet: {e}")

    def show_tablet_stream_callback(self, msg):
        page_name = "mostra_immagine.html"
        url = f"{self.tablet_ip}/{page_name}"
        try:
            if self._tablet is not None:
                self._tablet.showWebview(url)
            else:
                self.get_logger().warn("Tablet service non disponibile")
        except Exception as e:
            self.get_logger().error(f"Errore mostrando video sul tablet: {e}")

    def hide(self, msg):
        with self.tablet_lock:
            try:
                if not self.use_local_browser and self._tablet is not None:
                    self._tablet.hideWebview()
                if self.use_local_browser and self.driver:
                    self.driver.get("about:blank")
            except Exception as e:
                self.get_logger().error(f"Errore durante hide: {e}")

    def cleanup(self):
        self.get_logger().info("Pulizia nodo tablet...")
        try:
            if self._tablet is not None:
                self._tablet.hideWebview()
        except Exception as e:
            self.get_logger().warn(f"Impossibile nascondere webview: {e}")

        if self.driver:
            try:
                self.driver.quit()
            except Exception as e:
                self.get_logger().warn(f"Errore chiudendo browser locale: {e}")

def main(args=None):
    rclpy.init(args=args)
    time.sleep(0.5)
    node = QiUnipa2_tablet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from qi_unipa_2_interfaces.action import Browsing
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

        # Parametri e setup browser/tablet
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)

        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self.webserver_ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.tablet_ip = ""

        utils = Utils(ip=self.webserver_ip, port=self.port, mock_mode=self.mock_mode)

        self.tablet_lock = threading.Lock()
        self._tablet = None
        self.last_url = ""
        self.driver = None

        if not self.mock_mode:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {self.webserver_ip}:{self.port}")
                self.session = utils.session
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                self._tablet = self.session.service("ALTabletService")
                self._tablet.resetTablet()
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Nessuna connessione a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a BROWSER LOCALE in MOCK mode")
                self.mock_mode = True

        # Se MOCK o non connesso, usa browser locale
        if self.mock_mode:
            self.setup_browser()

        # Action server Browsing
        self._action_server = ActionServer(self, Browsing, 'browsing', execute_callback=self.browsing_execute_callback)

    def setup_browser(self):
        options = Options()
        options.add_argument("--no-sandbox")
        options.add_argument("--disable-dev-shm-usage")
        options.add_argument("--start-maximized")
        options.add_argument("--disable-gpu")
        options.add_argument("--disable-infobars")
        options.add_argument("--disable-extensions")
        #options.add_argument("--headless")

        try:
            self.get_logger().info("Avvio browser locale (Chrome) con Selenium...")
            self.driver = webdriver.Chrome(service=ChromeService(ChromeDriverManager().install()), options=options)
            self.get_logger().info("Browser locale (Chrome) avviato con successo.")
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio di Chrome: {e}")
            self.driver = None


    def browsing_execute_callback(self, goal_handle):
        html_page = goal_handle.request.html_page
        use_tablet = goal_handle.request.use_tablet

        self.get_logger().info(f"Browsing: richiesta pagina {html_page}, use_tablet={use_tablet}")

        feedback_msg = Browsing.Feedback()
        feedback_msg.current_status = "Inizio elaborazione..."
        goal_handle.publish_feedback(feedback_msg)

        with self.tablet_lock:
            try:
                if html_page.lower() == "hide" or html_page.lower() == "about:blank":
                    if use_tablet and self._tablet is not None:
                        self._tablet.hideWebview()
                    elif self.driver:
                        self.driver.get("about:blank")
                    else:
                        raise RuntimeError("Nessun metodo disponibile per nascondere webview")
                    feedback_msg.current_status = "Webview nascosta"
                else:
                    if use_tablet and not self.mock_mode and self._tablet is not None:
                        self._tablet.showWebview(html_page)
                    elif self.driver:
                        self.driver.get(html_page)
                    else:
                        raise RuntimeError("Nessun metodo disponibile per visualizzare pagina")
                    feedback_msg.current_status = "Pagina visualizzata"

                feedback_msg.tablet_ip = self.tablet_ip
                feedback_msg.webserver_ip = self.webserver_ip
                goal_handle.publish_feedback(feedback_msg)
                goal_handle.succeed()

                result = Browsing.Result()
                result.success = True
                return result

            except Exception as e:
                self.get_logger().error(f"Errore browsing: {e}")
                goal_handle.abort()
                result = Browsing.Result()
                result.success = False
                return result


def main(args=None):
    rclpy.init(args=args) 
    node = QiUnipa2_tablet()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

            

import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from qi_unipa_2_interfaces.action import Browsing
import threading
import json
import logging
import time
from std_msgs.msg import Bool, String

from selenium import webdriver
from selenium.webdriver.chrome.service import Service as ChromeService
from selenium.webdriver.chrome.options import Options
from webdriver_manager.chrome import ChromeDriverManager

from qi_unipa_2.http_utils import (
    HTTPServerManager,
    request_queue_user,
    test_queue,
    DEFAULT_PORT
)
from qi_unipa_2.utils import Utils

logger = logging.getLogger(__name__)

class QiUnipa2_server(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_server')

        self.declare_parameter('http_port', DEFAULT_PORT)
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('pepper_ip', '192.168.0.102')
        self.declare_parameter('pepper_port', 9559)
        self.declare_parameter('reaction_mode', 'Autonomous')

        self.reaction_mode = self.get_parameter('reaction_mode').get_parameter_value().string_value
        self.get_logger().info(f"Reaction mode impostato su: {self.reaction_mode}")

        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self.pepper_ip = self.get_parameter('pepper_ip').get_parameter_value().string_value
        self.pepper_port = self.get_parameter('pepper_port').get_parameter_value().integer_value

        self.get_logger().info("Nodo Server ROS avviato.")

        if self.mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")

        self.stats = {
            'browsing_requests': 0,
            'forms_collected': 0,
            'errors': 0
        }

        self.http_manager = HTTPServerManager(port=self.http_port)
        self.server_thread = threading.Thread(
            target=self.http_manager.start,
            kwargs={'max_retries': 5},
            daemon=True
        )
        self.server_thread.start()

        self.tablet_lock = threading.Lock()
        self._tablet = None
        self._driver = None
        self.tablet_ip = ""
        self.session = None
        self._setup_display()

        self._browsing_action = ActionServer(
            self,
            Browsing,
            '/pepper/actions/browsing',
            execute_callback=self.browsing_execute_callback
        )

        self.get_logger().info("Server con Browsing action estesa inizializzato.")


    def _setup_display(self):
        if self.mock_mode:
            self.get_logger().info("MOCK MODE ATTIVO - Utilizzo browser locale")
            self._setup_browser()
            return

        try:
            self.get_logger().info(f"Tentativo connessione Pepper: {self.pepper_ip}:{self.pepper_port}")
            utils = Utils(ip=self.pepper_ip, port=self.pepper_port, mock_mode=False)
            self.session = utils.session

            if self.session is None:
                raise RuntimeError("Sessione qi None ricevuta da Utils")

            self._tablet = self.session.service("ALTabletService")
            self._tablet.resetTablet()
            self.get_logger().info("Tablet Pepper connesso con successo")

        except Exception as e:
            self.get_logger().error(f"Connessione a Pepper fallita: {e}")
            self.get_logger().warn("Utilizzo browser locale come fallback")
            self.mock_mode = True
            self._setup_browser()


    def _setup_browser(self):
        options = Options()
        options.add_argument("--no-sandbox")
        options.add_argument("--disable-dev-shm-usage")
        options.add_argument("--start-maximized")
        options.add_argument("--disable-gpu")
        options.add_argument("--disable-infobars")
        options.add_argument("--disable-extensions")

        try:
            self.get_logger().info("Inizializzazione browser Chrome locale")
            self._driver = webdriver.Chrome(
                service=ChromeService(ChromeDriverManager().install()),
                options=options
            )
            self.get_logger().info("Browser Chrome avviato correttamente")
        except Exception as e:
            self.get_logger().error(f"Errore durante avvio Chrome: {e}")
            self._driver = None


    def browsing_execute_callback(self, goal_handle):
        html_page = goal_handle.request.html_page
        use_tablet = goal_handle.request.use_tablet
        wait_for_data = getattr(goal_handle.request, 'wait_for_data', False)
        timeout = getattr(goal_handle.request, 'timeout', 120.0)

        self.get_logger().info(
            f"Browsing action ricevuta: {html_page}, use_tablet={use_tablet}, wait_for_data={wait_for_data}"
        )

        feedback_msg = Browsing.Feedback()
        feedback_msg.current_status = "Preparazione visualizzazione pagina"
        goal_handle.publish_feedback(feedback_msg)

        with self.tablet_lock:
            try:
                if html_page.lower() in ["hide", "about:blank"]:
                    if use_tablet and self._tablet is not None:
                        self._tablet.hideWebview()
                        self.get_logger().info("Webview nascosta su tablet Pepper")
                    elif self._driver:
                        self._driver.get("about:blank")
                        self.get_logger().info("Browser svuotato")
                    else:
                        raise RuntimeError("Nessun metodo di display disponibile")

                    feedback_msg.current_status = "Webview nascosta"
                    goal_handle.publish_feedback(feedback_msg)

                    goal_handle.succeed()
                    result = Browsing.Result()
                    result.success = True
                    result.form_data_json = ""
                    self.stats['browsing_requests'] += 1
                    return result

                if not html_page.startswith(("http://", "https://")):
                    local_ip = Utils.get_local_ip()
                    html_page = f"http://{local_ip}:{self.http_port}/{html_page}"
                    self.get_logger().info(f"URL costruito automaticamente: {html_page}")

                if use_tablet and not self.mock_mode and self._tablet:
                    self._tablet.showWebview(html_page)
                    self.get_logger().info(f"Pagina visualizzata su tablet Pepper: {html_page}")
                elif self._driver:
                    self._driver.get(html_page)
                    self.get_logger().info(f"Pagina visualizzata su browser locale: {html_page}")
                else:
                    raise RuntimeError("Nessun metodo di display disponibile")

                feedback_msg.current_status = "Pagina visualizzata"
                feedback_msg.tablet_ip = self.tablet_ip
                feedback_msg.webserver_ip = self.pepper_ip
                goal_handle.publish_feedback(feedback_msg)

                if not wait_for_data:
                    goal_handle.succeed()
                    result = Browsing.Result()
                    result.success = True
                    result.form_data_json = ""
                    self.stats['browsing_requests'] += 1
                    return result

                feedback_msg.current_status = "In attesa input utente"
                goal_handle.publish_feedback(feedback_msg)

                form_data = self._wait_for_form_data(goal_handle, html_page, timeout)

                if form_data is not None:
                    if use_tablet and self._tablet:
                        self._tablet.hideWebview()
                    elif self._driver:
                        self._driver.get("about:blank")

                    goal_handle.succeed()
                    result = Browsing.Result()
                    result.success = True
                    result.form_data_json = json.dumps(form_data)
                    self.stats['forms_collected'] += 1
                    self.get_logger().info("Dati form raccolti con successo")
                    return result
                else:
                    if use_tablet and self._tablet:
                        self._tablet.hideWebview()
                    elif self._driver:
                        self._driver.get("about:blank")

                    goal_handle.abort()
                    result = Browsing.Result()
                    result.success = False
                    result.form_data_json = ""
                    self.stats['errors'] += 1
                    return result

            except Exception as e:
                self.get_logger().error(f"Errore durante browsing action: {e}")
                goal_handle.abort()
                result = Browsing.Result()
                result.success = False
                result.form_data_json = ""
                self.stats['errors'] += 1
                return result


    def _wait_for_form_data(self, goal_handle, html_page, timeout):
        start_time = time.time()
        is_user_registration = "registrazione" in html_page.lower()
        target_queue = request_queue_user if is_user_registration else test_queue
        feedback_msg = Browsing.Feedback()

        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Browsing action cancellata")
                return None

            elapsed = time.time() - start_time
            feedback_msg.timeout_remaining = timeout - elapsed
            feedback_msg.current_status = f"Attesa dati (timeout: {feedback_msg.timeout_remaining:.1f}s)"
            goal_handle.publish_feedback(feedback_msg)

            if not target_queue.empty():
                try:
                    form_data = target_queue.get(timeout=0.1)
                    self.get_logger().info(f"Dati form ricevuti: {list(form_data.keys())}")
                    return form_data
                except Exception as e:
                    self.get_logger().error(f"Errore lettura queue: {e}")

            time.sleep(0.1)

        self.get_logger().warning(f"Timeout raggiunto dopo {timeout}s senza dati form")
        return None


    '''
    def ending_callback(self, msg):
        if self.reaction_mode != "Autonomous":
            self.get_logger().debug("Reaction disabilitata, salto ending_callback")
            return
        self.pub_tracker("Stop", 0.3)
        self.call_set_posture_sync("Stand", 0.5)
        self.tablet_on = True
        self.start = False
        self.set_led(False)
        self.send_browsing("disattivo.html", use_tablet=False)


    def onTouched(self, value):
        if self.reaction_mode != "Autonomous":
            self.get_logger().debug("Reaction disabilitata, salto onTouched")
            return

        self.get_logger().info(f"Tocco: {value}")
        if not self.start:
            self.start = True
            msg = Bool()
            if self.first:
                msg.data = True
            else:
                msg.data = False
                self.first = True
            self.start_pub.publish(msg)
            self.get_logger().info("Start")
        else:
            self.start = False
            self.set_led(False)
            self.call_set_posture_sync("Stand", 0.5)
            self.get_logger().info("Ending")

        if self.tablet_on:
            self.pub_tracker("Face", 0.8)
            self.send_browsing("attivo.html", use_tablet=False)
            self.tablet_on = False
            msg3 = Bool()
            msg3.data = True
            self.touched.publish(msg3)
        else:
            self.pub_tracker("Stop", 0.3)
            self.send_browsing("disattivo.html", use_tablet=False)
            self.tablet_on = True
            msg3 = Bool()
            msg3.data = False
            self.touched.publish(msg3)

    def condividi_risposta(self, msg):
        if self.reaction_mode != "Autonomous":
            self.get_logger().debug("Reaction disabilitata, salto condividi_risposta")
            return

        risposta = msg.data
        self.get_logger().info(f"Risposta ricevuta: {risposta}")

        if risposta == 'no':
            self.send_browsing("about:blank", use_tablet=True)

        messaggio_json = {
            "trascrizione": risposta,
            "agente": getattr(self, 'agente', 'agente'),
            "percept": getattr(self, 'percept', 'percept')
        }

        self.get_logger().info(f"Risposta BDI preparata: {messaggio_json}")

    def stt_bdi_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.agente = data.get("agente", "agente")
            self.percept = data.get("percept", "percept")
            self.get_logger().info(f"BDI - Agente: {self.agente}, Percept: {self.percept}")
            self.send_browsing("si_no.html", use_tablet=True)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Errore parsing JSON: {e}")

    '''


    def destroy_node(self):
        self.http_manager.shutdown()
        if self._driver:
            self._driver.quit()
        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = QiUnipa2_server()
        logger.info("Server ROS2 avviato")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Arresto richiesto da utente")
    except Exception as e:
        logger.error(f"Errore nel main: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        logger.info("Server terminato")

if __name__ == '__main__':
    main()

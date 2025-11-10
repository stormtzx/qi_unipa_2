"""
Nodo ROS2 server unificato per gestione form HTML e display tablet.
- HTTP server per servire pagine HTML e raccogliere dati form
- Browsing action estesa per mostrare pagine e opzionalmente raccogliere dati
- Elimina necessità di nodo tablet separato e 13+ topics
"""

import qi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from qi_unipa_2_interfaces.action import Browsing
import threading
import json
import logging
import time

# Selenium per browser locale
from selenium import webdriver
from selenium.webdriver.chrome.service import Service as ChromeService
from selenium.webdriver.chrome.options import Options
from webdriver_manager.chrome import ChromeDriverManager

# Import da moduli locali
from qi_unipa_2.http_utils import ( #type: ignore
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
        
        # Dichiarazione parametri
        self.declare_parameter('http_port', DEFAULT_PORT)
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('pepper_ip', '192.168.0.102')
        self.declare_parameter('pepper_port', 9559)
        
        # Lettura parametri
        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self.pepper_ip = self.get_parameter('pepper_ip').get_parameter_value().string_value
        self.pepper_port = self.get_parameter('pepper_port').get_parameter_value().integer_value
        
        self.get_logger().info("Nodo Server ROS avviato.")
        
        # Log modalità
        if self.mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
        
        # Statistiche
        self.stats = {
            'browsing_requests': 0,
            'forms_collected': 0,
            'errors': 0
        }

        # Avvio HTTP server in thread separato
        self.http_manager = HTTPServerManager(port=self.http_port)
        self.server_thread = threading.Thread(
            target=self.http_manager.start,
            kwargs={'max_retries': 5},
            daemon=True
        )
        self.server_thread.start()

        # Setup display (tablet Pepper o browser locale)
        self.tablet_lock = threading.Lock()
        self._tablet = None
        self._driver = None
        self.tablet_ip = ""
        self.session = None
        self._setup_display()

        # Action server Browsing estesa
        self._browsing_action = ActionServer(
            self,
            Browsing,
            '/pepper/actions/browsing',
            execute_callback=self.browsing_execute_callback
        )

        self.get_logger().info("Server con Browsing action estesa inizializzato.")



    def _setup_display(self):
        """
        Inizializza display per output HTML.
        Tenta connessione a tablet Pepper; fallback a browser locale se non disponibile.
        """
        if self.mock_mode:
            self.get_logger().info("MOCK MODE ATTIVO - Utilizzo browser locale")
            self._setup_browser()
            return  # Esci subito senza tentare connessione Pepper
        
        # Solo se NON mock mode, tenta connessione Pepper
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
        """Configura e avvia Selenium Chrome WebDriver per display locale"""
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
        """
        Callback per action Browsing estesa.
        Mostra pagina HTML su tablet/browser e opzionalmente aspetta dati form.
        
        Args:
            goal_handle: Handle con request contenente:
                - html_page: URL completo o nome file (es: "registrazione.html")
                - use_tablet: bool per usare tablet Pepper vs browser locale
                - wait_for_data: bool per aspettare dati form prima di completare
                - timeout: timeout in secondi per raccolta dati (default 120.0)
        
        Returns:
            Browsing.Result con success e form_data_json
        """
        html_page = goal_handle.request.html_page
        use_tablet = goal_handle.request.use_tablet
        wait_for_data = goal_handle.request.wait_for_data if hasattr(goal_handle.request, 'wait_for_data') else False
        timeout = goal_handle.request.timeout if hasattr(goal_handle.request, 'timeout') else 120.0

        self.get_logger().info(
            f"Browsing action ricevuta: {html_page}, "
            f"use_tablet={use_tablet}, wait_for_data={wait_for_data}"
        )

        feedback_msg = Browsing.Feedback()
        feedback_msg.current_status = "Preparazione visualizzazione pagina"
        goal_handle.publish_feedback(feedback_msg)

        with self.tablet_lock:
            try:
                # Gestione nascondimento webview
                if html_page.lower() == "hide" or html_page.lower() == "about:blank":
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

                # Costruisci URL completo se necessario
                if not html_page.startswith("http://") and not html_page.startswith("https://"):
                    local_ip = Utils.get_local_ip()
                    html_page = f"http://{local_ip}:{self.http_port}/{html_page}"
                    self.get_logger().info(f"URL costruito automaticamente: {html_page}")

                # Visualizza pagina su tablet o browser
                if use_tablet and not self.mock_mode and self._tablet is not None:
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

                # Se non serve aspettare dati form, completa subito
                if not wait_for_data:
                    goal_handle.succeed()
                    result = Browsing.Result()
                    result.success = True
                    result.form_data_json = ""
                    self.stats['browsing_requests'] += 1
                    return result

                # Aspetta dati da HTTP queue
                feedback_msg.current_status = "In attesa input utente"
                goal_handle.publish_feedback(feedback_msg)

                form_data = self._wait_for_form_data(goal_handle, html_page, timeout)

                if form_data is not None:
                    # Nascondi form dopo ricezione dati
                    if use_tablet and self._tablet is not None:
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
                    # Timeout o cancellazione
                    if use_tablet and self._tablet is not None:
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
        """
        Aspetta dati form da HTTP queue con timeout e feedback.
        
        Args:
            goal_handle: Handle action per inviare feedback
            html_page: URL pagina per determinare queue da monitorare
            timeout: Timeout in secondi
        
        Returns:
            dict con dati form o None se timeout/cancellato
        """
        start_time = time.time()

        # Determina quale queue monitorare in base al tipo di pagina
        is_user_registration = "registrazione" in html_page.lower()
        target_queue = request_queue_user if is_user_registration else test_queue

        feedback_msg = Browsing.Feedback()

        while time.time() - start_time < timeout:
            # Controlla se action cancellata
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Browsing action cancellata")
                return None

            # Aggiorna feedback con countdown
            elapsed = time.time() - start_time
            feedback_msg.timeout_remaining = timeout - elapsed
            feedback_msg.current_status = f"Attesa dati (timeout: {feedback_msg.timeout_remaining:.1f}s)"
            goal_handle.publish_feedback(feedback_msg)

            # Controlla queue per dati
            if not target_queue.empty():
                try:
                    form_data = target_queue.get(timeout=0.1)
                    self.get_logger().info(f"Dati form ricevuti: {list(form_data.keys())}")
                    return form_data
                except Exception as e:
                    self.get_logger().error(f"Errore lettura queue: {e}")
                    continue

            time.sleep(0.1)

        # Timeout raggiunto
        self.get_logger().warning(f"Timeout raggiunto dopo {timeout}s senza dati form")
        return None

    def destroy_node(self):
        """Cleanup risorse prima di terminare nodo"""
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

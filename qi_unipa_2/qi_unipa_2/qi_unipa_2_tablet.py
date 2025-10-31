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
#NB: Attenzione: verificare sempre la versione dei webdriver del browser


class QiUnipa2_tablet(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_tablet')
        
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

        # Connessione condizionale
        if mock_mode:
            self.get_logger().warn("MOCK MODE ATTIVO - Nessuna connessione a Pepper")
            self.session = None
            self._tablet = None
        else:
            try:
                self.get_logger().info(f"Tentativo connessione a Pepper: {ip}:{port}")
                self.session = utils.session
                
                if self.session is None:
                    raise RuntimeError("Sessione None da utils")
                
                self._tablet = self.session.service("ALTabletService")
                self.get_logger().info("Connesso a Pepper!")
            except Exception as e:
                self.get_logger().error(f"Impossibile connettersi a Pepper: {e}")
                self.get_logger().warn("Passaggio automatico a MOCK MODE")
                self.session = None
                self._tablet = None

        # Sottoscrizioni ROS
        self.show_sub = self.create_subscription(String, "/show", self.show, 10)
        self.hide_sub = self.create_subscription(Bool, "/hide", self.hide, 10)
        self.ip_sub = self.create_subscription(String, "/ip", self.set_ip, 10)

        # Variabili di stato
        self.webserver_ip = ""
        self.tablet_lock = threading.Lock()
        self.last_url = ""
        self.driver = None

        self.get_logger().info("Nodo tablet avviato correttamente")

        # Configurazione browser Chrome
        self.options = Options()
        self.options.add_argument("--no-sandbox")
        self.options.add_argument("--disable-dev-shm-usage")
        self.options.add_argument("--start-maximized")
        self.options.add_argument("--disable-gpu")
        self.options.add_argument("--disable-infobars")
        self.options.add_argument("--disable-extensions")

        # Se vuoi farlo partire in background, scommenta la riga seguente:
        # self.options.add_argument("--headless")

        self.setup_browser()



    def setup_browser(self):
        """Avvia il browser locale (Chrome) se in modalità MOCK o come fallback."""
        try:
            self.get_logger().info("Avvio del browser locale (Chrome) con Selenium...")
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
        self.get_logger().debug(f"IP webserver impostato: {self.webserver_ip}") #Modificato in debug per ridurre log inutile


    def show(self, msg):
        """Mostra la pagina richiesta sul tablet Pepper o nel browser locale."""
        with self.tablet_lock:
            try:
                webPage = msg.data
                full_url = f"{self.webserver_ip}/{webPage}"
                self.get_logger().info(f"Richiesta visualizzazione: {full_url}")
                
                if self._tablet is not None:
                    # Modalità REALE: Usa tablet di Pepper
                    try:
                        self.get_logger().info("[PEPPER TABLET] Caricamento pagina sul tablet di Pepper...")
                        self._tablet.showWebview()
                        self._tablet.loadUrl(full_url)
                        self.last_url = full_url
                        self.get_logger().info(f"Pagina caricata sul tablet di Pepper: {full_url}")
                    except Exception as e:
                        self.get_logger().error(f"Errore con tablet Pepper: {e}")
                        # Fallback al browser locale
                        if self.driver:
                            self.get_logger().info("[FALLBACK] Uso browser locale...")
                            self.driver.get(full_url)
                else:
                    # Modalità MOCK: Usa browser locale
                    if self.driver:
                        self.get_logger().info("[MOCK - BROWSER LOCALE] Caricamento pagina in Chrome...")
                        self.driver.get(full_url)
                        self.last_url = full_url
                        self.get_logger().info(f"Pagina caricata nel browser locale: {full_url}")
                    else:
                        self.get_logger().warn("[MOCK] Nessun browser disponibile per visualizzare la pagina")
               
            except Exception as e:
                self.get_logger().error(f"Errore durante la funzione show: {e}")


    def hide(self, msg):
        """Nasconde la pagina visualizzata."""
        with self.tablet_lock:
            try:
                if self._tablet is not None:
                    # Modalità REALE: Nascondi tablet Pepper
                    try:
                        self.get_logger().info("[PEPPER TABLET] Nascondo webview...")
                        self._tablet.hideWebview()
                    except Exception as e:
                        self.get_logger().error(f"Errore nascondendo tablet Pepper: {e}")
                
                # Pulizia browser locale (sia in MOCK che come fallback)
                if self.driver:
                    self.get_logger().info("[BROWSER LOCALE] Pulisco la schermata di Chrome.")
                    self.driver.get("about:blank")
            except Exception as e:
                self.get_logger().error(f"Errore durante hide: {e}")


    def cleanup(self):
        """Pulizia risorse prima di terminare il nodo."""
        self.get_logger().info("Esecuzione pulizia nodo...")
        
        # Pulizia tablet Pepper
        if self._tablet is not None:
            try:
                self._tablet.hideWebview()
                self.get_logger().info("Webview Pepper nascosta")
            except Exception as e:
                self.get_logger().warn(f"Impossibile nascondere la webview: {e}")

        # Pulizia browser locale
        if self.driver:
            self.get_logger().info("Chiusura del browser locale Selenium (Chrome)...")
            try:
                self.driver.quit()
            except Exception as e:
                self.get_logger().warn(f"Errore chiudendo Chrome: {e}")


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

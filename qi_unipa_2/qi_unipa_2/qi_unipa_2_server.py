import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from qi_unipa_2_interfaces.action import Browsing
import threading
import json
import logging
import time
import webbrowser
import os
from std_msgs.msg import String
from qi_unipa_2.http_utils import (
    HTTPServerManager,
    request_queue_user,
    test_queue,
    DEFAULT_PORT
)


logger = logging.getLogger(__name__)


class QiUnipa2_server(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_server')
        
        self.declare_parameter('http_port', DEFAULT_PORT)
        self.declare_parameter('http_host', 'localhost')
        self.declare_parameter('tablet_ip', '192.168.1.100')
        self.declare_parameter('mock_mode', False)  # Modalità mock
        
        http_port = self.get_parameter('http_port').value
        self.http_host = self.get_parameter('http_host').value
        self.tablet_ip = self.get_parameter('tablet_ip').value
        self.mock_mode = self.get_parameter('mock_mode').value
        
        # URL base per accedere alle pagine
        self.base_url = f"http://{self.http_host}:{http_port}"
        
        self.user_update_pub = self.create_publisher(
            String,
            'pepper/topics/user_update',
            10
        )
        
        self.form_data_pub = self.create_publisher(
            String,
            'pepper/topics/form_data',
            10
        )
        
        self.page_navigation = {
            "registrazione.html": {"next": "login.html", "queue": request_queue_user},
            "login.html": {"next": "attivo.html", "queue": test_queue},
            "attivo.html": {"next": "vuoi_effettuare_il_test.html", "queue": test_queue},
            "vuoi_effettuare_il_test.html": {"next": "minicog_orologio.html", "queue": test_queue},
            "minicog_orologio.html": {"next": "minicog_parole.html", "queue": test_queue},
            "minicog_parole.html": {"next": "minicog_parole_verifica.html", "queue": test_queue},
            "minicog_parole_verifica.html": {"next": "test_emoji.html", "queue": test_queue},
            "test_emoji.html": {"next": "thinking.html", "queue": test_queue},
            "thinking.html": {"next": None, "queue": test_queue},
            "dati_must.html": {"next": None, "queue": test_queue},
            "si_no.html": {"next": None, "queue": test_queue},
            "disattivo.html": {"next": None, "queue": test_queue},
            "autoValutazione_3_opzioni.html": {"next": None, "queue": test_queue},
            "autoValutazione_5_opzioni.html": {"next": None, "queue": test_queue},
        }
        
        self.browsing_server = ActionServer(
            self,
            Browsing,
            'pepper/actions/browsing',
            self.browsing_execute_callback
        )
        
        self.http_server_manager = HTTPServerManager(
              port=http_port,
              html_dir="/home/daniele/Scrivania/ros2_ws/src/qi_unipa_2/html_pages" 
        )
        
        self.http_server_thread = threading.Thread(
            target=self.http_server_manager.start,
            daemon=True
        )
        self.http_server_thread.start()
        
        self.get_logger().info(f"QiUnipa2_server inizializzato")
        self.get_logger().info(f"HTTP server: {self.base_url}")
        self.get_logger().info(f"Tablet IP: {self.tablet_ip}")
        self.get_logger().info(f"Modalità mock: {'ABILITATA' if self.mock_mode else 'DISABILITATA'}")
    
    def browsing_execute_callback(self, goal_handle):
        html_page = goal_handle.request.html_page
        use_tablet = goal_handle.request.use_tablet
        wait_for_data = goal_handle.request.wait_for_data
        timeout = goal_handle.request.timeout if goal_handle.request.timeout > 0 else 120.0
        
        self.get_logger().info(
            f"Browsing action: pagina={html_page}, tablet={use_tablet}, "
            f"attendi_dati={wait_for_data}, timeout={timeout}s"
        )
        
        page_info = self.page_navigation.get(html_page)
        if not page_info:
            self.get_logger().warn(f"Pagina sconosciuta: {html_page}")
            result = Browsing.Result()
            result.success = False
            result.form_data_json = "{}"
            goal_handle.abort()
            return result
        
        # Costruisci URL completo
        page_url = f"{self.base_url}/{html_page}"
        
        # In modalità mock, apri il browser automaticamente
        if self.mock_mode:
            self._open_browser_mock(page_url)
        
        # Feedback iniziale con URL
        feedback_msg = Browsing.Feedback()
        feedback_msg.current_status = f"Caricamento pagina: {html_page}"
        
        if self.mock_mode:
            feedback_msg.tablet_ip = "MOCK (Browser locale)"
        elif use_tablet:
            feedback_msg.tablet_ip = self.tablet_ip
        else:
            feedback_msg.tablet_ip = "N/A"
            
        feedback_msg.webserver_ip = self.http_host
        feedback_msg.timeout_remaining = timeout
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info(f"URL pagina: {page_url}")
        
        result = Browsing.Result()
        result.success = True
        
        # Se wait_for_data è False, return subito con URL
        if not wait_for_data:
            result.form_data_json = json.dumps({"page_url": page_url})
            self.get_logger().info(f"Pagina caricata senza attesa dati: {page_url}")
            goal_handle.succeed()
            return result
        
        # Altrimenti attendi i dati dal form
        target_queue = page_info.get("queue")
        form_data = self._wait_for_form_data(
            goal_handle, 
            html_page, 
            timeout, 
            target_queue
        )
        
        if form_data:
            # Aggiungi URL ai dati del form
            form_data["page_url"] = page_url
            result.success = True
            result.form_data_json = json.dumps(form_data)
            self.get_logger().info(f"Dati form raccolti: {list(form_data.keys())}")
            
            self._publish_form_data(form_data, html_page)
            
            goal_handle.succeed()
        else:
            result.success = False
            result.form_data_json = json.dumps({"page_url": page_url, "error": "timeout"})
            self.get_logger().warn(f"Timeout nella raccolta dati per {html_page}")
            goal_handle.abort()
        
        return result
    
    def _open_browser_mock(self, page_url):
        """Apre il browser locale in modalità mock"""
        try:
            self.get_logger().info(f"[MOCK] Apertura browser con URL: {page_url}")
            
            # Usa webbrowser per aprire in background
            threading.Thread(
                target=webbrowser.open,
                args=(page_url,),
                daemon=True
            ).start()
            
            print(f"\n{'='*70}")
            print(f"📱 MODALITÀ MOCK - BROWSER APERTO")
            print(f"{'='*70}")
            print(f"URL: {page_url}")
            print(f"{'='*70}\n")
            
        except Exception as e:
            self.get_logger().error(f"Errore apertura browser: {e}")
            print(f"⚠️ Impossibile aprire browser automaticamente. Apri manualmente: {page_url}")
        
    def _wait_for_form_data(self, goal_handle, html_page, timeout, target_queue):
        """Attendi dati form con feedback di timeout"""
        start_time = time.time()
        check_interval = 0.5
        
        while time.time() - start_time < timeout:
            if target_queue and not target_queue.empty():
                try:
                    form_data_raw = target_queue.get(timeout=0.1)
                    self.get_logger().info(f"Dati form grezzi ricevuti: {form_data_raw}")
                    
                    form_data = self._normalize_form_data(form_data_raw)
                    self.get_logger().info(f"Dati form normalizzati: {form_data}")
                    return form_data
                except Exception as e:
                    self.get_logger().error(f"Errore lettura queue: {e}")
                    return None
            
            # Pubblica feedback con timeout rimanente
            elapsed = time.time() - start_time
            remaining = timeout - elapsed
            if remaining % check_interval < 0.1:
                feedback_msg = Browsing.Feedback()
                feedback_msg.current_status = f"In attesa di dati da {html_page}"
                feedback_msg.timeout_remaining = max(0, remaining)
                feedback_msg.tablet_ip = self.tablet_ip if hasattr(self, 'tablet_ip') else "N/A"
                feedback_msg.webserver_ip = self.http_host if hasattr(self, 'http_host') else "N/A"
                goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.1)
        
        self.get_logger().warn(f"Timeout: nessun dato ricevuto da {html_page}")
        return None

    def _normalize_form_data(self, form_data_raw):
        """Normalizza dati form da liste a singoli valori"""
        normalized = {}
        for key, value in form_data_raw.items():
            if isinstance(value, list) and len(value) > 0:
                normalized[key] = value[0]
            else:
                normalized[key] = value
        return normalized
    
    def _publish_form_data(self, form_data, html_page):
        """Pubblica dati form su topic dedicato"""
        msg = String()
        
        if "registrazione" in html_page.lower():
            msg.data = (
                f"nome: {form_data.get('nome', '')}, "
                f"cognome: {form_data.get('cognome', '')}, "
                f"peso: {form_data.get('peso', '')}, "
                f"altezza: {form_data.get('altezza', '')}, "
                f"patologie: {form_data.get('patologie', '')}, "
                f"intolleranze: {form_data.get('intolleranze', '')}"
            )
            
            self.user_update_pub.publish(msg)
            self.get_logger().info(f"Pubblicato su pepper/topics/user_update: {msg.data}")
        else:
            msg.data = json.dumps(form_data)
            self.form_data_pub.publish(msg)
            self.get_logger().info(f"Pubblicato su pepper/topics/form_data: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    
    server_node = QiUnipa2_server()
    
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

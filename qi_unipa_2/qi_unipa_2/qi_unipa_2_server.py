import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from qi_unipa_2_interfaces.action import Browsing
import threading
import json
import logging
import time
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
        http_port = self.get_parameter('http_port').value
        
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
              #Valutare se rendere dinamica questa variabile, con directory assegnabile "dinamicamente"
              html_dir="/home/daniele/Scrivania/ros2_ws/src/qi_unipa_2/html_pages" 
        )
        
        self.http_server_thread = threading.Thread(
            target=self.http_server_manager.start,
            daemon=True
        )
        self.http_server_thread.start()
        
        self.get_logger().info(f"QiUnipa2_server inizializzato")
        self.get_logger().info(f"HTTP server su porta {http_port}")
        self.get_logger().info(f"Navigazione gestita lato server HTTP")
    
    def browsing_execute_callback(self, goal_handle):
        self.get_logger().info(f"Browsing action ricevuta: {goal_handle.request.html_page}")
        
        feedback_msg = Browsing.Feedback()
        html_page = goal_handle.request.html_page
        timeout = goal_handle.request.timeout if hasattr(goal_handle.request, 'timeout') else 60.0
        
        feedback_msg.status = "Pagina caricata, in attesa di input utente"
        goal_handle.publish_feedback(feedback_msg)
        
        page_info = self.page_navigation.get(html_page)
        if not page_info:
            self.get_logger().warn(f"Pagina sconosciuta: {html_page}")
            result = Browsing.Result()
            result.success = False
            result.form_data = "{}"
            goal_handle.abort()
            return result
        
        target_queue = page_info.get("queue")
        form_data = self._wait_for_form_data(goal_handle, html_page, timeout, target_queue)
        
        result = Browsing.Result()
        
        if form_data:
            result.success = True
            result.form_data = json.dumps(form_data)
            self.get_logger().info(f"Dati form raccolti: {list(form_data.keys())}")
            
            self._publish_form_data(form_data, html_page)
            
            goal_handle.succeed()
        else:
            result.success = False
            result.form_data = "{}"
            self.get_logger().warn(f"Timeout o errore nella raccolta dati")
            goal_handle.abort()
        
        return result
        
    def _wait_for_form_data(self, goal_handle, html_page, timeout, target_queue):
        start_time = time.time()
        
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
            
            time.sleep(0.1)
        
        self.get_logger().warn(f"Timeout: nessun dato ricevuto da {html_page}")
        return None

    def _normalize_form_data(self, form_data_raw):
        normalized = {}
        for key, value in form_data_raw.items():
            if isinstance(value, list) and len(value) > 0:
                normalized[key] = value[0]
            else:
                normalized[key] = value
        return normalized
    

    
    def _publish_form_data(self, form_data, html_page):
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

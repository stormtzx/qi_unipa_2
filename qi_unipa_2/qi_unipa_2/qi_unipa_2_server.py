import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from qi_unipa_2_interfaces.action import Browsing
from qi_unipa_2_interfaces.msg import UserData, TestData
import threading
import json
import logging
import webbrowser
import datetime
import time
from qi_unipa_2.utils import Utils #type: ignore
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
        self.declare_parameter('mock_mode', False)
        
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        
        http_port = self.get_parameter('http_port').value
        self.http_host = self.get_parameter('http_host').value
        self.tablet_ip = self.get_parameter('tablet_ip').value
        self.mock_mode = self.get_parameter('mock_mode').value
        
        # Crea Utils        
        # Recupera IP e Port per Pepper dai parametri in input
        pepper_ip = self.get_parameter('ip').value
        pepper_port = self.get_parameter('port').value
        
        # Passa l'ip e porta anche ad a Utils
        utils = Utils(
            mock_mode=self.mock_mode,
            ip=pepper_ip,           # ← Passa IP dal parametro
            port=pepper_port        # ← Passa Port dal parametro
        )
        

        # URL base per accedere alle pagine
        self.base_url = f"http://{self.http_host}:{http_port}"
        
        # ========== PUBLISHERS ==========
        
        # Publisher per dati utente (registrazione)
        self.user_data_pub = self.create_publisher(
            UserData, 
            '/pepper/topics/user_data', 
            10
        )
        
        # Publisher per dati test (minicog, must, mobilità)
        self.test_data_pub = self.create_publisher(
            TestData, 
            '/pepper/topics/test_data', 
            10
        )

        # NAVIGAZIONE AUTOMATICA
        self.page_navigation = {
            # Registrazione
            "registrazione.html": "registrazione_completata.html",
            
            # MiniCog
            "test_minicog_parte1.html": "test_minicog_parte2.html",
            "test_minicog_parte2.html": "test_minicog_parte3.html",
            "test_minicog_parte3.html": "test_minicog_parte4.html",
            "test_minicog_parte4.html": "test_minicog_completato.html",
            "test_minicog_completato.html": None,  # Fine test - binario morto
            
            # Mobilità
            "test_mobilita_parte1.html": "test_mobilita_parte2.html",
            "test_mobilita_parte2.html": "test_mobilita_parte3.html",
            "test_mobilita_parte3.html": "test_mobilita_parte4.html",
            "test_mobilita_parte4.html": "test_mobilita_parte5.html",
            "test_mobilita_parte5.html": "test_mobilita_completato.html",
            "test_mobilita_completato.html": None,  # Fine test - binario morto
            
            # MUST
            "test_MUST_parte1.html": "test_MUST_parte2.html",
            "test_MUST_parte2.html": "test_MUST_parte3.html",
            "test_MUST_parte3.html": "test_must_completato.html",
            "test_must_completato.html": None,  # Fine test - binario morto
        }
        
        # Action server Browsing
        self.browsing_server = ActionServer(
            self,
            Browsing,
            '/pepper/actions/browsing',
            self.browsing_execute_callback
        )
        
        # Action client per auto-navigazione
        self.browsing_client = ActionClient(self, Browsing, '/pepper/actions/browsing')
        
        # HTTP Server
        self.http_server_manager = HTTPServerManager(
            port=http_port,
            html_dir="/home/daniele/Scrivania/ros2_ws/src/qi_unipa_2/html_pages" 
        )
        
        self.http_server_thread = threading.Thread(
            target=self.http_server_manager.start,
            daemon=True
        )
        self.http_server_thread.start()
        
        # Queue processor thread
        self.queue_processor_thread = threading.Thread(
            target=self._process_queues,
            daemon=True
        )
        self.queue_processor_thread.start()
        
        if self.mock_mode:
            self.get_logger().info(f"Nodo SERVER attivo in MOCK MODE")
        else:
            self.get_logger().info(f"Nodo SERVER attivo e connesso a Pepper all'indirizzo-> {pepper_ip}:{pepper_port}")
            
        self.get_logger().info(f"HTTP server: {self.base_url}, tablet IP: {self.tablet_ip}")
    

    def _process_queues(self):
        """Thread che legge dalle queue e pubblica su topic ROS2 separati"""
        while True:
            try:
                # DEBUG: Vedi dimensione queue
                user_queue_size = request_queue_user.qsize()
                test_queue_size = test_queue.qsize()
                
                if user_queue_size > 0 or test_queue_size > 0:
                    self.get_logger().info(
                        f" Queue sizes - user: {user_queue_size}, test: {test_queue_size}"
                    )
                
                # ========== QUEUE REGISTRAZIONE UTENTE ==========
                last_user_message = None
                messages_read = 0
                
                while not request_queue_user.empty():
                    try:
                        last_user_message = request_queue_user.get(timeout=0.1)
                        messages_read += 1
                        self.get_logger().info(f" Letto messaggio #{messages_read} dalla user queue")
                    except Exception as e:
                        self.get_logger().error(f" Errore lettura user queue: {e}")
                        break
                
                if last_user_message:
                    if messages_read > 1:
                        self.get_logger().warn(
                            f"️ request_queue_user aveva {messages_read} msg - "
                            f"scartati {messages_read-1} vecchi"
                        )
                    
                    self.get_logger().info(f" Processando dati utente: {last_user_message}")
                    
                    # Normalizza
                    if isinstance(last_user_message, dict) and any(isinstance(v, list) for v in last_user_message.values()):
                        last_user_message = self._normalize_form_data(last_user_message)
                    
                    # Pubblica UserData
                    self._publish_user_data(last_user_message)
                
                # ========== QUEUE TEST ==========
                last_test_message = None
                test_messages_read = 0
                
                while not test_queue.empty():
                    try:
                        last_test_message = test_queue.get(timeout=0.1)
                        test_messages_read += 1
                        self.get_logger().info(f"Letto messaggio #{test_messages_read} dalla test queue")
                    except Exception as e:
                        self.get_logger().error(f" Errore lettura test queue: {e}")
                        break
                
                if last_test_message:
                    if test_messages_read > 1:
                        self.get_logger().warn(
                            f"️ test_queue aveva {test_messages_read} msg - "
                            f"scartati {test_messages_read-1} vecchi"
                        )
                    
                    self.get_logger().info(f"Processando dati test: {last_test_message}")
                    
                    if isinstance(last_test_message, dict) and any(isinstance(v, list) for v in last_test_message.values()):
                        last_test_message = self._normalize_form_data(last_test_message)
                    
                    # Pubblica TestData
                    formtype = last_test_message.get('formtype', 'unknown')
                    self._publish_test_data(last_test_message, formtype)
                
                time.sleep(0.05)
            
            except Exception as e:
                self.get_logger().error(f" Errore queue processor: {e}")
                time.sleep(1)


    def browsing_execute_callback(self, goal_handle):
        """
        Action Browsing: carica pagina HTML e ritorna subito.
        I dati del form arrivano in modo asincrono su topic separati.
        """
        html_page = goal_handle.request.html_page
        use_tablet = goal_handle.request.use_tablet
        
        self.get_logger().info(
            f" Browsing action: pagina={html_page}, tablet={use_tablet}"
        )
        
        # Costruisci URL completo
        page_url = f"{self.base_url}/{html_page}"
        
        # In modalità mock, apri il browser automaticamente
        
        self._open_browser_mock(page_url)
        
        # Feedback: pagina caricata
        feedback_msg = Browsing.Feedback()
        feedback_msg.current_status = f"Pagina caricata: {html_page}"
        feedback_msg.tablet_ip = self.tablet_ip if use_tablet else "N/A"
        feedback_msg.webserver_ip = self.http_host
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info(f" URL pagina: {page_url}")
        
        # RITORNA SUBITO - No timeout, no attesa
        result = Browsing.Result()
        result.success = True
        result.form_data_json = json.dumps({"page_url": page_url, "status": "loaded"})
        
        self.get_logger().info(f" Action completata: {html_page}")
        goal_handle.succeed()
        
        return result
    
    
    def _open_browser_mock(self, page_url):
        """Apre il browser locale in modalità mock"""
        try:
            self.get_logger().info(f"[MOCK] Apertura browser con URL: {page_url}")
            
            threading.Thread(
                target=webbrowser.open,
                args=(page_url,),
                daemon=True
            ).start()
            
            print(f"\n{'='*70}")
            print(f" MODALITÀ MOCK - BROWSER APERTO")
            print(f"{'='*70}")
            print(f"URL: {page_url}")
            print(f"{'='*70}\n")
            
        except Exception as e:
            self.get_logger().error(f" Errore apertura browser: {e}")
            print(f"️ Impossibile aprire browser automaticamente. Apri manualmente: {page_url}")
    
    
    def _normalize_form_data(self, form_data_raw):
        """Normalizza dati form da liste a singoli valori"""
        normalized = {}
        for key, value in form_data_raw.items():
            if isinstance(value, list) and len(value) > 0:
                normalized[key] = value[0]
            else:
                normalized[key] = value
        return normalized
    

    # =========================================================================
    # PUBBLICAZIONE DATI UTENTE (Registrazione)
    # =========================================================================
    
    def _publish_user_data(self, user_data):
        """
        Pubblica dati utente (registrazione) sul topic UserData
        
        Args:
            user_data: dict con dati registrazione
        """
        msg = UserData()
        
        # Metadata
        msg.timestamp = datetime.datetime.now().isoformat()
        msg.success = True
        
        # Dati anagrafici
        msg.nome = str(user_data.get('nome', ''))
        msg.cognome = str(user_data.get('cognome', ''))
        
        try:
            msg.peso = float(user_data.get('peso', 0.0))
        except (ValueError, TypeError):
            msg.peso = 0.0
        
        try:
            msg.altezza = float(user_data.get('altezza', 0.0))
        except (ValueError, TypeError):
            msg.altezza = 0.0
        
        msg.patologie = str(user_data.get('patologie', ''))
        msg.intolleranze = str(user_data.get('intolleranze', ''))
        
        # Pubblica
        self.user_data_pub.publish(msg)
        
        self.get_logger().info(
            f" UserData pubblicato - Nome: {msg.nome} {msg.cognome}, "
            f"Peso: {msg.peso}kg, Altezza: {msg.altezza}cm"
        )


    # =========================================================================
    # PUBBLICAZIONE DATI TEST (MiniCog, MUST, Mobilità)
    # =========================================================================
    
    def _publish_test_data(self, test_data, formtype):
        """
        Pubblica dati test sul topic TestData
        
        Args:
            test_data: dict con dati test
            formtype: tipo di form/test
        """
        msg = TestData()
        
        # Metadata
        msg.timestamp = datetime.datetime.now().isoformat()
        msg.success = True
        msg.test_type = formtype
        
        self.get_logger().info(f"Processing test_type: {formtype}")
        
        current_page = None
        
        # ========== TEST MINICOG ==========
        if formtype == "minicog_parte2":
            msg.minicog_parte2_score = int(test_data.get('score', 0))
            msg.minicog_parte2_ore = int(test_data.get('ore', 0))
            msg.minicog_parte2_minuti = int(test_data.get('minuti', 0))
            current_page = "test_minicog_parte2.html"
            msg.test_name = "MiniCog"
            msg.page_name = current_page
            
            self.get_logger().info(f" MINICOG PARTE 2 - Score: {msg.minicog_parte2_score}, Ora: {msg.minicog_parte2_ore}:{msg.minicog_parte2_minuti:02d}")
        
        elif formtype == "minicog_parte3":
            msg.minicog_parte3_score = 0 
            current_page = "test_minicog_parte3.html"
            msg.test_name = "MiniCog"
            msg.page_name = current_page
            
            self.get_logger().info(f" MINICOG PARTE 3 - Score: 2 (fisso)")
        
        elif formtype == "minicog_parte4":
            msg.minicog_parte4_score = int(test_data.get('score', 0))
            msg.minicog_parte4_corrette = int(test_data.get('corrette', 0))
            msg.minicog_parte4_sbagliate = int(test_data.get('sbagliate', 0))
            
            # Parole selezionate (array)
            selezionate = test_data.get('parole_selezionate', [])
            if isinstance(selezionate, str):
                selezionate = selezionate.split(',')
            msg.minicog_parte4_selezionate = selezionate
            
            # Calcola totale (parte2 + parte3 + parte4)
            # Nota: parte2 e parte3 sono già stati inviati prima, qui calcoliamo solo parziale
            msg.minicog_punteggio_totale = msg.minicog_parte4_score  # Sarà sommato dopo
            
            current_page = "test_minicog_parte4.html"
            msg.test_name = "MiniCog"
            msg.page_name = current_page
            
            self.get_logger().info(f" MINICOG PARTE 4 - Score: {msg.minicog_parte4_score}, Corrette: {msg.minicog_parte4_corrette}/3")
        
        elif formtype == "minicog_complete":
            # Riepilogo finale
            msg.minicog_punteggio_totale = int(test_data.get('punteggio_totale', 0))
            current_page = "test_minicog_completato.html"
            msg.test_name = "MiniCog"
            msg.page_name = current_page
            
            self.get_logger().info(f" MINICOG COMPLETO - Totale: {msg.minicog_punteggio_totale}/14")
        
        # ========== TEST MOBILITÀ ==========
        elif formtype == "mobilita_test1":
            msg.mobilita_test1_score = int(test_data.get('score', 0))
            msg.mobilita_test1_stato = str(test_data.get('stato', ''))
            current_page = "test_mobilita_parte1.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA TEST 1 - Score: {msg.mobilita_test1_score}, Stato: {msg.mobilita_test1_stato}")
        
        elif formtype == "mobilita_test2":
            msg.mobilita_test2_score = int(test_data.get('score', 0))
            msg.mobilita_test2_stato = str(test_data.get('stato', ''))
            current_page = "test_mobilita_parte2.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA TEST 2 - Score: {msg.mobilita_test2_score}, Stato: {msg.mobilita_test2_stato}")
        
        elif formtype == "mobilita_test3":
            msg.mobilita_test3_score = int(test_data.get('score', 0))
            msg.mobilita_test3_stato = str(test_data.get('stato', ''))
            current_page = "test_mobilita_parte3.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA TEST 3 - Score: {msg.mobilita_test3_score}, Stato: {msg.mobilita_test3_stato}")
        
        elif formtype == "mobilita_test4":
            msg.mobilita_test4_score = int(test_data.get('score', 0))
            msg.mobilita_test4_stato = str(test_data.get('stato', ''))
            current_page = "test_mobilita_parte4.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA TEST 4 - Score: {msg.mobilita_test4_score}, Stato: {msg.mobilita_test4_stato}")
        
        elif formtype == "mobilita_test5":
            msg.mobilita_test5_score = int(test_data.get('score', 0))
            msg.mobilita_test5_stato = str(test_data.get('stato', ''))
            current_page = "test_mobilita_parte5.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA TEST 5 - Score: {msg.mobilita_test5_score}, Stato: {msg.mobilita_test5_stato}")
        
        elif formtype == "mobilita_complete":
            msg.mobilita_punteggio_totale = int(test_data.get('punteggio_totale', 0))
            current_page = "test_mobilita_completato.html"
            msg.test_name = "Mobilita"
            msg.page_name = current_page
            
            self.get_logger().info(f" MOBILITA COMPLETO - Totale: {msg.mobilita_punteggio_totale}/13")

            
        # ========== TEST MUST ==========
        elif formtype == "must_parte1":
            msg.must_parte1_score = int(test_data.get('score', 0))
            msg.must_parte1_bmi = float(test_data.get('bmi', 0.0))
            msg.must_parte1_peso = float(test_data.get('peso', 0.0))
            msg.must_parte1_altezza = float(test_data.get('altezza', 0.0))
            msg.must_parte1_stato = str(test_data.get('stato', ''))
            current_page = "test_MUST_parte1.html"
            msg.test_name = "MUST"
            msg.page_name = current_page
            
            self.get_logger().info(f" MUST PARTE 1 - BMI: {msg.must_parte1_bmi:.1f}, Score: {msg.must_parte1_score}, Stato: {msg.must_parte1_stato}")
        
        elif formtype == "must_parte2":
            msg.must_parte2_score = int(test_data.get('score', 0))
            msg.must_parte2_stato = str(test_data.get('stato', ''))
            current_page = "test_MUST_parte2.html"
            msg.test_name = "MUST"
            msg.page_name = current_page
            
            self.get_logger().info(f" MUST PARTE 2 - Score: {msg.must_parte2_score}, Stato: {msg.must_parte2_stato}")
        
        elif formtype == "must_parte3":
            msg.must_parte3_score = int(test_data.get('score', 0))
            msg.must_parte3_stato = str(test_data.get('stato', ''))
            current_page = "test_MUST_parte3.html"
            msg.test_name = "MUST"
            msg.page_name = current_page
            
            self.get_logger().info(f" MUST PARTE 3 - Score: {msg.must_parte3_score}, Stato: {msg.must_parte3_stato}")
        
        elif formtype == "must_complete":
            msg.must_punteggio_totale = int(test_data.get('punteggio_totale', 0))
            msg.must_stato_nutrizionale = str(test_data.get('stato_nutrizionale', ''))
            current_page = "test_MUST_completato.html"
            msg.test_name = "MUST"
            msg.page_name = current_page
            
            self.get_logger().info(f" MUST COMPLETO - Totale: {msg.must_punteggio_totale}/6, Stato: {msg.must_stato_nutrizionale}")
        
        else:
            msg.test_data = json.dumps(test_data)
            self.get_logger().warn(f"️ Formtype non riconosciuto: {formtype}")
        
        # Pubblica il messaggio
        self.test_data_pub.publish(msg)
        self.get_logger().info(
            f"TestData pubblicato - Type: {msg.test_type}, "
            f"Test: {msg.test_name}, Timestamp: {msg.timestamp}"
        )
        
        # ========== NAVIGAZIONE AUTOMATICA ==========
        if current_page and current_page in self.page_navigation:
            next_page = self.page_navigation[current_page]
            if next_page:
                self.get_logger().info(f"Pagina successiva: {next_page}")
            else:
                self.get_logger().info(f"Fine sequenza: {current_page}")


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

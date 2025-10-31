import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.parse
import os
import threading
import queue
import json
import socket
import subprocess
import re
import time
import logging
from socketserver import ThreadingMixIn

PORT = 8080
ROOT_DIR = "/home/roboticslab/Scrivania/Robot_Architecture/MHARA_Unipa/src/qi_unipa/html_pages"

# Queue con dimensione aumentata e timeout configurabile
request_queue_user = queue.Queue(maxsize=200)
test_queue = queue.Queue(maxsize=200)

# Setup logging migliorato
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def get_local_ip():
    """Ottiene l'indirizzo IP locale della macchina"""
    try:
        # Metodo più affidabile usando socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        pass
    
    try:
        # Metodo di fallback con ip route
        result = subprocess.run(['ip', 'route'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'default via' in line:
                    interface_match = re.search(r'dev (\w+)', line)
                    if interface_match:
                        interface = interface_match.group(1)
                        ip_result = subprocess.run(['ip', 'addr', 'show', interface], 
                                                 capture_output=True, text=True, timeout=5)
                        if ip_result.returncode == 0:
                            ip_match = re.search(r'inet (192\.168\.\d+\.\d+|10\.\d+\.\d+\.\d+)', 
                                               ip_result.stdout)
                            if ip_match:
                                return ip_match.group(1)
    except Exception as e:
        logger.error(f"Errore nel get_local_ip: {e}")
    
    return "127.0.0.1"

class ImprovedHandler(BaseHTTPRequestHandler):
    """Handler HTTP migliorato con gestione errori robusta"""
    
    def __init__(self, *args, **kwargs):
        # Inizializza il timeout per le connessioni
        super().__init__(*args, **kwargs)
    
    def setup(self):
        """Setup della connessione con timeout"""
        super().setup()
        # Imposta timeout per evitare connessioni appese
        if hasattr(self.request, 'settimeout'):
            self.request.settimeout(30)
    
    def do_GET(self):
        """Gestisce le richieste GET con logging dettagliato"""
        client_addr = self.client_address[0] if self.client_address else "unknown"
        start_time = time.time()
        
        try:
            parsed_path = urllib.parse.urlparse(self.path)
            path = parsed_path.path.strip("/")
            query = urllib.parse.parse_qs(parsed_path.query)

            logger.info(f"[{client_addr}] GET {path} - Query: {bool(query)}")

            # Mappatura dei path
            path_handlers = {
                "registrazione.html": self._handle_registrazione,
                "attivo.html": self._handle_simple_page,
                "disattivo.html": self._handle_simple_page,
                "vuoi_effettuare_il_test.html": self._handle_test_page,
                "minicog_orologio.html": self._handle_test_page,
                "minicog_parole.html": self._handle_test_page,
                "minicog_parole_verifica.html": self._handle_test_page,
                "si_no.html": self._handle_test_page,
                "autovalutazione_3_opzioni.html": self._handle_test_page,
                "autovalutazione_5_opzioni.html": self._handle_test_page,
                "dati_must.html": self._handle_test_page,
                "thinking.html": self._handle_simple_page
            }

            if path in path_handlers:
                file_path = os.path.join(ROOT_DIR, path)
                if os.path.exists(file_path):
                    # Processa la query PRIMA di servire il file
                    if query:
                        success = path_handlers[path](path, query)
                        if not success:
                            logger.warning(f"[{client_addr}] Errore nel processare query per {path}")
                    
                    # Servi il file
                    self._serve_file(file_path, client_addr)
                    
                    processing_time = time.time() - start_time
                    logger.info(f"[{client_addr}] Completato in {processing_time:.3f}s")
                else:
                    logger.error(f"[{client_addr}] File non trovato: {file_path}")
                    self.send_error(404, "File non trovato")
            else:
                logger.warning(f"[{client_addr}] Path non riconosciuto: {path}")
                self.send_error(404, "Risorsa non trovata")

        except ConnectionAbortedError:
            logger.warning(f"[{client_addr}] Connessione interrotta dal client")
        except Exception as e:
            logger.error(f"[{client_addr}] Errore in do_GET: {e}", exc_info=True)
            try:
                self.send_error(500, "Errore interno del server")
            except:
                pass  # Connessione già chiusa

    def _handle_registrazione(self, path, query):
        """Gestisce registrazione con retry e validazione"""
        try:
            logger.info(f"Processando registrazione: {query}")
            
            # Validazione base dei dati
            required_fields = ['nome', 'cognome']
            for field in required_fields:
                if field not in query or not query[field][0].strip():
                    logger.warning(f"Campo mancante o vuoto: {field}")
                    return False
            
            # Retry per inserimento in coda
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    request_queue_user.put(query, timeout=1.0)
                    logger.info(f"Query registrazione aggiunta alla coda (tentativo {attempt + 1})")
                    return True
                except queue.Full:
                    if attempt < max_retries - 1:
                        logger.warning(f"Coda piena, tentativo {attempt + 1}, retry in 0.1s")
                        time.sleep(0.1)
                    else:
                        logger.error("Coda registrazione piena dopo tutti i tentativi")
                        return False
            
        except Exception as e:
            logger.error(f"Errore in _handle_registrazione: {e}")
            return False

    def _handle_simple_page(self, path, query):
        """Gestisce pagine semplici"""
        return True

    def _handle_test_page(self, path, query):
        """Gestisce pagine di test con retry"""
        try:
            logger.info(f"Processando test da {path}: {query}")
            
            # Retry per inserimento in coda
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    test_queue.put(query, timeout=1.0)
                    logger.info(f"Query test da {path} aggiunta alla coda (tentativo {attempt + 1})")
                    return True
                except queue.Full:
                    if attempt < max_retries - 1:
                        logger.warning(f"Coda test piena, tentativo {attempt + 1}, retry in 0.1s")
                        time.sleep(0.1)
                    else:
                        logger.error("Coda test piena dopo tutti i tentativi")
                        return False
                        
        except Exception as e:
            logger.error(f"Errore in _handle_test_page: {e}")
            return False

    def _serve_file(self, file_path, client_addr="unknown"):
        """Serve file con gestione errori migliorata"""
        try:
            if not os.path.exists(file_path):
                self.send_error(404, "File non trovato")
                return

            # Determina content-type
            content_type = "text/html; charset=utf-8"
            if file_path.endswith('.css'):
                content_type = "text/css"
            elif file_path.endswith('.js'):
                content_type = "application/javascript"

            # Leggi il file
            with open(file_path, "rb") as f:
                content = f.read()

            # Headers migliorati
            self.send_response(200)
            self.send_header("Content-type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Connection", "close")  # Forza chiusura connessione
            self.end_headers()

            # Invia contenuto in chunks per file grandi
            chunk_size = 8192
            bytes_sent = 0
            while bytes_sent < len(content):
                chunk = content[bytes_sent:bytes_sent + chunk_size]
                try:
                    self.wfile.write(chunk)
                    self.wfile.flush()
                    bytes_sent += len(chunk)
                except (ConnectionAbortedError, BrokenPipeError):
                    logger.warning(f"[{client_addr}] Connessione interrotta durante invio")
                    break

            if bytes_sent == len(content):
                logger.info(f"[{client_addr}] File servito: {os.path.basename(file_path)} ({len(content)} bytes)")

        except IOError as e:
            logger.error(f"[{client_addr}] Errore I/O: {e}")
            try:
                self.send_error(500, "Errore nel leggere il file")
            except:
                pass
        except Exception as e:
            logger.error(f"[{client_addr}] Errore generico: {e}")
            try:
                self.send_error(500, "Errore interno")
            except:
                pass

    def log_message(self, format, *args):
        """Log personalizzato meno verboso"""
        pass  # Disabilita log automatici, usiamo il nostro

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Server HTTP multi-thread con configurazioni migliorate"""
    
    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)
        self.daemon_threads = True
        self.allow_reuse_address = True
        # Configurazioni per migliorare la gestione delle connessioni
        self.request_queue_size = 10  # Aumenta la coda di richieste in attesa
        
    def server_bind(self):
        """Binding del server con opzioni socket migliorate"""
        super().server_bind()
        # Configurazioni socket per migliorare le performance
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            try:
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass  # SO_REUSEPORT non supportato su tutti i sistemi

def run_server():
    """Avvia il server HTTP con gestione errori migliorata"""
    max_retries = 5
    retry_delay = 1
    
    for attempt in range(max_retries):
        try:
            server = ThreadedHTTPServer(('', PORT), ImprovedHandler)
            logger.info(f"Server HTTP avviato sulla porta {PORT} (tentativo {attempt + 1})")
            server.serve_forever()
            break
            
        except OSError as e:
            if e.errno == 98:  # Address already in use
                logger.warning(f"Porta {PORT} occupata, tentativo {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    # Tenta di killare processi sulla porta
                    try:
                        subprocess.run(['fuser', '-k', f'{PORT}/tcp'], 
                                     capture_output=True, timeout=5)
                        time.sleep(retry_delay)
                    except:
                        time.sleep(retry_delay)
                    retry_delay *= 2
                else:
                    logger.error(f"Impossibile avviare server dopo {max_retries} tentativi")
                    raise
            else:
                logger.error(f"Errore OSError nel server HTTP: {e}")
                raise
                
        except Exception as e:
            logger.error(f"Errore imprevisto nel server: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                retry_delay *= 2
            else:
                raise

class QiUnipa2_server(Node):
    def __init__(self):
        super().__init__('qi_unipa_2_server')
        self.get_logger().info("Nodo Server ROS avviato.")

        # Statistiche per monitoraggio
        self.stats = {
            'requests_processed': 0,
            'tests_processed': 0,
            'queue_full_errors': 0,
            'processing_errors': 0
        }

        # Avvia il server HTTP in un thread separato
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

        # Publishers
        self._create_publishers()

        # Timer con frequenze ottimizzate
        self.create_timer(0.01, self.check_requests_user)  
        self.create_timer(0.01, self.check_test_stato_cognitivo_user)
        self.create_timer(1.0, self.publish_server_url)  
      

    def _create_publishers(self):
        """Crea tutti i publisher ROS"""
        self.user_pub = self.create_publisher(String, '/user_update', 10)
        self.orologio_pub = self.create_publisher(String, '/orologio_input', 10)
        self.fine_visualizzazione_3_parole = self.create_publisher(Bool, '/fine_visualizzazione_3_parole', 10)
        self.parole_selezionate = self.create_publisher(String, '/parole_selezionate', 10)
        self.scelta_pub = self.create_publisher(String, '/risposta_si_no', 10)
        self.get_data = self.create_publisher(String, '/get_data', 10)
        
        self.risultato_test1 = self.create_publisher(String, '/risultato_test1', 10)
        self.risultato_test2 = self.create_publisher(String, '/risultato_test2', 10)
        self.risultato_test3 = self.create_publisher(String, '/risultato_test3', 10)
        self.risultato_test4 = self.create_publisher(String, '/risultato_test4', 10)
        self.risultato_test5 = self.create_publisher(String, '/risultato_test5', 10)
        
        self.must_test1 = self.create_publisher(String, '/must_test1', 10)
        self.must_test2 = self.create_publisher(String, '/must_test2', 10)
        self.must_test3 = self.create_publisher(String, '/must_test3', 10)

        self.server_url_pub = self.create_publisher(String, '/ip', 10)

    def log_stats(self):
        """Log delle statistiche periodiche"""
        self.get_logger().info(
            f"Stats - Processed: {self.stats['requests_processed']} requests, "
            f"{self.stats['tests_processed']} tests. "
            f"Errors: {self.stats['queue_full_errors']} queue_full, "
            f"{self.stats['processing_errors']} processing. "
            f"Queue sizes: user={request_queue_user.qsize()}, test={test_queue.qsize()}"
        )

    def publish_server_url(self):
        """Pubblica l'indirizzo corrente del server (meno frequente)"""
        try:
            local_ip = get_local_ip()
            server_url = f"http://{local_ip}:{PORT}"
            
            msg = String()
            msg.data = server_url
            self.server_url_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore nel pubblicare URL server: {e}")

    def check_requests_user(self):
        """Controlla richieste utente con batch processing"""
        processed_count = 0
        max_process_per_cycle = 10  # Aumentato per migliore throughput
        
        while not request_queue_user.empty() and processed_count < max_process_per_cycle:
            try:
                value = request_queue_user.get(timeout=0.1)
                self.get_logger().info(f"Processando richiesta utente: {value}")
                
                nome = self._safe_extract(value, 'nome')
                cognome = self._safe_extract(value, 'cognome')
                peso = self._safe_extract(value, 'peso')
                altezza = self._safe_extract(value, 'altezza')
                
                # Gestione patologie e intolleranze migliorata
                patologie = ' ' if (value.get('patologie', [''])[0] == 'Nessuna' or not value.get('patologie')) else value['patologie']
                intolleranze = ' ' if (value.get('intolleranze', [''])[0] == 'Nessuna' or not value.get('intolleranze')) else value['intolleranze']
                
                if nome or cognome or peso:  # Validazione base
                    msg = String()
                    msg.data = f"nome: {nome}, cognome: {cognome}, patologia: {patologie}, intolleranza: {intolleranze}, peso: {peso}, altezza: {altezza}"
                    self.get_logger().info(f"Pubblicando dati utente: {msg.data}")
                    self.user_pub.publish(msg)
                    
                    # Pubblica anche i dati JSON
                    messaggio_json = {"nome": nome, "cognome": cognome}
                    msg_json = String()
                    msg_json.data = json.dumps(messaggio_json)
                    self.get_data.publish(msg_json)
                    
                    self.stats['requests_processed'] += 1
                
                processed_count += 1

            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Errore nel processare richiesta utente: {e}")
                self.stats['processing_errors'] += 1
                processed_count += 1

    def check_test_stato_cognitivo_user(self):
        """Controlla test con batch processing migliorato"""
        processed_count = 0
        max_process_per_cycle = 10
        
        while not test_queue.empty() and processed_count < max_process_per_cycle:
            try:
                value = test_queue.get(timeout=0.1)
                self.get_logger().info(f"Processando test: {value}")
                
                success = self._process_test_value(value)
                if success:
                    self.stats['tests_processed'] += 1
                else:
                    self.stats['processing_errors'] += 1
                    
                processed_count += 1
                
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Errore nel processare test: {e}")
                self.stats['processing_errors'] += 1
                processed_count += 1

    def _safe_extract(self, data_dict, key, default=''):
        """Estrae sicuramente un valore da un dizionario di liste"""
        try:
            value_list = data_dict.get(key, [default])
            return value_list[0] if value_list and value_list[0] else default
        except (IndexError, TypeError):
            return default

    def _process_test_value(self, value):
        """Processa i valori dei test con gestione errori migliorata"""
        try:
            # Mapping migliorato per evitare ripetizioni
            simple_tests = {
                'scelta': (self.scelta_pub, "SCELTA UTENTE"),
                'test1': (self.risultato_test1, "TEST1"),
                'test2': (self.risultato_test2, "TEST2"),
                'test3': (self.risultato_test3, "TEST3"),
                'test4': (self.risultato_test4, "TEST4"),
                'test5': (self.risultato_test5, "TEST5"),
                'must2': (self.must_test2, "MUST2"),
                'must3': (self.must_test3, "MUST3")
            }
            
            # Gestisci test semplici
            for key, (publisher, log_name) in simple_tests.items():
                if key in value:
                    scelta = self._safe_extract(value, key)
                    self.get_logger().info(f"[{log_name}] Utente ha selezionato: {scelta}")
                    
                    msg = String()
                    msg.data = scelta
                    publisher.publish(msg)
                    return True
            
            # Gestisci casi speciali
            if 'ora1' in value:
                return self._handle_orologio(value)
            elif 'esito' in value and self._safe_extract(value, 'esito') == 'fine_visualizzazione_3_parole':
                return self._handle_fine_visualizzazione(value)
            elif all(key in value for key in ['s1', 's2', 's3']):
                return self._handle_parole_selezionate(value)
            elif 'peso' in value and 'altezza' in value:
                return self._handle_must_test1(value)
            
            self.get_logger().warning(f"Tipo di test non riconosciuto: {list(value.keys())}")
            return False
                            
        except Exception as e:
            self.get_logger().error(f"Errore nel processare valore test: {e}")
            return False

    def _handle_orologio(self, value):
        """Gestisce i dati dell'orologio"""
        try:
            orologio = {k: value[k][0] for k in sorted(value) if k.startswith('ora')}
            msg = String()
            msg.data = json.dumps(orologio)
            self.get_logger().info(f"[MINICOG OROLOGIO] {msg.data}")
            self.orologio_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Errore nella gestione orologio: {e}")
            return False

    def _handle_fine_visualizzazione(self, value):
        """Gestisce la fine della visualizzazione delle 3 parole"""
        try:
            w1 = self._safe_extract(value, 'w1')
            w2 = self._safe_extract(value, 'w2')
            w3 = self._safe_extract(value, 'w3')
            self.get_logger().info(f"Parole mostrate: {w1}, {w2}, {w3}")
            self.fine_visualizzazione_3_parole.publish(Bool(data=True))
            return True
        except Exception as e:
            self.get_logger().error(f"Errore nella gestione fine visualizzazione: {e}")
            return False

    def _handle_parole_selezionate(self, value):
        """Gestisce le parole selezionate"""
        try:
            selezionate = [
                self._safe_extract(value, 's1'),
                self._safe_extract(value, 's2'),
                self._safe_extract(value, 's3')
            ]
            self.get_logger().info(f"[MINICOG - VERIFICA PAROLE] Selezionate: {selezionate}")
            msg = String()
            msg.data = json.dumps({"selezionate": selezionate})
            self.parole_selezionate.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Errore nella gestione parole selezionate: {e}")
            return False

    def _handle_must_test1(self, value):
        """Gestisce il test MUST 1 (peso e altezza)"""
        try:
            peso = self._safe_extract(value, 'peso')
            altezza = self._safe_extract(value, 'altezza')
            
            self.get_logger().info(f"[MUST TEST1] Peso: {peso} kg, Altezza: {altezza} cm")
            
            msg = String()
            msg.data = f"{peso},{altezza}"
            self.must_test1.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Errore nella gestione MUST test1: {e}")
            return False

def main(args=None):
    try:
        rclpy.init(args=args)
        node = QiUnipa2_server()
        
        logger.info("Server ROS2 avviato correttamente")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        logger.info("Arresto del server richiesto dall'utente")
    except Exception as e:
        logger.error(f"Errore nel main: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        logger.info("Server arrestato")

if __name__ == '__main__':
    main()
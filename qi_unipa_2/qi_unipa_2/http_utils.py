"""
HTTP server utilities per gestione form HTML e comunicazione con tablet.
Separato dal nodo ROS2 per migliore testabilità e manutenibilità.
"""
import os
import time
import logging
import queue
import socket  # ← Aggiungi questo
import urllib.parse
import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from qi_unipa_2.utils import Utils  # ← Import funzione separata


# ====================
# CONFIGURAZIONE
# ====================

DEFAULT_PORT = 8080

# Setup logging
logger = logging.getLogger(__name__)


# ====================
# QUEUE GLOBALI
# ====================
# Note: Queste queue sono condivise tra HTTP server thread e nodo ROS2

request_queue_user = queue.Queue(maxsize=200)
test_queue = queue.Queue(maxsize=200)

# ====================
# HTTP HANDLER
# ====================

class FormHTTPHandler(BaseHTTPRequestHandler):
    """
    Handler HTTP per gestione pagine HTML e form.
    Processa dati form e li inserisce in queue per nodo ROS2.
    """
    
    # Path HTML pages (impostato dal server)
    html_dir = None
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def setup(self):
        """Setup connessione con timeout"""
        super().setup()
        if hasattr(self.request, 'settimeout'):
            self.request.settimeout(30)
    
    def do_GET(self):
        """Gestisce richieste GET con logging dettagliato"""
        client_addr = self.client_address[0] if self.client_address else "unknown"
        start_time = time.time()
        
        try:
            parsed_path = urllib.parse.urlparse(self.path)
            path = parsed_path.path.strip("/")
            query = urllib.parse.parse_qs(parsed_path.query)
            
            logger.info(f"[{client_addr}] GET {path} - Query: {bool(query)}")
            
            # Mappatura handler per pagine specifiche
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
                file_path = os.path.join(self.html_dir, path)
                if os.path.exists(file_path):
                    # Processa query PRIMA di servire file
                    if query:
                        success = path_handlers[path](path, query)
                        if not success:
                            logger.warning(f"[{client_addr}] Errore processamento query {path}")
                    
                    # Servi file HTML
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
                pass
    
    def _handle_registrazione(self, path, query):
        """Gestisce form registrazione utente"""
        try:
            logger.info(f"Processando registrazione: {query}")
            
            # Validazione campi obbligatori
            required_fields = ['nome', 'cognome']
            for field in required_fields:
                if field not in query or not query[field][0].strip():
                    logger.warning(f"Campo mancante: {field}")
                    return False
            
            # Inserisci in queue con retry
            return self._queue_put_retry(request_queue_user, query, "registrazione")
        
        except Exception as e:
            logger.error(f"Errore in _handle_registrazione: {e}")
            return False
    

    def _handle_simple_page(self, path, query):
        """Gestisce pagine semplici senza form"""
        return True
    

    def _handle_test_page(self, path, query):
        """Gestisce form test cognitivi"""
        try:
            logger.info(f"Processando test da {path}: {query}")
            return self._queue_put_retry(test_queue, query, "test")
        except Exception as e:
            logger.error(f"Errore in _handle_test_page: {e}")
            return False
    

    def _queue_put_retry(self, target_queue, data, queue_name, max_retries=3):
        """Helper per inserimento in queue con retry"""
        for attempt in range(max_retries):
            try:
                target_queue.put(data, timeout=1.0)
                logger.info(f"Query {queue_name} aggiunta (tentativo {attempt + 1})")
                return True
            except queue.Full:
                if attempt < max_retries - 1:
                    logger.warning(f"Coda {queue_name} piena, retry {attempt + 1}")
                    time.sleep(0.1)
                else:
                    logger.error(f"Coda {queue_name} piena dopo {max_retries} tentativi")
                    return False
        return False
    

    def _serve_file(self, file_path, client_addr="unknown"):
        """Serve file HTML/CSS/JS"""
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
            
            # Leggi file
            with open(file_path, "rb") as f:
                content = f.read()
            
            # Invia response
            self.send_response(200)
            self.send_header("Content-type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            
            # Invia contenuto in chunks
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
                logger.info(f"[{client_addr}] File servito: {os.path.basename(file_path)}")
        
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
        """Disabilita log automatici HTTP (usiamo custom logger)"""
        pass



# ====================
# HTTP SERVER
# ====================

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Server HTTP multi-thread con configurazioni migliorate"""
    
    def __init__(self, server_address, RequestHandlerClass, html_dir):
        # Imposta html_dir prima di chiamare super().__init__
        RequestHandlerClass.html_dir = html_dir
        
        super().__init__(server_address, RequestHandlerClass)
        self.daemon_threads = True
        self.allow_reuse_address = True
        self.request_queue_size = 10
    
    def server_bind(self):
        """Binding con opzioni socket migliorate"""
        super().server_bind()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            try:
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass


# ====================
# SERVER MANAGER
# ====================

class HTTPServerManager:
    """
    Manager per lifecycle del server HTTP.
    Gestisce creazione, avvio con retry, e shutdown del server.
    """
    
    def __init__(self, port=DEFAULT_PORT, html_dir=None):
        """
        Inizializza manager.
        
        Args:
            port: Porta su cui avviare server
            html_dir: Path cartella HTML (auto-detect se None)
        """
        self.port = port
        self.html_dir = html_dir if html_dir else Utils.get_html_pages_dir()
        self.server = None
        
        logger.info(f"HTTPServerManager inizializzato - Porta: {self.port}, HTML: {self.html_dir}")
    
    def create_server(self):
        """
        Crea istanza server HTTP.
        
        Returns:
            ThreadedHTTPServer configurato
        """
        logger.info(f"Creazione server HTTP sulla porta {self.port}")
        self.server = ThreadedHTTPServer(
            ('', self.port),
            FormHTTPHandler,
            self.html_dir
        )
        return self.server
    
    def start(self, max_retries=5):
        """
        Avvia server con retry automatici.
        Questa funzione BLOCCA fino a shutdown del server.
        
        Args:
            max_retries: Numero massimo tentativi
        
        Raises:
            OSError: Se impossibile avviare server dopo retry
        """
        retry_delay = 1
        
        for attempt in range(max_retries):
            try:
                if self.server is None:
                    self.create_server()
                
                logger.info(f"Avvio server HTTP (tentativo {attempt + 1}/{max_retries})")
                self.server.serve_forever()
                break
            
            except OSError as e:
                if e.errno == 98:  # Address already in use
                    logger.warning(f"Porta {self.port} occupata, tentativo {attempt + 1}/{max_retries}")
                    if attempt < max_retries - 1:
                        self._kill_port_process()
                        time.sleep(retry_delay)
                        retry_delay *= 2
                        self.server = None  # Reset per creare nuovo server
                    else:
                        logger.error(f"Impossibile avviare server dopo {max_retries} tentativi")
                        raise
                else:
                    logger.error(f"Errore OSError: {e}")
                    raise
            
            except Exception as e:
                logger.error(f"Errore imprevisto: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    retry_delay *= 2
                    self.server = None
                else:
                    raise
    
    def _kill_port_process(self):
        """Tenta di killare processi sulla porta"""
        try:
            subprocess.run(
                ['fuser', '-k', f'{self.port}/tcp'],
                capture_output=True,
                timeout=5
            )
            logger.info(f"Processi sulla porta {self.port} killati")
        except Exception as e:
            logger.warning(f"Impossibile killare processi su porta {self.port}: {e}")
    
    def shutdown(self):
        """Shutdown graceful del server"""
        if self.server:
            logger.info("Shutdown server HTTP")
            self.server.shutdown()
            self.server = None
    
    def __enter__(self):
        """Context manager support"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup"""
        self.shutdown()
"""
HTTP server utilities per gestione form HTML e comunicazione con tablet.
Separato dal nodo ROS2 per migliore testabilità e manutenibilità.
"""
import os
import time
import logging
import queue
import socket
import urllib.parse
import subprocess
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from qi_unipa_2.utils import Utils



# ====================
# CONFIGURAZIONE
# ====================


DEFAULT_PORT = 8080


# Setup logging
logger = logging.getLogger(__name__)



# ====================
# QUEUE GLOBALI
# ====================


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
            
            # ✅ Path handlers - TUTTE LE PAGINE
            path_handlers = {
                # Registrazione
                "registrazione.html": self._handle_simple_page,
                "registrazione_completata.html": self._handle_simple_page,
                
                # MiniCog
                "test_minicog_parte1.html": self._handle_simple_page,
                "test_minicog_parte2.html": self._handle_simple_page,
                "test_minicog_parte3.html": self._handle_simple_page,
                "test_minicog_parte4.html": self._handle_simple_page,
                "test_minicog_completato.html": self._handle_final_page,
                
                # Mobilità
                "test_mobilita_parte1.html": self._handle_simple_page,
                "test_mobilita_parte2.html": self._handle_simple_page,
                "test_mobilita_parte3.html": self._handle_simple_page,
                "test_mobilita_parte4.html": self._handle_simple_page,
                "test_mobilita_parte5.html": self._handle_simple_page,
                "test_mobilita_completato.html": self._handle_final_page,
                
                # MUST
                "test_MUST_parte1.html": self._handle_simple_page,
                "test_MUST_parte2.html": self._handle_simple_page,
                "test_MUST_parte3.html": self._handle_simple_page,
                "test_MUST_completato.html": self._handle_final_page,
                
                # Pagine di stato
                "attivo.html": self._handle_simple_page,
                "disattivo.html": self._handle_simple_page,
                "thinking.html": self._handle_simple_page,
            }
            
            if path in path_handlers:
                file_path = os.path.join(self.html_dir, path)
                if os.path.exists(file_path):
                    handler = path_handlers[path]
                    
                    # ✅ Chiama handler SOLO se ha query E non è final_page
                    if query and handler != self._handle_final_page:
                        success = handler(path, query)
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



    def _handle_simple_page(self, path, query):
        """Gestisce pagine semplici senza form"""
        if query:
            logger.warning(f"⚠️ Query su pagina semplice {path} (ignorata): {query}")
        return True


    def _handle_final_page(self, path, query):
        """Gestisce pagine finali di test (binari morti - no redirect)"""
        if query:
            logger.warning(f"⚠️ Query su pagina finale {path} (ignorata - binario morto): {query}")
        logger.info(f"🏁 Pagina finale caricata: {path}")
        return True



    def do_POST(self):
        """Gestisce richieste POST per /submit_form con nuova struttura FormData"""
        client_addr = self.client_address[0] if self.client_address else "unknown"
        
        if self.path == '/submit_form':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                post_data = self.rfile.read(content_length)
                form_data = json.loads(post_data.decode('utf-8'))
                
                logger.info(f"[{client_addr}] 📥 POST /submit_form")
                logger.info(f"[{client_addr}] 📦 form_data ricevuti: {form_data}")  # ← DEBUG 1
                
                formtype = form_data.get('formtype', 'unknown')
                logger.info(f"[{client_addr}] 🔍 formtype estratto: '{formtype}'")  # ← DEBUG 2
                
                # ========== ROUTING DATI IN QUEUE ==========
                
                success = False
                
                # REGISTRAZIONE → user queue
                if formtype == 'registrazione':
                    success = self._queue_put_retry(request_queue_user, form_data, "registrazione")
                    logger.info(f"[{client_addr}] ✅ REGISTRAZIONE → queue, success={success}")  # ← DEBUG 3
                
                # MINICOG (tutti) → test queue
                elif formtype.startswith('minicog_'):
                    success = self._queue_put_retry(test_queue, form_data, f"minicog_{formtype}")
                    logger.info(f"[{client_addr}] ✅ MINICOG → queue, success={success}")  # ← DEBUG 3
                
                # MOBILITÀ (tutti) → test queue
                elif formtype.startswith('mobilita_'):
                    success = self._queue_put_retry(test_queue, form_data, f"mobilita_{formtype}")
                    logger.info(f"[{client_addr}] ✅ MOBILITA → queue, success={success}")  # ← DEBUG 3
                
                # MUST (tutti) → test queue
                elif formtype.startswith('must_'):
                    success = self._queue_put_retry(test_queue, form_data, f"must_{formtype}")
                    logger.info(f"[{client_addr}] ✅ MUST → queue, success={success}")  # ← DEBUG 3
                
                # SCONOSCIUTO → user queue
                else:
                    logger.warning(f"[{client_addr}] ⚠️ Formtype SCONOSCIUTO: '{formtype}'")
                    success = self._queue_put_retry(request_queue_user, form_data, "sconosciuto")
                    logger.info(f"[{client_addr}] ✅ SCONOSCIUTO → queue, success={success}")  # ← DEBUG 3
                
                if success:
                    # ✅ CALCOLA REDIRECT URL
                    redirect_url = self._get_redirect_url(formtype)
                    logger.info(f"[{client_addr}] 🔗 redirect_url calcolato: '{redirect_url}'")  # ← DEBUG 4
                    
                    # Prepara response
                    response = {
                        "status": "success",
                        "message": "Data received",
                        "redirect_url": redirect_url  
                    }
                    logger.info(f"[{client_addr}] 📤 Response pronto: {response}")  # ← DEBUG 5
                    
                    # Invia response
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    
                    response_json = json.dumps(response)
                    self.wfile.write(response_json.encode())
                    logger.info(f"[{client_addr}] ✅ Response inviato ({len(response_json)} bytes)")  # ← DEBUG 6
                    
                    if redirect_url:
                        logger.info(f"[{client_addr}] ✅ Form OK - Redirect a: {redirect_url}")
                    else:
                        logger.info(f"[{client_addr}] ✅ Form OK - Binario morto (no redirect)")
                else:
                    logger.error(f"[{client_addr}] ❌ success=False! Nessun put_retry ha funzionato!")  # ← DEBUG 7
                    raise Exception("Queue full - nessun retry ha avuto successo")
                    
            except json.JSONDecodeError as e:
                logger.error(f"[{client_addr}] ❌ Errore JSON decode: {e}")
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                error_response = {"status": "error", "message": "Invalid JSON"}
                self.wfile.write(json.dumps(error_response).encode())
                logger.info(f"[{client_addr}] 📤 Errore response inviato (400)")
            
            except Exception as e:
                logger.error(f"[{client_addr}] ❌ Errore POST handler: {e}", exc_info=True)
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                error_response = {"status": "error", "message": str(e)}
                error_json = json.dumps(error_response)
                self.wfile.write(error_json.encode())
                logger.info(f"[{client_addr}] 📤 Errore response inviato (500): {str(e)}")
        else:
            logger.warning(f"[{client_addr}] POST path non riconosciuto: {self.path}")
            self.send_error(404, "Endpoint non trovato")



    def _get_redirect_url(self, formtype):
        """
        Determina URL di redirect in base al formtype.
        Ritorna None per pagine finali (binari morti).
        """
        # Mappatura formtype → pagina successiva
        redirect_map = {
            # Registrazione
            "registrazione": "registrazione_completata.html",
            
            # MiniCog
            "minicog_parte1": "test_minicog_parte2.html",
            "minicog_parte2": "test_minicog_parte3.html",
            "minicog_parte3": "test_minicog_parte4.html",
            "minicog_parte4": "test_minicog_completato.html",
            
            # Mobilità
            "mobilita_test1": "test_mobilita_parte2.html",
            "mobilita_test2": "test_mobilita_parte3.html",
            "mobilita_test3": "test_mobilita_parte4.html",
            "mobilita_test4": "test_mobilita_parte5.html",
            "mobilita_test5": "test_mobilita_completato.html",
            
            # MUST
            "must_parte1": "test_MUST_parte2.html",
            "must_parte2": "test_MUST_parte3.html",
            "must_parte3": "test_MUST_completato.html",
            "must_complete": "test_MUST_completato.html",
            
            # Pagine finali (binari morti) - no redirect
            "minicog_complete": None,
            "mobilita_complete": None,
            "must_complete": None,
        }
        
        next_page = redirect_map.get(formtype)
        
        if next_page is not None:
            return f"http://localhost:8080/{next_page}"  # ✅ URL completo
        elif next_page is None and formtype in redirect_map:
            # Consapevolmente None (pagina finale)
            logger.info(f"🏁 Binario morto per formtype: {formtype}")
            return None
        else:
            logger.warning(f"⚠️ Nessun mapping per formtype: {formtype}")
            return None


    
    def do_OPTIONS(self):
        """Gestisce preflight CORS"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    
    def _queue_put_retry(self, target_queue, data, queue_name, max_retries=3):
        """Helper per inserimento in queue con retry"""
        for attempt in range(max_retries):
            try:
                target_queue.put(data, timeout=1.0)
                logger.info(f"📤 Dati {queue_name} aggiunti alla queue (tentativo {attempt + 1})")
                return True
            except queue.Full:
                if attempt < max_retries - 1:
                    logger.warning(f"⚠️ Queue {queue_name} piena, retry {attempt + 1}")
                    time.sleep(0.1)
                else:
                    logger.error(f"❌ Queue {queue_name} piena dopo {max_retries} tentativi")
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
                        self.server = None
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

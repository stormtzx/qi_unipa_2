Altro aggiornamento 
DATA: 10 Novembre 2025


 - Soppresso il nodo Tablet e unificate le sue funzionalità con il nodo server che adesso sfrutta l'action di browsing per 
  le pagine html e per i form. Inoltre sono state spostate le funzioni di supporto presenti ad inizio del file in un file
   separato chiamato http_utils e parte in utils. 


================================================================================
AGGIORNAMENTO: NAMESPACE UNIFICATO PER API ROS2
================================================================================

DATA: 10 Novembre 2025
VERSIONE: v2.0.0
AUTORE: @stormtzx

================================================================================
SOMMARIO
================================================================================

Implementato uno schema di namespace unificato per tutte le interfacce ROS2 
(actions, services, topics) seguendo il pattern:

    /pepper/[tipo]/[nome]

Questo migliora l'organizzazione del codice, facilita l'integrazione con 
package esterni (es. pepper_ai) e supporta scenari multi-robot.

STANDARDIZZAZIONE NAMESPACE
---------------------------

Tutte le interfacce di comunicazione ROS2 seguono ora uno schema consistente:

    /pepper/actions/<nome_action>     - Operazioni a lunga durata
    /pepper/services/<nome_service>   - Operazioni request-response
    /pepper/topics/<nome_topic>       - Flussi continui di dati

VANTAGGI:
- Struttura API chiara per integrazione esterna
- Supporto multi-robot (es. /pepper, /nao, /tiago)
- Debug facilitato con: ros2 topic/action/service list | grep pepper
- Migliore documentazione e discoverabilità



================================================================================
API COMPLETA
================================================================================

-----------------------------------
ACTIONS (/pepper/actions/)
-----------------------------------

Operazioni a lunga durata con feedback real-time e supporto cancellazione.

NOME: /pepper/actions/walking
INTERFACCIA: Walking.action
DESCRIZIONE: Movimento base (x, y, theta)
ESEMPIO: ros2 action send_goal /pepper/actions/walking qi_unipa_2_interfaces/action/Walking "{x: 1.0, y: 0.5, theta: 0.0}"

NOME: /pepper/actions/talking
INTERFACCIA: Talking.action
DESCRIZIONE: Text-to-speech con supporto agent/percept
ESEMPIO: ros2 action send_goal /pepper/actions/talking qi_unipa_2_interfaces/action/Talking "{message: 'Ciao da Pepper!'}"

NOME: /pepper/actions/navigating
INTERFACCIA: Navigating.action
DESCRIZIONE: Navigazione autonoma con obstacle avoidance
ESEMPIO: ros2 action send_goal /pepper/actions/navigating qi_unipa_2_interfaces/action/Navigating "{target_x: 2.0, target_y: 1.5}"

NOME: /pepper/actions/browsing
INTERFACCIA: Browsing.action
DESCRIZIONE: Caricamento pagine HTML su tablet/browser
ESEMPIO: ros2 action send_goal /pepper/actions/browsing qi_unipa_2_interfaces/action/Browsing "{html_page: 'index.html', use_tablet: true}"


-----------------------------------
SERVICES (/pepper/services/)
-----------------------------------

Operazioni sincrone request-response.

MOVIMENTO E POSTURA:
- /pepper/services/set_posture (SetPosture.srv)
  Imposta postura robot (Stand, Sit, Crouch, etc.)
  
- /pepper/services/set_joint_angles (SetJointAngles.srv)
  Imposta angoli articolazioni specifiche
  
- /pepper/services/set_hand (SetHand.srv)
  Apri/chiudi mani
  
- /pepper/services/get_position (GetPosition.srv)
  Ottieni posizione robot (x, y, theta)

VISIONE:
- /pepper/services/get_image (GetImage.srv)
  Cattura immagine da camere
  Parametri: camera_index, resolution
  
- /pepper/services/get_coordinates (GetCoordinates.srv)
  Ottieni coordinate 3D da depth camera

AUDIO E TRACKING:
- /pepper/services/move_to_sound (MoveToSound.srv)
  Naviga verso sorgente audio rilevata
  
- /pepper/services/get_tracked_obj_coordinates (GetTrackedObjCoordinates.srv)
  Ottieni coordinate oggetto tracciato
  
- /pepper/services/move_to_tracked_obj (MoveToTrackedObj.srv)
  Naviga verso oggetto tracciato

ESEMPIO CHIAMATA SERVICE:
ros2 service call /pepper/services/get_image qi_unipa_2_interfaces/srv/GetImage "{camera_index: 0, resolution: 1}"


-----------------------------------
TOPICS (/pepper/topics/)
-----------------------------------

Flussi continui di dati per sensori e informazioni di stato.

DATI SENSORI:

NOME: /pepper/topics/bumper
TIPO: Bumper.msg
DESCRIZIONE: Sensori pressione (sinistro, destro, posteriore)
FREQUENZA: Event-based

NOME: /pepper/topics/sonar
TIPO: Sonar.msg
DESCRIZIONE: Sensori ultrasonici (anteriore, posteriore)
FREQUENZA: 10 Hz

NOME: /pepper/topics/head_touch
TIPO: HeadTouch.msg
DESCRIZIONE: Sensori tattili testa
FREQUENZA: Event-based

NOME: /pepper/topics/hand_touch
TIPO: HandTouch.msg
DESCRIZIONE: Sensori tattili mani
FREQUENZA: Event-based

NOME: /pepper/topics/battery
TIPO: Battery.msg
DESCRIZIONE: Stato batteria
FREQUENZA: Event-based

NOME: /pepper/topics/imu
TIPO: IMU.msg
DESCRIZIONE: Dati sensore inerziale
FREQUENZA: 10 Hz

NOME: /pepper/topics/infrared
TIPO: Infrared.msg
DESCRIZIONE: Rilevamento ostacoli IR
FREQUENZA: 10 Hz

NOME: /pepper/topics/emotion
TIPO: Emotion.msg
DESCRIZIONE: Emozione rilevata (se disponibile)
FREQUENZA: Variabile

STREAMING AUDIO/VIDEO:

NOME: /pepper/topics/transcription
TIPO: String
DESCRIZIONE: Output speech-to-text

NOME: /pepper/topics/sound_location
TIPO: PointStamped
DESCRIZIONE: Posizione 3D sorgente audio

NOME: /pepper/topics/video_feed
TIPO: Image
DESCRIZIONE: Stream video live da camera

DATI TRACKING:

NOME: /pepper/topics/tracked_coordinates
TIPO: PointStamped
DESCRIZIONE: Coordinate 3D continue oggetto tracciato
FREQUENZA: 10 Hz

NOME: /pepper/topics/tracker
TIPO: Tracker.msg
DESCRIZIONE: Comandi tracking (start/stop)

ESEMPIO MONITOR TOPICS:
ros2 topic echo /pepper/topics/bumper
ros2 topic echo /pepper/topics/sonar
ros2 topic echo /pepper/topics/tracked_coordinates


================================================================================
GUIDA MIGRAZIONE
================================================================================

Per utilizzatori del package qi_unipa_2 da package esterni (es. pepper_ai), 
aggiornare il codice client come segue:

-----------------------------------
ACTIONS
-----------------------------------

PRIMA (deprecato):
    self.walking_client = ActionClient(self, Walking, 'walking')

DOPO:
    self.walking_client = ActionClient(self, Walking, '/pepper/actions/walking')


-----------------------------------
SERVICES
-----------------------------------

PRIMA (deprecato):
    self.get_image_client = self.create_client(GetImage, 'get_image')

DOPO:
    self.get_image_client = self.create_client(GetImage, '/pepper/services/get_image')


-----------------------------------
TOPICS
-----------------------------------

PRIMA (deprecato):
    self.create_subscription(Bumper, '/bumper', callback, 10)

DOPO:
    self.create_subscription(Bumper, '/pepper/topics/bumper', callback, 10)


================================================================================
NODI AGGIORNATI
================================================================================

Tutti i nodi sono stati aggiornati per usare il nuovo namespace:

- qi_unipa_2_movement: Walking, Navigating actions + servizi movimento
- qi_unipa_2_speech: Talking action
- qi_unipa_2_tablet: Browsing action
- qi_unipa_2_sensor: Topics sensori (bumper, sonar, touch, battery)
- qi_unipa_2_audio: Trascrizione, localizzazione audio
- qi_unipa_2_vision: Servizi camera e rilevamento emozioni
- qi_unipa_2_tracking: Servizi tracking e streaming coordinate
- qi_unipa_2_reference: Servizio posizione, topics IMU e infrarossi


================================================================================
TESTING
================================================================================

-----------------------------------
VERIFICA STRUTTURA NAMESPACE
-----------------------------------

Lista tutte le action Pepper:
    ros2 action list | grep pepper

Lista tutti i service Pepper:
    ros2 service list | grep pepper

Lista tutti i topic Pepper:
    ros2 topic list | grep pepper


-----------------------------------
TEST ACTIONS
-----------------------------------

Test walking:
    ros2 action send_goal /pepper/actions/walking qi_unipa_2_interfaces/action/Walking "{x: 1.0, y: 0.5, theta: 0.0}"

Test talking:
    ros2 action send_goal /pepper/actions/talking qi_unipa_2_interfaces/action/Talking "{message: 'Ciao da Pepper!'}"


-----------------------------------
TEST SERVICES
-----------------------------------

Ottieni posizione:
    ros2 service call /pepper/services/get_position qi_unipa_2_interfaces/srv/GetPosition

Cattura immagine:
    ros2 service call /pepper/services/get_image qi_unipa_2_interfaces/srv/GetImage "{camera_index: 0, resolution: 1}"


-----------------------------------
MONITOR TOPICS
-----------------------------------

Monitor sensori:
    ros2 topic echo /pepper/topics/bumper
    ros2 topic echo /pepper/topics/sonar

Monitor trascrizione:
    ros2 topic echo /pepper/topics/transcription


================================================================================
ESEMPIO INTEGRAZIONE
================================================================================

-----------------------------------
CLIENT PYTHON (pepper_ai)
-----------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from qi_unipa_2_interfaces.action import Walking, Talking
from qi_unipa_2_interfaces.srv import GetImage
from qi_unipa_2_interfaces.msg import Bumper

class PepperAI(Node):
    def __init__(self):
        super().__init__('pepper_ai')
        
        # Action clients
        self.walking_client = ActionClient(
            self, 
            Walking, 
            '/pepper/actions/walking'
        )
        self.talking_client = ActionClient(
            self, 
            Talking, 
            '/pepper/actions/talking'
        )
        
        # Service clients
        self.get_image_client = self.create_client(
            GetImage, 
            '/pepper/services/get_image'
        )
        
        # Topic subscribers
        self.create_subscription(
            Bumper, 
            '/pepper/topics/bumper', 
            self.bumper_callback, 
            10
        )
    
    def bumper_callback(self, msg):
        self.get_logger().info(
            f'Bumper: left={msg.left}, right={msg.right}'
        )


================================================================================
BREAKING CHANGES
================================================================================

ATTENZIONE: Questa è una modifica BREAKING per codice esistente che usa il 
vecchio namespace. Aggiornare il codice client secondo la guida migrazione.

VERSIONI INTERESSATE: Tutte le versioni precedenti a v2.0.0


================================================================================
LAVORI FUTURI
================================================================================

- Aggiungere file constants.py per path namespace
- Implementare refactoring server HTTP (separato da nodo ROS2)
- Aggiungere unit test completi per tutte le interfacce
- Generare documentazione API automatica da definizioni interfacce


================================================================================
COMMIT GIT
================================================================================

git add .
git commit -m "feat: implementato namespace unificato per API ROS2

- Standardizzate tutte le action sotto /pepper/actions/*
- Standardizzati tutti i service sotto /pepper/services/*
- Standardizzati tutti i topic sotto /pepper/topics/*
- Aggiornati tutti i 9 nodi per usare nuovo namespace
- Migliorato supporto multi-robot e integrazione
- Aggiunta documentazione API completa

BREAKING CHANGE: Tutti i path interfacce ROS2 sono cambiati.
Package esterni devono aggiornare codice client. Vedi guida migrazione."

git push origin main


================================================================================
FINE DOCUMENTO
================================================================================

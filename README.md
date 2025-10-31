# qi_unipa_2

Versione aggiornata del package "qi_unipa" creato dai colleghi del laboratorio dell'università di Palermo tra il 2024 e 2025. Questa versione implementa le stesse funzionalità precedenti ma diverse di queste sono state convertite da topic ad action e altre da topic a service. Sono presenti DUE package: uno è l'interfaccia contenente i services, actions e topics (che secondo ROS2 Humble DEVONO essere predisposti separatamente in un package munito di CMakeLists) e l'altro è il package vero e proprio con i nodi di implementazione.

## Modifiche Implementate

### Nodo Movement (qi_unipa_2_movement.py)
- /move (topic) -> Action Walking con feedback in tempo reale durante la marcia
- /set_joint (topic) -> Service SetJointAngles per controllo sincrono degli angoli articolari
- /posture (topic) -> Service SetPosture per cambi postura atomici
- /get_position (topic) -> Service GetPosition per query on-demand della posizione attuale
- Feedback dell'action Walking: feedback continuo con status durante il movimento
- Conferma sincrona delle operazioni via service

### Nodo Speech (qi_unipa_2_speech.py)
- /speak (topic) -> Action Talking con feedback durante la sintesi vocale
- Supporto per agente e percept come parametri del goal
- Feedback booleano is_talking in tempo reale
- Possibilità di cancellare il goal mid-speech

### Nodo Vision (qi_unipa_2_vision.py)
- Richieste immagini -> Service GetImage (request-response pattern)
- Il precedente topic "get_camera" per il flusso video si chiama ora "get_video" (rimane uguale a prima)
- Parametri selezionabili: camera_index (top/bottom), risoluzione (QVGA/VGA/4VGA/HD)
- Risposta con immagine sensor_msgs/Image e flag di successo

### Nodo Sensor (qi_unipa_2_sensor.py)
- Implementazione di custom message types per flussi continui:
- Bumper.msg: dati sensori pressione (left, right, back)
- Sonar.msg: dati sensori ultrasonici (front, back)
- Track.msg: target tracking (target_name, distance)
- StringArray.msg: array di stringhe generiche

### Nodo STT (qi_unipa_2_stt.py)
- Precedentemente era un package separato "qi_unipa_stt" sviluppato dai colleghi
- Ora integrato come nodo interno al package qi_unipa_2
- Rimane topic-based per flusso continuo di trascrizioni
- Pubblicazione su topic /transcription con timestamp
- Gestione automatic speech recognition per Pepper

### Nodo Tablet (qi_unipa_2_tablet.py)
- Gestione interfaccia tablet tramite service-based calls
- Service SetState per cambio stato interfaccia

### Nodo Tracking (qi_unipa_2_tracking.py)
- Tracking visivo tramite topic continuo
- Pubblicazione dati tracking su /tracking

### Utilità (utils.py)
- Centralizzazione delle funzioni condivise precedentemente distribuite nei nodi
- Funzioni di validazione stato robot
- Helper di trasformazione coordinate
- Utility di connessione NAOqi
- Wrapper di gestione errori

### Separazione Package Interfaces
- Nuovo package dedicato qi_unipa_2_interfaces con:
- Cartella /action con Walking.action, Talking.action
- Cartella /srv con SetPosture.srv, SetJointAngles.srv, SetHand.srv, GetPosition.srv, GetImage.srv, SetState.srv
- Cartella /msg con Bumper.msg, Sonar.msg, Track.msg, StringArray.msg
- CMakeLists.txt corretto secondo standard ROS2 Humble
- Conforme a best practice ROS2 (interfaces separate dai nodi)

### Modalità Mockup
- Supporto testing senza robot fisico tramite parametro mockup:=True
- Simulazione connessione NAOqi
- Ritorno dati sensori sintetici
- Accettazione comandi senza esecuzione fisica
- Testing integrazione con pepper_ai

### Nodo Server Principale (qi_unipa_2_server.py)
- Orchestrazione di tutti i nodi ROS2
- Gestione lifecycle dei nodi
- Coordinamento asincronizzazione tra action/service/topic
- Ricezione e propagazione stati

## Struttura Architetturale

### Pattern Ibrido ROS2
- Actions: operazioni lunghe osservabili (Walking, Talking)
- Services: query/comandi request-response (GetPosition, SetJointAngles, GetImage, SetPosture, SetHand, SetState)
- Topics: flussi continui dati sensoriali (Bumper, Sonar, Track, transcription)


## Requisiti

- ROS2 Humble
- Python 3.10+
- NAOqi SDK (Softbank Robotics)
- std_msgs, sensor_msgs (ROS2)

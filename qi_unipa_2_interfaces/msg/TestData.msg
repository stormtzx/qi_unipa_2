# Messaggio per tutti i dati di test: cognitivo, MUST e test mobilità
# Usato per passare i dati all'alto livello
# Pubblicato su /pepper/topics/UserTestData

# === METADATI ===
string timestamp                    # Timestamp acquisizione (ISO 8601)
bool success                        # True se dati validi
string test_type                     # Tipo form: "minicog_parte1", "minicog_parte2", "minicog_parte3", "minicog_complete",
                                    #            "mobilita_test1", "mobilita_test2", "mobilita_test3", "mobilita_test4", "mobilita_test5", "mobilita_complete",
                                    #            "must_parte1", "must_parte2", "must_parte3", "must_complete",
                                    #            "registrazione"

# === TEST MINICOG (4 parti) ===
# minicog_parte1: Parole memorizzazione (visualizza solo le parole, non c'è invio dati)

# minicog_parte2: Orologio
int32 minicog_parte2_score          # 0, 3 o 7 punti
int32 minicog_parte2_ore            # Ore impostate
int32 minicog_parte2_minuti         # Minuti impostati

# minicog_parte3: Disegno orologio
int32 minicog_parte3_score          # Fisso 1 punti
# string minicog_parte3_disegno_base64 # Disegno in base64, decommentare se si vuole inviare anche l'immagine

# minicog_parte4: Verifica parole
int32 minicog_parte4_score          # 0-3 punti
string[] minicog_parte4_selezionate # Parole selezionate utente
int32 minicog_parte4_corrette       # Numero parole corrette
int32 minicog_parte4_sbagliate      # Numero parole sbagliate

# minicog_complete: Riepilogo finale
int32 minicog_punteggio_totale      # Somma tutti i punteggi (0-14)

# === TEST MOBILITÀ (5 parti) ===
# mobilita_test1: Equilibrio piedi affiancati
int32 mobilita_test1_score          # 0, 1 o 2
string mobilita_test1_stato         # "≥10s", "3-10s", "<3s"

# mobilita_test2: Equilibrio piedi sfalsati
int32 mobilita_test2_score          # 0, 1 o 2
string mobilita_test2_stato         # "≥10s", "3-10s", "<3s"

# mobilita_test3: Equilibrio tallone-punta
int32 mobilita_test3_score          # 0, 1 o 2
string mobilita_test3_stato         # "≥20s", "10-20s", "<10s"

# mobilita_test4: Camminata 4 metri
int32 mobilita_test4_score          # 0, 1, 2 o 3
string mobilita_test4_stato         # "0-4s", "4-6s", "6-9s", "9-60s"

# mobilita_test5: Sollevamento da sedia
int32 mobilita_test5_score          # 0, 1, 2 o 3
string mobilita_test5_stato         # "0-12s", "12-15s", "15-18s", "18-60s"

# mobilita_complete: Riepilogo finale
int32 mobilita_punteggio_totale     # Somma tutti i punteggi (0-13)

# === TEST MUST (3 parti) ===
# must_parte1: BMI
int32 must_parte1_score             # 0, 1 o 2 (basato su BMI)
float32 must_parte1_bmi             # Valore BMI calcolato
float32 must_parte1_peso            # Peso in kg
float32 must_parte1_altezza         # Altezza in cm
string must_parte1_stato            # "Sottopeso", "Normopeso", "Sovrappeso", "Obeso"

# must_parte2: Perdita di peso
int32 must_parte2_score             # 0, 1 o 2
string must_parte2_stato            # "Nessuna perdita", "Lieve perdita", "Notevole perdita"

# must_parte3: Digiuno
int32 must_parte3_score             # 0, 1 o 2
string must_parte3_stato            # "0-10 giorni", "1-2 settimane", "Più di 2 settimane"

# must_complete: Riepilogo finale
int32 must_punteggio_totale         # Somma tutti i punteggi (0-6)
string must_stato_nutrizionale      # "Malnutrito", "Normale", "Ben nutrito"

# === DATI GENERALI ===
string test_name                    # Nome test (es: "MiniCog", "Mobilità", "MUST")
string page_name                    # Nome pagina HTML (es: "test_minicog_parte1.html")
string test_data                    # JSON generico per dati non strutturati

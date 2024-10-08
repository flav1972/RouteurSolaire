/* 
  Routeur solaire d'après la version développé par le Profes'Solaire v9.15 - 28-11-2023 - professolaire@gmail.com

  TODO
  ajouter forcage par pin et forcage pour une duree
*/


/*
   Configuration de connexion sur le WiFi d'une box
   créer le fichier wifi_config.h
   qui doit contenir les 2 lignes suivantes:
#define SSID  "xxxxxxxxxxxxx"                            // nom de votre réseau wifi
#define PASSWORD  "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx "       // mot de passe de votre réseau wifi
*/

// Librairies (voir Readme)
#include <HardwareSerial.h>     // 2nd Port Serie- dans Board esp32 by Espressif 2.0.17
#include <RBDdimmer.h>          // gestion des Dimmers:  https://github.com/flav1972/RBDDimmer - github tag 1.1
#include <U8g2lib.h>            // gestion affichage écran Oled:  https://github.com/olikraus/U8g2_Arduino/ - lib U8g2 2.34.22
#include <Wire.h>               // pour connexion au bus I2C (écran) - dans Board esp32 by Espressif 2.0.17
#include <WiFi.h>               // gestion du wifi - dans Board esp32 by Espressif 2.0.17
#include <ESPDash.h>            // page web Dash:  https://github.com/ayushsharma82/ESP-DASH - lib ESP-DASH 3.0.8
#include <AsyncTCP.h>           // pour ESPAsyncWebserver: https://github.com/me-no-dev/AsyncTCP  - lib AsyncTCP 3.1.4
#include <ESPAsyncWebServer.h>  // pour serveur web: https://github.com/me-no-dev/ESPAsyncWebServer  - git commit 7f3753454b1f176c4b6d6bcd1587a135d95ca63c
                                // et https://github.com/bblanchon/ArduinoJson - lib ArduinoJson 7.1.00
#include <ArduinoOTA.h>         // mise à jour OTA par wifi - dans Board esp32 by Espressif 2.0.17
#include <DNSServer.h>          // serveur DNS - dans Board esp32 by Espressif 2.0.17
#if defined __has_include
#  if __has_include ("wifi_config.h")
#    include "wifi_config.h"
#    define USESTA
#  endif
#endif

// Coefficient pour simuler des charges reeles avec des petites charges
//#define CoefSimulation 60.0

#ifndef CoefSimulation
#define CoefSimulation 1.0
#endif

/*
 * Configuration pour Wifi Mode STA
 */
/* Set these to your desired credentials. */
const char *soft_ap_ssid_start = "SOLAR_";
char soft_ap_ssid[15];

/* Broches utilisées */
const int zeroCrossPin = 35; /* broche utilisée pour le zéro crossing */
const int pulsePin1 = 25;    /* broche impulsions routage 1*/
const int pulsePin2 = 26;    /* broche impulsions routage 2*/
const int forcagePin = 34;   // broche pour forcer la mache: 3.3v = forcage

// Adresse Ports Amperemetre
#define RXD2 16
#define TXD2 17

#define SER_BUF_SIZE 4096

byte ByteArray[130]; // les donnees brutes du JSY
long ByteData[14];   // les donnees converties du JSY

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);


//déclaration des variables
// pour le calcul de regulation
float Kp = 0;
float Ki = 0.004; 
float Kd = -5;

// pour le calcul de regulation
float ajusteConso = 0.0;        // puissance consomme par le balon - puissance produite = +/- power2 = - puissance injectee*/
float ajusteConso_n1 = 0.0;     // valeur en n-1
float ajusteConso_n2 = 0.0;     // valeur en n-2
float pas_dimmer;
float valDim = 0.0;             // somme des valeurs des dimmers
float valDim1 = 0.0;
float valDim2 = 0.0;
const float maxDimmer1 = 97.0;  // Forcage à 100% au dela de cette valeur pour eviter les clignottements
const float maxDimmer2 = 97.0;  // Forcage à 100% au dela de cette valeur pour eviter les clignottements
const float minDimmer = 0.0;
const float maxDimmer = 100.0;  // Valeur theorique maximale d'un dimmer
const float maxDimmers = 2 * maxDimmer;


// Donnees de l'amperemetre
float Voltage;                                         // Tension
float Intensite1;                                      // Courant dans le capteur sur la platine
float EnergyPos1;                                      // Energie totale (somme des positifs) passe dans le capteur 1
float EnergyNeg1;                                      // Energie totale (somme des negatifs) passe dans le capteur 1
float Frequency;                                       // Frequence
float PowerFactor1;                                    // Facteur de puissance (dephasage enter I et V)
float Intensite2;                                      // Intensite dans le capteur deporte
float EnergyPos2;                                      // Energie active passe dans le capteur 2 = energie injectee
float EnergyNeg2;                                      // Negative energy capteur 2 = consomation d'energie
int Sens1;                                           // Sens dans le capteur 1 : non utilise
int Sens2;                                           // Sens dans le captuer 2 : 0 = on consomme, 1 = on produit
float Power1;                                            // puissance envoyée au ballon + chauffages (capteur 1)
float Power2;                                            // puissance entrant ou sortant de l'habitation

unsigned long currentTimeTask1 = 0;                    // temps actuel dans la tache1
unsigned long previousTimeTask1 = 0;                   // temps precedent dans la tache1

unsigned long currentTimeTask2 = 0;                    // temps actuel dans la tache2
unsigned long previousTimeWifi = 0;                    // temps de la derniere tentative de reconnexion wifi

boolean lastmarcheforceepin;      // statut marche par le pin : forcage off au depart
boolean marcheforcee = 0;         // statut marche : forcage off au depart
boolean oled = 1;                 // statut écran Oled : allumé au depart

///  configuration serveur web ///
AsyncWebServer server(80);

// Page pas trouvee envoye vers /
void notFound(AsyncWebServerRequest *request) {
      request->redirect("/");
}

ESPDash dashboard(&server);
Card consommationsurplus(&dashboard, GENERIC_CARD, "Surplus (+=injection, -=conso)", "Watts");
Card puissance(&dashboard, GENERIC_CARD, "Consumation Ballon+Planchers", "Watts");
Card energieSauvee(&dashboard, GENERIC_CARD, "Energie sauvée totale", "kwh");
Card energieInject(&dashboard, GENERIC_CARD, "Energie injectée totale", "kwh");
Card energieConso(&dashboard, GENERIC_CARD, "Consommation Enedis totale", "kWh");
Card valdim1(&dashboard, PROGRESS_CARD, "Triac 1", "%", 0, 95);
Card valdim2(&dashboard, PROGRESS_CARD, "Triac 2", "%", 0, 95);
Card BOled(&dashboard, BUTTON_CARD, "Écran On/Off");
Card Bmarcheforcee(&dashboard, BUTTON_CARD, "Marche forcée On/Off");

// DNS pour le mode AP
DNSServer dnsServer;

// tache loop
TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t binsem1;

// prototype fonctions attachées aux taches
void Task1code(void *);
void Task2code(void *);

// dimmer
dimmerLamp dimmer1(pulsePin1, zeroCrossPin);
dimmerLamp dimmer2(pulsePin2, zeroCrossPin);

// prototype fonction pour la programmation par le WiFi
void initOTA(); 

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(SER_BUF_SIZE);
  Serial2.begin(38400, SERIAL_8N1, RXD2, TXD2);  //PORT DE CONNEXION AVEC LE CAPTEUR JSY-MK-194
  delay(300);
  Serial.println("Routeur Solaire Starting...");
  
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.printf("SDK Version is : %s\n", esp_get_idf_version());
  
  pinMode(forcagePin, INPUT_PULLDOWN);          // pin marche forcee en Input et ressistance pulldown
  marcheforcee = lastmarcheforceepin = digitalRead(forcagePin);

  u8g2.begin();            // ECRAN OLED
  u8g2.enableUTF8Print();  //nécessaire pour écrire des caractères accentués
  dimmer1.begin(NORMAL_MODE, ON); /// Pourquoi y a pas le dimmer 2 , si 2eme dimmer, ca marche plus
  dimmer1.setPower(0);
  dimmer2.setMode(NORMAL_MODE);

  dimmer2.setPower(0);
  delay(100);
  uint64_t chip64 = ESP.getEfuseMac();           // recupere la MAC adresse (qui est a l envers)
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {               // retourne les 3 premiers octets
    chipId |= ((chip64 >> (40 - i)) & 0xff) << i;
  }
  snprintf(soft_ap_ssid, 15, "%s%06X", soft_ap_ssid_start, chipId);
  #ifdef USESTA
  WiFi.mode(WIFI_MODE_APSTA);                    // Mode Acces Point et Station Wifi
  #else
  WiFi.mode(WIFI_MODE_AP);                       // Mode Acces Point
  #endif
  WiFi.softAP(soft_ap_ssid);                     // Configuration de l'acces point (sans mot de passe)
  dnsServer.start(53, "*", WiFi.softAPIP());     // Lancement du serveur DNS

  #ifdef USESTA
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi...");
  for (int i = 5; i >= 0; i--) {
    delay(1000);
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    Serial.print(i);
    Serial.print(" ");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wifi STA not connected");
  }
  #endif
  
  Serial.print("ESP32 IP as soft AP: ");
  Serial.println(WiFi.softAPIP());
 
  // configuration de la page web pas trouvee
  server.onNotFound(notFound);
  
  // Attachement de pages web
  server.on("/datas", HTTP_GET, handleDataRequest);
  server.on("/pid", HTTP_GET, handlePIDRequest);

  // lancement du serveur web
  server.begin();
  delay(100);

  initOTA();  // initialisation OTA Wifi

  // Boutons ESP-DASH interactifs
  BOled.attachCallback([&](bool value) {
    oled = value;
    BOled.update(oled);
    dashboard.sendUpdates();
  });

  Bmarcheforcee.attachCallback([&](bool value) {
    marcheforcee = value;
    Bmarcheforcee.update(oled);
    dashboard.sendUpdates();
  });


  // create a binary semaphore for task synchronization
  binsem1 = xSemaphoreCreateBinary();

  //Code pour créer un Task Core 0//
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //Code pour créer un Task Core 1//
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}

// Gestion requetteweb
void handleDataRequest(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/plain");
  response->print("Routeur Solaire\n");
  response->printf("Vous avez essaye de joindre la page: http://%s%s\n", request->host().c_str(), request->url().c_str());
  response->printf("ajusteConso = %d\n", ajusteConso);
  response->printf("valDim = %f\n", valDim);
  response->printf("Kp = %f\n", Kp);
  response->printf("Ki = %f\n", Ki);
  response->printf("Kd= %f\n", Kd);
  response->printf("Voltage = %f\n", Voltage);
  response->printf("Intensite1 = %f\n", Intensite1);
  response->printf("Power1 = %f\n", Power1);
  response->printf("EnergyPos1 = %f\n", EnergyPos1);
  response->printf("EnergyNeg1 = %f\n", EnergyNeg1);
  response->printf("Sens1 = %d\n", Sens1);
  response->printf("Sens2 = %d\n", Sens2);
  response->printf("Frequency = %f\n", Frequency);
  response->printf("Intensite2 = %f\n", Intensite2);
  response->printf("Power2 = %f\n", Power2);
  response->printf("EnergyPos2 = %f\n", EnergyPos2);
  response->printf("EnergyNeg2 = %f\n", EnergyNeg2);
  response->printf("ajusteConso= %f\n", ajusteConso);
  request->send(response);
}

// Mise à jour des parametres PID par http
const char* KP_PARAM = "kp";
const char* KI_PARAM = "ki";
const char* KD_PARAM = "kd";
char buff[100];
void handlePIDRequest(AsyncWebServerRequest *request) {
  String string;
  if (request->hasParam(KP_PARAM)) {
      string = request->getParam(KP_PARAM)->value();
      Kp = string.toFloat();
  }
  if (request->hasParam(KI_PARAM)) {
      string = request->getParam(KI_PARAM)->value();
      Ki = string.toFloat();
  }
  if (request->hasParam(KD_PARAM)) {
      string = request->getParam(KD_PARAM)->value();
      Kd = string.toFloat();
  }
  snprintf(buff, sizeof(buff), "Hello, GET K values : Kp=%f, Ki=%f, Kd=%f", Kp, Ki, Kd);
  request->send(200, "text/plain", buff);
}

// Programmation par OTA
void initOTA() {

  ArduinoOTA.setHostname(soft_ap_ssid);                           // SOLAR_XXXXXX comme pour le SSID AP
  ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3"); // MD5 Hash of password

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}


void PrintDatas() {
  /*
  Serial.print("Voltage:");
  Serial.print(Voltage);
  Serial.print(",Intensite1:");
  Serial.print(Intensite1);
  */
  Serial.print("Power1:");
  Serial.print(Power1);
  /*
  Serial.print(",EnergyPos1:");
  Serial.print(EnergyPos1);
  Serial.print(",EnergyNeg1:");
  Serial.print(EnergyNeg1);
  Serial.print(",Sens1:");
  Serial.print(Sens1);
  Serial.print(",Sens2:");
  Serial.print(Sens2);
  Serial.print(",Frequency:");
  Serial.print(Frequency);
  Serial.print(",Intensite2:");
  Serial.print(Intensite2);
  Serial.print(",Power2:");
  Serial.print(Power2);
  Serial.print(",EnergyPos2:");
  Serial.print(EnergyPos2);
  Serial.print(",EnergyNeg2:");
  Serial.print(EnergyNeg2);
  */
  Serial.print(",ajusteConso:");
  Serial.print(ajusteConso);
}

// Lecture des données de puissance/courant
void Datas() {
  int i, j;
  byte msg[] = { 0x01, 0x03, 0x00, 0x48, 0x00, 0x0E, 0x44, 0x18 };
  int len = 8;

  //Serial.println("Read Amp Data");

  // vide le buffer
  while (Serial2.available()) {
    Serial2.read();
  }

  ////// Envoie des requêtes Modbus RTU sur le Serial port 2
  for (i = 0; i < len; i++) {
    Serial2.write(msg[i]);
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  delay(50); // une petite pause pour attendre l'arrivée des données

  ////////// Reception  des données Modbus RTU venant du capteur JSY-MK-194 ////////////////////////
  int a = 0;
  while (Serial2.available()) {
    ByteArray[a] = Serial2.read();
    a++;
  }

  if(a != 61) {
    Serial.println("donnees invalides");
    return;
  }

  //////// Conversion HEX /////////////////
  j = 3;
  for (i = 0; i < 14; i++) {  // conversion séries de 4 octets en long
    ByteData[i] = ByteArray[j++] << 24;
    ByteData[i] += ByteArray[j++] << 16;
    ByteData[i] += ByteArray[j++] << 8;
    ByteData[i] += ByteArray[j++];
  }

  ///////// Normalisation des valeurs ///////////////
  #define OFFSETJSY -1
  Voltage = ByteData[1+OFFSETJSY] * 0.0001;                       // Tension
  Intensite1 = ByteData[2+OFFSETJSY] * 0.0001 * CoefSimulation;   // Intensité 1 en A
  Power1 = ByteData[3+OFFSETJSY] * 0.0001 * CoefSimulation;       // Puissance 1 en W
  EnergyPos1 = ByteData[4+OFFSETJSY] * 0.0001 * CoefSimulation;   // Energie 1 somme des Positifs: "produite" par le balon et le chauffage
  EnergyNeg1 = ByteData[6+OFFSETJSY] * 0.0001 * CoefSimulation;   // Energie 1 somme des Negatifs: "consommee" par le balon et le chauffage

  Sens1 = ByteArray[27];  // Sens 1
  Sens2 = ByteArray[28];
  Frequency = ByteData[8+OFFSETJSY] * 0.01;                       // Fréquence en Hz
  Intensite2 = ByteData[10+OFFSETJSY] * 0.0001 * CoefSimulation;  // Intensité 2 en A
  Power2 = ByteData[11+OFFSETJSY] * 0.0001 * CoefSimulation;      // Puissance 2 en W
  EnergyPos2 = ByteData[12+OFFSETJSY] * 0.0001 * CoefSimulation;  // Energie 2 positive (injectee) en Wh
  EnergyNeg2 = ByteData[14+OFFSETJSY] * 0.0001 * CoefSimulation;  // Energie 2 negative (consommation) en Wh

  if (Sens2 == 0) { // On produit
    ajusteConso = Power2;
  }

  if (Sens2 == 1) { // On Consomme
    ajusteConso = -Power2;
  }
  PrintDatas();
}

//programme utilisant le Core 1 de l'ESP32//

void Task1code(void *pvParameters) {
  int mfpin;  // pour la lecture du pin de marche forcee

  unsigned long durationCalc_n = 0;    // delta temps du calcul courant
  unsigned long durationCalc_n_1 = 0;  // delta temps du calcul precedent

  // PID fonction :
  // E est l'erreur
  // d_commande[n] = (Ki + Kp/dt + Kd/dt2) * E[n] + (-Kp/dt-2*Kd/dt2) * E[n-1] + Kd/dt2*E[n-2]
  float Kd_factor; // facteur dependant de Kd/dt (derivée)
  float Kp_factor; // facteur dependant de Kp/dt2 (derivée seconde)
  for (;;) {
    currentTimeTask1 = millis();
    durationCalc_n_1 = durationCalc_n;
    durationCalc_n =  currentTimeTask1 - previousTimeTask1;
    previousTimeTask1 = currentTimeTask1;

    Datas();

    if (durationCalc_n == 0 || Kp == 0.0) {
      Kp_factor = 0.0;  
    }
    else {
      Kp_factor = Kp / float(durationCalc_n); 
    }

    if (durationCalc_n == 0 || durationCalc_n_1 == 0) {
      Kd_factor = 0.0;  
    }
    else {
      Kd_factor = Kd / float(durationCalc_n * durationCalc_n_1); 
    }
    pas_dimmer = (Ki + Kp_factor + Kd_factor)*ajusteConso - (Kp_factor + 2*Kd_factor)*ajusteConso_n1 + Kd_factor*ajusteConso_n2;
    
    ajusteConso_n2 = ajusteConso_n1;
    ajusteConso_n1 = ajusteConso;

    valDim = valDim + pas_dimmer;
    if (valDim > maxDimmers) {
      valDim = maxDimmers;
    }
    else if (valDim < minDimmer) {
      valDim = minDimmer;
    }

    valDim1 = min(valDim, maxDimmer);

    // gestion de la marche forcee
    mfpin = digitalRead(forcagePin);
    if(mfpin != lastmarcheforceepin) {
      marcheforcee = lastmarcheforceepin = mfpin;
    }

    if(marcheforcee) {
      Serial.println("Marche Forcee");
      valDim1 = maxDimmer;
    }

    // dimmer 2
    valDim2 = max(minDimmer, valDim - maxDimmer);

    if(valDim1 == minDimmer) {
      dimmer1.setState(OFF);
    }
    else if(valDim1 >= maxDimmer1) {
      dimmer1.setMode(TOGGLE_MODE);
      dimmer1.setState(ON);
    }
    else {
      dimmer1.setMode(NORMAL_MODE);
      dimmer1.setState(ON);
    }

    if(valDim2 == minDimmer) {
      dimmer2.setState(OFF);
    }
    else if(valDim2 >= maxDimmer2) {
      dimmer2.setMode(TOGGLE_MODE);
      dimmer2.setState(ON);
    } else {
      dimmer2.setMode(NORMAL_MODE);
      dimmer2.setState(ON);
    }

    dimmer1.setPower(valDim1);
    delay(60);
    dimmer2.setPower(valDim2);
    delay(60);

    Serial.print(",pas_dimmer:");
    Serial.print(pas_dimmer, 3);

    Serial.print(",valDim:");
    Serial.print(valDim);

    Serial.print(",valDim1:");
    Serial.print(valDim1);

    Serial.print(",valDim2:");
    Serial.println(valDim2);

    dnsServer.processNextRequest(); // On processe ici les requttes DNS
    delay(200);                     // On boucle 5 fois par seconde environ
  }
}



//programme utilisant le Core 2 de l'ESP32//

void Task2code(void *pvParameters) {

  for (;;) {
    ArduinoOTA.handle();

    currentTimeTask2 = millis();

    //////////////////////////////////////////////////////////////////
    ///////////// Reconnexion wifi automatique ///////////////////////
    //////////////////////////////////////////////////////////////////
    #ifdef USESTA
    if ((WiFi.status() != WL_CONNECTED) && (currentTimeTask2 - previousTimeWifi >= 60000)) {
      WiFi.disconnect();
      WiFi.reconnect();
      Serial.println("reconnecting wifi...");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(WiFi.localIP());
      }
      previousTimeWifi = currentTimeTask2;
    }
    #endif

    ///////////////////////////////////////////////////////////////////
    ////////////// Fin reconnexion automatique Wifi ///////////////////
    ///////////////////////////////////////////////////////////////////


    // affichage page web DASH //
    consommationsurplus.update(ajusteConso);
    puissance.update(Power1);
    energieSauvee.update(EnergyNeg1-EnergyPos1);
    energieInject.update(EnergyPos2);
    energieConso.update(EnergyNeg2);
    valdim1.update(valDim1);
    valdim2.update(valDim2);
    BOled.update(oled);
    Bmarcheforcee.update(marcheforcee);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    dashboard.sendUpdates();

    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////// affichage écran ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    u8g2.clearBuffer();  // on efface ce qui se trouve déjà dans le buffer
    if (oled == 1) {
      u8g2.setFont(u8g2_font_6x13_tf);
      u8g2.setCursor(0, 12);      // position du début du texte
      u8g2.print("P Triacs: ");   // écriture puisance utilisée
      u8g2.print(Power1);
      u8g2.print(" W");
      u8g2.setCursor(0, 25);      // position du début du texte
      u8g2.print("Injection: ");  // écriture puisance reseau
      u8g2.print(ajusteConso);
      u8g2.print(" W");

      u8g2.setCursor(0, 39);
      u8g2.print("T1: ");
      u8g2.print((int)valDim1);
      u8g2.print("% T2: ");
      u8g2.print((int)valDim2);
      u8g2.print("%");   
      
      #ifdef USESTA
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 48);
      u8g2.print("IP: ");
      u8g2.print(WiFi.localIP());  // affichage adresse ip //
      #endif

      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 57);
      u8g2.print("AP IP: ");
      u8g2.print(WiFi.softAPIP());  // affichage adresse ip //
     

      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 63);
      u8g2.print("kWh sauves: ");  // écriture de texte
      u8g2.print(EnergyNeg1);  // écriture de texte
      
      u8g2.sendBuffer();  // l'image qu'on vient de construire est affichée à l'écran
    }

    if (oled == 0) {
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    ////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////// Fin affichage écran //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    delay(500); // un update 2 fois par seconde est suffisant
  }
}

void loop() {
}

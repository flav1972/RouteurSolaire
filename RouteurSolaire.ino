/* TODO
** supression forcage par energie journaliere
** mette a jour schema pour entre ZC du 2eme triac
** ajouter forcage par pin et forcage pour une duree
** ajouter mode AP + STA
*/

/* Routeur solaire développé par le Profes'Solaire v9.15 - 28-11-2023 - professolaire@gmail.com
Merci à Jean-Victor pour l'idée d'optimisation de la gestion des Dimmers
- 2 sorties 16A / 3000 watts
- 1 relais on/off
- 1 serveur web Dash Lite avec On / Off
- heure NTP
- relay marche forcée : 16A mini
- marche forcée automatique suivant surplus et par rapport au volume ballon
- marche forcée automatique avec sonde de température : 50 degrés min
- mise à jour OTA en wifi
 * ESP32 + JSY-MK-194 (16 et 17) + Dimmer1 24A-600V (35 ZC et 25 PW) + Dimmer 2 24A-600V ( 35 ZC et 26 PW) + écran Oled (22 : SCK et 21 SDA) + relay (13) + relay marche forcée (32) + sonde température DS18B20 (4)
 Utilisation des 2 Cores de l'Esp32
*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// CONFIGURATION ///// PARTIE A MODIFIER POUR VOTRE RESEAU //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "wifi_config.h"
/*
le fichichier wifi_config.h doit contenir les 2 lignes suivantes:
const char* ssid = "xxxxxxxxxxxxx";                            // nom de votre réseau wifi
const char* password = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx ";       // mot de passe de votre réseau wifi
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Librairies //

#include <HardwareSerial.h>     // https://github.com/espressif/arduino-esp32
#include <RBDdimmer.h>          // gestion des Dimmers  https://github.com/RobotDynOfficial/RBDDimmer // github
#include <U8g2lib.h>            // gestion affichage écran Oled  https://github.com/olikraus/U8g2_Arduino/ //
#include <Wire.h>               // pour esp-Dash
#include <WiFi.h>               // gestion du wifi
#include <ESPDash.h>            // page web Dash  https://github.com/ayushsharma82/ESP-DASH //
#include <AsyncTCP.h>           //  https://github.com/me-no-dev/AsyncTCP  ///
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer  et https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>         // mise à jour OTA par wifi

// Coefficient pour simuler des charges reeles avec des petites charges
#define CoefSimulation 60.0

#ifndef CoefSimulation
#define CoefSimulation 1.0
#endif

// Adresse Ports Amperemetre
#define RXD2 16
#define TXD2 17

byte ByteArray[250];
int ByteData[20];

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);



//déclaration des variables//

// pour le calcul de regulation
float routagePuissance = -30; /* puissance minimale autour du zero */
int ajustePuissance = 0;      /* Puissance a consommer par le balon (negatif: on doit consommer) */
float puissanceRoutageOK;
float pas_dimmer;
float valDim1 = 0;
float valDim2 = 0;
float maxDimmer = 100;
float maxDimmer2 = 97;
float minDimmer = 0;

// Donnees de l'amperemetre
float Voltage;                                         // Tension
float Intensite1;                                      // Courant dans le capteur sur la platine
float Energy1;                                         // Energie totale passe dans le capteur 1
float Frequency;                                       // Frequence
float PowerFactor1;                                    // Facteur de puissance (dephasage enter I et V)
float Intensite2;                                      // Intensite dans le capteur deporte
float Energy2;                                         // Energie active passe dans le capteur 2 = energie injectee
float Energy2C;                                        // Negative energy capteur 2 = consomation d'energie
float Sens1;                                           // Sens dans le capteur 1 : non utilise
float Sens2;                                           // Sens dans le captuer 2 : 0 = on consomme, 1 = on produit
int Power1;                                            // puissance envoyée au ballon + chauffages (capteur 1)
int Power2;                                            // puissance entrant ou sortant de l'habitation

unsigned long currentTimeTask1 = 0;                    // temps actuel dans la tache1
unsigned long currentTimeTask2 = 0;                    // temps actuel dans la tache2
unsigned long previousTimeWifi = 0;                    // variable temps pour reconnexion wifi
boolean oled = 1;                 // écran Oled allumé

///  configuration serveur web ///
AsyncWebServer server(80);
WiFiClient espClient;
ESPDash dashboard(&server);
Card consommationsurplus(&dashboard, GENERIC_CARD, "Surplus (+=injection, -=conso)", "Watts");
Card puissance(&dashboard, GENERIC_CARD, "Consumation Ballon+Planchers", "Watts");
Card energy1(&dashboard, GENERIC_CARD, "Energie sauvée totale", "kwh");
Card energy2(&dashboard, GENERIC_CARD, "Energie injectée totale", "kwh");
Card energy2C(&dashboard, GENERIC_CARD, "Consommation Enedis totale", "kWh");
Card Oled(&dashboard, BUTTON_CARD, "Écran On/Off");
Card valdim1(&dashboard, PROGRESS_CARD, "Triac 1", "%", 0, 95);
Card valdim2(&dashboard, PROGRESS_CARD, "Triac 2", "%", 0, 95);

////////////// Fin connexion wifi //////////

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t binsem1;

/* Broches utilisées */
const int zeroCrossPin = 35; /* broche utilisée pour le zéro crossing */
const int pulsePin1 = 25;    /* broche impulsions routage 1*/
const int pulsePin2 = 26;    /* broche impulsions routage 2*/
const int oneWireBus = 4;    // broche du capteur DS18B20 //

dimmerLamp dimmer1(pulsePin1, zeroCrossPin);
dimmerLamp dimmer2(pulsePin2, zeroCrossPin);

void initOTA();  // déclaré plus bas
void Task1code(void *);
void Task2code(void *);

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8N1, RXD2, TXD2);  //PORT DE CONNEXION AVEC LE CAPTEUR JSY-MK-194
  delay(300);
  Serial.println("Routeur Solaire Starting...");
  u8g2.begin();            // ECRAN OLED
  u8g2.enableUTF8Print();  //nécessaire pour écrire des caractères accentués
  dimmer1.begin(NORMAL_MODE, ON); /// Pourquoi y a pas le dimmer 2 , si 2eme dimmer, ca marche plus
  dimmer1.setPower(0);
  delay(100);
  WiFi.mode(WIFI_STA);  //Optional
  WiFi.begin(ssid, password);
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
  server.begin();
  delay(100);

  initOTA();  // initialisation OTA Wifi

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

// Programmation par OTA
void initOTA() {

  ArduinoOTA.setHostname("Profes'Solaire routeur");
  ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

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
  Serial.print("Voltage: ");
  Serial.print(Voltage);
  Serial.print(", Intensite1: ");
  Serial.print(Intensite1);
  Serial.print(", Power1: ");
  Serial.print(Power1);
  Serial.print(", Energy1: ");
  Serial.print(Energy1);
  Serial.print(", Sens1: ");
  Serial.print(Sens1);
  Serial.print(", Sens2: ");
  Serial.print(Sens2);
  Serial.print(", Frequency: ");
  Serial.print(Frequency);
  Serial.print(", Intensite2: ");
  Serial.print(Intensite2);
  Serial.print(", Power2: ");
  Serial.print(Power2);
  Serial.print(", Energy2: ");
  Serial.print(Energy2);
  Serial.print(", Energy2C: ");
  Serial.print(Energy2C);
  Serial.print(", ajustePuissance: ");
  Serial.println(ajustePuissance);
}

// Lecture des données de puissance/courant
void Datas() {
  Serial.println("Read Amp Data");
  vTaskDelay(60 / portTICK_PERIOD_MS);

  byte msg[] = { 0x01, 0x03, 0x00, 0x48, 0x00, 0x0E, 0x44, 0x18 };
  int i;
  int len = 8;

  ////// Envoie des requêtes Modbus RTU sur le Serial port 2
  for (i = 0; i < len; i++) {
    Serial2.write(msg[i]);
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////// Reception  des données Modbus RTU venant du capteur JSY-MK-194 ////////////////////////
  int a = 0;
  while (Serial2.available()) {
    ByteArray[a] = Serial2.read();
    a++;
  }

  Serial.print("donnes recues:");
  Serial.println(a);
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if(a != 61) {
    Serial.println("donnees invalides");
    return;
  }

  //////// Conversion HEX /////////////////
  ByteData[1] = ByteArray[3] * 16777216 + ByteArray[4] * 65536 + ByteArray[5] * 256 + ByteArray[6];       // Tension en Volts
  ByteData[2] = ByteArray[7] * 16777216 + ByteArray[8] * 65536 + ByteArray[9] * 256 + ByteArray[10];      // Intensité 1 en Ampères
  ByteData[3] = ByteArray[11] * 16777216 + ByteArray[12] * 65536 + ByteArray[13] * 256 + ByteArray[14];   // Puissance 1 en Watts
  ByteData[4] = ByteArray[15] * 16777216 + ByteArray[16] * 65536 + ByteArray[17] * 256 + ByteArray[18];   // Energie 1 en kwh sauvées
  ByteData[7] = ByteArray[27];                                                                            // sens 1 du courant
  ByteData[9] = ByteArray[28];                                                                            // sens 2 du courant
  ByteData[8] = ByteArray[31] * 16777216 + ByteArray[32] * 65536 + ByteArray[33] * 256 + ByteArray[34];   // Fréquence en hz
  ByteData[10] = ByteArray[39] * 16777216 + ByteArray[40] * 65536 + ByteArray[41] * 256 + ByteArray[42];  // Intensité 2 en Ampères
  ByteData[11] = ByteArray[43] * 16777216 + ByteArray[44] * 65536 + ByteArray[45] * 256 + ByteArray[46];  // Puissance 2 en Watts
  ByteData[12] = ByteArray[47] * 16777216 + ByteArray[48] * 65536 + ByteArray[49] * 256 + ByteArray[50];  // Energie 2 en kwh injectee
  ByteData[14] = ByteArray[55] * 16777216 + ByteArray[56] * 65536 + ByteArray[57] * 256 + ByteArray[58];  // Energie 2 en kwh consommation

  ////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////// Normalisation des valeurs ///////////////
  Voltage = ByteData[1] * 0.0001;      // Tension
  Intensite1 = ByteData[2] * 0.0001;   // Intensité 1
  Power1 = ByteData[3] * 0.0001 * CoefSimulation;       // Puissance 1
  Energy1 = ByteData[4] * 0.0001;      // Energie 1 : envoyee vers le balon et le chauffage
  Sens1 = ByteData[7];                 // Sens 1
  Sens2 = ByteData[9];                 // Sens 2
  Frequency = ByteData[8] * 0.01;      // Fréquence
  Intensite2 = ByteData[10] * 0.0001;  // Intensité 2
  Power2 = ByteData[11] * 0.0001 * CoefSimulation;      // Puissance 2
  Energy2 = ByteData[12] * 0.0001;     // Energie 2 positive (injectee)
  Energy2C = ByteData[14] * 0.0001;    // Energie 2 negative (consommation)

  if (Sens2 == 1) { // On produit
    ajustePuissance = -Power2;
  }

  if (Sens2 == 0) { // On Consomme
    ajustePuissance = Power2;
  }
  Serial.print("Power1 : ");
  Serial.print(Power1);
  Serial.print(" / Power2=AjoustePuissance : ");
  Serial.println(ajustePuissance);
  PrintDatas();
}

//programme utilisant le Core 1 de l'ESP32//

void Task1code(void *pvParameters) {
  for (;;) {
    currentTimeTask1 = millis();

    Datas();

    // calcul triacs ///

    /// injection ok ///
    if (ajustePuissance <= 0 && ajustePuissance >= routagePuissance) {
      puissanceRoutageOK = 1;
    } else {
      puissanceRoutageOK = 0;
    }

    /// réglages pas Dimmer ///

    if (puissanceRoutageOK == 1) {
      pas_dimmer = 0.0;
    }

    else if (ajustePuissance <= -1000 && puissanceRoutageOK == 0) {
      pas_dimmer = 5.0;
    } else if (ajustePuissance > -1000 && ajustePuissance <= -800 && puissanceRoutageOK == 0) {
      pas_dimmer = 3.0;
    } else if (ajustePuissance > -800 && ajustePuissance <= -400 && puissanceRoutageOK == 0) {
      pas_dimmer = 2.0;
    } else if (ajustePuissance > -400 && ajustePuissance <= -300 && puissanceRoutageOK == 0) {
      pas_dimmer = 1.0;
    } else if (ajustePuissance > -300 && ajustePuissance <= -200 && puissanceRoutageOK == 0) {
      pas_dimmer = 0.75;
    } else if (ajustePuissance > -200 && ajustePuissance <= -100 && puissanceRoutageOK == 0) {
      pas_dimmer = 0.5;
    } else if (ajustePuissance > -100 && ajustePuissance <= -50 && puissanceRoutageOK == 0) {
      pas_dimmer = 0.1;
    } else if (ajustePuissance > -50 && ajustePuissance <= routagePuissance && puissanceRoutageOK == 0) {
      pas_dimmer = 0.05;
    }

    else if (ajustePuissance >= 1000 && puissanceRoutageOK == 0) {
      pas_dimmer = -10.0;
    } else if (ajustePuissance < 1000 && ajustePuissance >= 800 && puissanceRoutageOK == 0) {
      pas_dimmer = -6.0;
    } else if (ajustePuissance < 800 && ajustePuissance >= 400 && puissanceRoutageOK == 0) {
      pas_dimmer = -4.0;
    } else if (ajustePuissance < 400 && ajustePuissance >= 300 && puissanceRoutageOK == 0) {
      pas_dimmer = -3.0;
    } else if (ajustePuissance < 300 && ajustePuissance >= 200 && puissanceRoutageOK == 0) {
      pas_dimmer = -2.0;
    } else if (ajustePuissance < 200 && ajustePuissance >= 100 && puissanceRoutageOK == 0) {
      pas_dimmer = -1.0;
    } else if (ajustePuissance < 100 && ajustePuissance >= 50 && puissanceRoutageOK == 0) {
      pas_dimmer = -0.5;
    } else if (ajustePuissance < 50 && ajustePuissance >= 30 && puissanceRoutageOK == 0) {
      pas_dimmer = -0.5;
    } else if (ajustePuissance < 30 && ajustePuissance >= 1 && puissanceRoutageOK == 0) {
      pas_dimmer = -0.1;
    }


    Serial.print("pas_dimmer: ");
    Serial.print(pas_dimmer);

    // réglages Dimmer 1 ///
    if(valDim2 <= minDimmer) // seulement si Dimmer2 n'est pas en cours
      valDim1 = valDim1 + pas_dimmer;

    if (valDim1 <= minDimmer) {
      dimmer1.setState(OFF);
      dimmer1.setPower(minDimmer);
      valDim1 = minDimmer;
      delay(60);
    }

    else if (valDim1 >= maxDimmer) {
      dimmer1.setState(ON);
      dimmer1.setPower(maxDimmer);
      valDim1 = maxDimmer;
      delay(60);
    }

    else {
      dimmer1.setState(ON);
      dimmer1.setPower(valDim1);
      delay(60);
    }

    // réglages Dimmer 2 ///

    if (valDim1 >= maxDimmer) {

      valDim2 = valDim2 + pas_dimmer;

      if (valDim2 <= minDimmer) {
        dimmer2.setState(OFF);
        dimmer2.setPower(minDimmer);
        valDim2 = minDimmer;
        delay(60);
      }

      else if (valDim2 >= maxDimmer2) {
        dimmer2.setState(ON);
        dimmer2.setPower(maxDimmer2);
        valDim2 = maxDimmer2;
        delay(60);
      }
      else {
        dimmer2.setState(ON);
        dimmer2.setPower(valDim2);
        delay(60);
      }

    }

    else {
      valDim2 = minDimmer;
      dimmer2.setPower(valDim2);
      dimmer2.setState(OFF);
    }

    Serial.print(" / valDim1: ");
    Serial.print(valDim1);

    Serial.print(" / valDim2: ");
    Serial.println(valDim2);

    delay(500);
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

    if ((WiFi.status() != WL_CONNECTED) && (currentTimeTask2 - previousTimeWifi >= 60000)) {
      WiFi.disconnect();
      WiFi.reconnect();
      Serial.println("reconnecting wifi...");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(WiFi.localIP());
      }
      previousTimeWifi = currentTimeTask2;
    }

    ///////////////////////////////////////////////////////////////////
    ////////////// Fin reconnexion automatique Wifi ///////////////////
    ///////////////////////////////////////////////////////////////////


    // affichage page web DASH //
    consommationsurplus.update(-ajustePuissance);
    puissance.update(Power1);
    energy1.update(Energy1);
    energy2.update(Energy2);
    energy2C.update(Energy2C);
    valdim1.update(valDim1);
    valdim2.update(valDim2);
    Oled.update(oled);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    dashboard.sendUpdates();

    Oled.attachCallback([&](bool value) {
      oled = value;
      Oled.update(oled);
      dashboard.sendUpdates();
    });

    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////// affichage écran ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    u8g2.clearBuffer();  // on efface ce qui se trouve déjà dans le buffer
    if (oled == 1) {
      u8g2.setFont(u8g2_font_6x13_tf);
      u8g2.setCursor(0, 12);     // position du début du texte
      u8g2.print("Util: ");  // écriture puisance utilisée
      u8g2.print(Power1);
      u8g2.print(" W");
      u8g2.setCursor(0, 25);     // position du début du texte
      u8g2.print("Inject: ");  // écriture puisance reseau
      if(Sens2 == 0) {
        u8g2.print("-");
      }
      u8g2.print(Power2);
      u8g2.print(" W");

      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 34);
      u8g2.print("T1: ");
      u8g2.print((int)valDim1);
      u8g2.print("% T2: ");
      u8g2.print((int)valDim2);
      u8g2.print("%");   

      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 50);
      u8g2.print("IP: ");
      u8g2.print(WiFi.localIP());  // affichage adresse ip //
   
     

      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.setCursor(0, 64);
      u8g2.print("Energie sauvee : ");  // écriture de texte
      u8g2.print(Energy1), u8g2.print(" kWh");  // écriture de texte
      
      u8g2.sendBuffer();  // l'image qu'on vient de construire est affichée à l'écran
    }

    if (oled == 0) {
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    //////////////////////////////////////////////////////-//////////////////////////////
    ///////////////////////////// Fin affichage écran //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    delay(500); // un update 2 fois par seconde est suffisant
  }
}

void loop() {
}

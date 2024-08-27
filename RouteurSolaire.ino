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
int relayOn = 1000;                   // puissance du surplus pour déclencher le relay //
int relayOff = 800;                   // puissance du surplus pour stopper le relay //
boolean marcheForceeVol = 0;          // marche forcée automatique suivant le volume du ballon : 0 ou 1
int volume = 200;                     // volume du ballon en litres
byte HOn = 01;                        // heure début marche forcée
byte MnOn = 30;                       // minute début marche forcée
byte SecOn = 00;                      // sec début marche forcée

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Librairies //

#include <HardwareSerial.h>     // https://github.com/espressif/arduino-esp32
#include <RBDdimmer.h>          // gestion des Dimmers  https://github.com/RobotDynOfficial/RBDDimmer //
#include <U8g2lib.h>            // gestion affichage écran Oled  https://github.com/olikraus/U8g2_Arduino/ //
#include <Wire.h>               // pour esp-Dash
#include <WiFi.h>               // gestion du wifi
#include <ESPDash.h>            // page web Dash  https://github.com/ayushsharma82/ESP-DASH //
#include <AsyncTCP.h>           //  https://github.com/me-no-dev/AsyncTCP  ///
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer  et https://github.com/bblanchon/ArduinoJson
#include <NTPClient.h>          // gestion de l'heure https://github.com/arduino-libraries/NTPClient //
#include <ArduinoOTA.h>         // mise à jour OTA par wifi

// Coefficient pour simuler des charges reeles avec des petites charges
#define CoefSimulation 60.0

#ifndef CoefSimulation
#define CoefSimulation 1.0
#endif

WiFiUDP ntpUDP;
/*
* Choix du serveur NTP pour récupérer l'heure, 3600 =1h est le fuseau horaire et 60000=60s est le * taux de rafraichissement
*/
NTPClient temps(ntpUDP, "fr.pool.ntp.org", 3600, 60000);


#define RXD2 16
#define TXD2 17
#define Relay1 13  // relay on/off déclenchement LOW
#define Relay2 32  // relay 16A mini marche forcée déclenchement LOW



byte ByteArray[250];
int ByteData[20];

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);



//déclaration des variables//

float routagePuissance = -30; /* puissance d'injection réseau en watts */
int ajustePuissance = 0;      /* réglage puissance */
float puissanceRoutage = 0;
float puissanceRoutageOK;
float pas_dimmer;
float valDim1 = 0;
float valDim2 = 0;
float maxDimmer = 100;
float maxDimmer2 = 97;
float minDimmer = 0;
float Voltage, Intensite1, Energy1, Energy2C, Frequency, PowerFactor1, Intensite2, Energy2, Sens1, Sens2;
int Power1;                                            // puissance envoyée au ballon
int Power2;                                            // puissance entrant ou sortant de l'habitation
boolean Auto = 1;                                      // mise en route en automatique //
byte Value1;                                           // marche forcée Off//
byte Value2;                                           // marche forcée On//
float EnergyJ = 0;                                     // énergie sauvées le jour J et remise à zéro tous les jours //
float EnergyInit;                                      // énergie en début de journée //
boolean Start = 1;                                     // variable de démarrage du programme //
float energyNecessaireJ = 1.162 * 20 * volume / 1000;  // énergie nécessaire minimum par jour suivant le volume du ballon //
float energyComp;                                      // énergie marche forcée en complément
unsigned int TpsMarcheForcee;                          // temps de fonctionnement marche forcée automatique
byte HOffC;
byte MnOffC;
byte SecOffC;
byte HOff;         // heure fin marche forcée
byte MnOff;        // minute fin marche forcée
byte SecOff;       // sec fin marche forcée
char mn00Off[2];   // affichage des mns fin marche forcée au format 0
unsigned long currentTime = 0;
unsigned long previousTime1 = 0;  // temporisation relai sortie 3
unsigned long previousTime2 = 0;  // temporisation mqqt
unsigned long previousTime3 = 0;  // affichage alterné kWh J / kWh total
unsigned long previousTime4 = 0;  // demande de température au capteur
unsigned long previousTime5 = 0;  // variable temps pour reconnexion wifi
boolean oled = 1;                 // écran Oled allumé

///  configuration wifi ///

AsyncWebServer server(80);
WiFiClient espClient;
ESPDash dashboard(&server);
Card button(&dashboard, BUTTON_CARD, "Auto / Marche forcée");
Card horlogeRH(&dashboard, SLIDER_CARD, "Réglage horloge Heure de départ :", "h", 0, 23);
Card horlogeRM(&dashboard, SLIDER_CARD, "Réglage horloge Minute de départ :", "mn", 0, 59);
Card button3(&dashboard, BUTTON_CARD, "Horloge On/Off volume");
Card consommationsurplus(&dashboard, GENERIC_CARD, "Surplus (+=injection, -=conso)", "Watts");
Card puissance(&dashboard, GENERIC_CARD, "Consumation Ballon+Planchers", "Watts");
Card energy1(&dashboard, GENERIC_CARD, "énergie sauvée totale", "kwh");
Card energy2C(&dashboard, GENERIC_CARD, "Consommation Enedis totale", "kWh");
Card energyj(&dashboard, GENERIC_CARD, "énergie sauvée aujourd'hui", "kWh");
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
  Serial.println("Professeur Solaire Starting...");
  u8g2.begin();            // ECRAN OLED
  u8g2.enableUTF8Print();  //nécessaire pour écrire des caractères accentués
  dimmer1.begin(NORMAL_MODE, ON); /// Pourquoi y a pas le dimmer 2 ??????????????????????
  dimmer1.setPower(0);
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
    Serial.println("not connected");
  }
  server.begin();
  delay(100);
  temps.begin();    //Intialisation du client NTP
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
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



//// marche forcée suivant volume du ballon ///

void marcheForcee() {
  energyComp = (energyNecessaireJ * 1000) - (EnergyJ * 1000);
  TpsMarcheForcee = ((energyComp * 3600) / (volume * 10));  // temps marche forcée em sec suivant volume du ballon //
  energyComp = 0;

  HOffC = TpsMarcheForcee / 3600;
  MnOffC = TpsMarcheForcee / 60 - HOffC * 60;
  SecOffC = TpsMarcheForcee - HOffC * 3600 - MnOffC * 60;

  if ((MnOn + MnOffC) > 60) {
    MnOff = MnOn + MnOffC - 60;
    HOff = HOn + HOffC + 1;
  }
  if ((MnOn + MnOffC) < 60) {
    MnOff = MnOn + MnOffC;
    SecOff = SecOn + SecOffC;
  }
  if ((HOn + HOffC) > 23) {
    HOff = HOn + HOffC - 24;
  }

  if ((HOn + HOffC) < 24) {
    HOff = HOn + HOffC;
  }
  if ((SecOn + SecOffC) < 60) {
    SecOff = SecOn + SecOffC;
  }
  if ((SecOn + SecOffC) > 59) {
    SecOff = SecOn + SecOffC - 60;
  }

  Auto = 0;
}

///////////////////////////////////////////////

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
  Serial.println(Voltage);
  Serial.print("Intensite1: ");
  Serial.println(Intensite1);
  Serial.print("Power1: ");
  Serial.println(Power1);
  Serial.print("Energy1: ");
  Serial.println(Energy1);
  Serial.print("Sens1: ");
  Serial.println(Sens1);
  Serial.print("Sens2: ");
  Serial.println(Sens2);
  Serial.print("Frequency: ");
  Serial.println(Frequency);
  Serial.print("Intensite2: ");
  Serial.println(Intensite2);
  Serial.print("Power2: ");
  Serial.println(Power2);
  Serial.print("Energy2: ");
  Serial.println(Energy2);
  Serial.print("Energy2C: ");
  Serial.println(Energy2C);
  Serial.print("ajustePuissance: ");
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
  len = 0;

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
  ByteData[4] = ByteArray[15] * 16777216 + ByteArray[16] * 65536 + ByteArray[17] * 256 + ByteArray[18];   // Energie 1 en kwh surplus
  ByteData[7] = ByteArray[27];                                                                            // sens 1 du courant
  ByteData[9] = ByteArray[28];                                                                            // sens 2 du courant
  ByteData[8] = ByteArray[31] * 16777216 + ByteArray[32] * 65536 + ByteArray[33] * 256 + ByteArray[34];   // Fréquence en hz
  ByteData[10] = ByteArray[39] * 16777216 + ByteArray[40] * 65536 + ByteArray[41] * 256 + ByteArray[42];  // Intensité 2 en Ampères
  ByteData[11] = ByteArray[43] * 16777216 + ByteArray[44] * 65536 + ByteArray[45] * 256 + ByteArray[46];  // Puissance 2 en Watts
  ByteData[12] = ByteArray[47] * 16777216 + ByteArray[48] * 65536 + ByteArray[49] * 256 + ByteArray[50];  // Energie 2 en kwh sauvées
  ByteData[14] = ByteArray[55] * 16777216 + ByteArray[56] * 65536 + ByteArray[57] * 256 + ByteArray[58];  // Energie 2 en kwh consommation

  ////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////// Normalisation des valeurs ///////////////
  Voltage = ByteData[1] * 0.0001;      // Tension
  Intensite1 = ByteData[2] * 0.0001;   // Intensité 1
  Power1 = ByteData[3] * 0.0001 * CoefSimulation;       // Puissance 1
  Energy1 = ByteData[4] * 0.0001;      // Energie 1 surplus
  Sens1 = ByteData[7];                 // Sens 1
  Sens2 = ByteData[9];                 // Sens 2
  Frequency = ByteData[8] * 0.01;      // Fréquence
  Intensite2 = ByteData[10] * 0.0001;  // Intensité 2
  Power2 = ByteData[11] * 0.0001 * CoefSimulation;      // Puissance 2
  Energy2 = ByteData[12] * 0.0001;     // Energie 2
  Energy2C = ByteData[14] * 0.0001;    // Energie 2 consommation

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
  //PrintDatas();
}

//programme utilisant le Core 1 de l'ESP32//

void Task1code(void *pvParameters) {
  for (;;) {

    currentTime = millis();

    Auto = 1;

    if (Auto == 1) {
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


    }
    delay(2000);
  }
}



//programme utilisant le Core 2 de l'ESP32//

void Task2code(void *pvParameters) {

  for (;;) {

    ArduinoOTA.handle();

    //////////////////////////////////////////////////////////////////
    ///////////// Reconnexion wifi automatique ///////////////////////
    //////////////////////////////////////////////////////////////////

    if ((WiFi.status() != WL_CONNECTED) && (currentTime - previousTime5 >= 60000)) {
      WiFi.disconnect();
      WiFi.reconnect();
      Serial.println("reconnecting wifi...");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(WiFi.localIP());
      }
      previousTime5 = currentTime;
    }

    ///////////////////////////////////////////////////////////////////
    ////////////// Fin reconnexion automatique Wifi ///////////////////
    ///////////////////////////////////////////////////////////////////


    //// reboot ESP (EnergyJ) tous les jours à 04h30mn00 du matin ////

    if (temps.getHours() == 04 & temps.getMinutes() == 30 & temps.getSeconds() == 00) {
      delay(5000);
      Start = 1;
      //ESP.restart();
    }

    ///  initialisation énergie du jour ////

    if (Start == 1) {
      delay(100);
      Datas();
      EnergyInit = Energy1;
      Start = 0;
    }

    EnergyJ = Energy1 - EnergyInit;

    ///  affichage heure ///

    //Update de l'heure
    temps.update();

    sprintf(mn00Off, "%02d", MnOff);  //L'heure est envoyée sur le port serie au format 00:00:00 en 1 fois




    // affichage page web DASH //

    consommationsurplus.update(-ajustePuissance);
    puissance.update(Power1);
    energy1.update(Energy1);
    energy2C.update(Energy2);
    energyj.update(EnergyJ);
    valdim1.update(valDim1);
    valdim2.update(valDim2);
    button.update(Auto);
    button3.update(marcheForceeVol);
    horlogeRH.update(HOn);
    horlogeRM.update(MnOn);
    Oled.update(oled);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    dashboard.sendUpdates();


    //     u8g2.print("Fin : "),u8g2.print(HOff),u8g2.print("h");u8g2.print(mn00Off);


    // boutons page web //

    button.attachCallback([&](bool value) {
      Auto = value;
      button.update(Auto);
      dashboard.sendUpdates();
    });

    button3.attachCallback([&](bool value) {
      marcheForceeVol = value;
      button3.update(marcheForceeVol);
      dashboard.sendUpdates();
    });

    Oled.attachCallback([&](bool value) {
      oled = value;
      Oled.update(oled);
      dashboard.sendUpdates();
    });

    horlogeRH.attachCallback([&](int value) {
      HOn = value;
      horlogeRH.update(HOn);
      dashboard.sendUpdates();
    });

    horlogeRM.attachCallback([&](int value) {
      MnOn = value;
      horlogeRM.update(MnOn);
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
   
    
      /// alternance kwh sauvés par jour vs total /////

      if ((currentTime - previousTime3) < 2000) {
        u8g2.setFont(u8g2_font_4x6_tf);
        u8g2.setCursor(10, 64);
        u8g2.print("Sauvés J : ");  // écriture de texte
        u8g2.setCursor(55, 64);
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.print(EnergyJ), u8g2.print("kWh");  // écriture de texte
      }

      if ((currentTime - previousTime3) >= 2000 && (currentTime - previousTime3) < 5000) {
        u8g2.setFont(u8g2_font_4x6_tf);
        u8g2.setCursor(10, 64);
        u8g2.print("Sauvés T : ");  // écriture de texte
        u8g2.setCursor(55, 64);
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.print(Energy1), u8g2.print("kWh");  // écriture de texte
      }

      if ((currentTime - previousTime3) >= 4000 && (currentTime - previousTime3) < 86400000) { previousTime3 = currentTime; }

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

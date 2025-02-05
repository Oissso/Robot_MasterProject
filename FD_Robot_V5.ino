/*
  Démonstrateur de robot avec IA embarquée
  Mise en oeuvre apprentissage et détection d'anomalies
  Penser à mettre ACTIVATE_TOF à 0 en cas de comportement instable des TOF
  Attention : Version du programme pour le robot équippé de TOF à l'avant et à l'arrière
  Attention : Suppose que les messages BLE proviennent de l'application Android "BLE Simple Remote"
  Attention : Sur les cartes Nucleo WB55 distribuées depuis 2022, il est nécessaire de reprogrammer
  le firmware BLE HCI. Voir ce tutoriel : https://stm32python.gitlab.io/fr/docs/tools/cubeprog/cube_prog_firmware_ble_hci
*/

/* Définitions diverses */

// Révision du code
#define REV "0.06"

// Sélection des messages affichés et fonctions compilées
#define DEBUG 0 // Doit-on afficher les messages de debug (sur le port série du ST LINK) ? 
#define DISP_ACCELS 0 // Doit-on afficher les accélérations mesurées ?
#define DISP_CYCLES 0 // Doit-on afficher la durée d'un cycle du robot ?
#define DISP_CODERS 0 // Doit-on afficher les valeurs des roues codeuses / cycle ?
#define ACTIVATE_TOF 0 // Doit-on activer les modules ToF ?

// Débit du port série du ST-LINK
#define ST_LINK_BAUDRATE (115200)

// Nombre d'overflows du timer 16 constituant la base de temps
#define TIME_BASE (95) // Ajustée pour donner des cycles de 100 ms

// Valeur des consignes de vitesses pour les moteurs (nombre de ticks des fourches optiques)
#define V0 (0)
#define V1 (7)
#define V2 (12)
#define V3 (17)

// Consignes pour le régulateur PID
#define CKp (7.) // Coefficient proportionnel
#define CKi (3.) // Coefficient intégral
#define CKd (0.1) // Coefficient dérivé

/* Défintions pour les contrôleurs I2C */

#include <Wire.h> // Bibliothèque Arduino pour l'I2C
#define I2C3_SDA (PC1) // Soit A1 pour Arduino, broche SDA de l'I2C3
#define I2C3_SCL (PC0) // Soit A0 pour Arduino, broche SCL de l'I2C3

TwoWire Wire3(I2C3_SDA, I2C3_SCL); // Instanciation du contrôleur de bus I2C3

/* Définitions pour les GPIO expanders PCF8574 */

#include "PCF8574.h" // Bibliothèque disponible ici : https://github.com/RobTillaart/PCF8574
#define GPIOEXP_I2C_ADDR1 (0x38) // Adresse I2C pour PCF8574AP 1
#define GPIOEXP_I2C_ADDR2 (0x39) // Adresse I2C pour PCF8574AP 2
#define GPIOEXP_I2C_ADDR3 (0x3A) // Adresse I2C pour PCF8574AP 3

PCF8574 pcf8574_1(GPIOEXP_I2C_ADDR1, &Wire3); // Instanciation de PCF8574AP 1
PCF8574 pcf8574_2(GPIOEXP_I2C_ADDR2, &Wire3); // Instanciation de PCF8574AP 2
PCF8574 pcf8574_3(GPIOEXP_I2C_ADDR3, &Wire3); // Instanciation de PCF8574AP 3

/* Définitions pour le timer 16 : base de temps du robot */

// Base de temps du robot (fréquence interruption du timer 16)
#define TIME_BASE_HZ (1000)

// Instanciation de gestion d'interruption de dépassement du timer 16
TIM_TypeDef *Instance16 = TIM16;
HardwareTimer *Timer16 = new HardwareTimer(Instance16);

volatile uint8_t ControlCounter = 0; // Compteur pour les opérations de contrôle du robot

/* Définitions pour le BLE de la Nucleo WB55 */

#include <STM32duinoBLE.h> // Bibliothèque disponible ici : https://github.com/stm32duino/STM32duinoBLE

// Instanciation du BLE
HCISharedMemTransportClass HCISharedMemTransport;
BLELocalDevice BLEObj(&HCISharedMemTransport);
BLELocalDevice& BLE = BLEObj;

// Déclarations pour le service BLE Nordic UART et les caractéristiques Rx (réception) et Tx (émission)
#define BLE_UART_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define MAX_BLE_CHAR (15) // Taille du buffer de réception (en nombre de caractères)

BLEService remoteService(BLE_UART_UUID); // Crée le service avec le bon UUID
BLEStringCharacteristic rxCharacteristic(BLE_UART_RX_UUID, BLEWrite, MAX_BLE_CHAR); // Caractéristique Rx
BLEStringCharacteristic txCharacteristic(BLE_UART_TX_UUID, BLENotify, MAX_BLE_CHAR); // Caractéristique Tx

// Fréquence de polling des évènements BLE (fréquence interruption du timer 1)
#define BLE_POLLING_HZ (15)

// Instanciation de gestion de l'interruption de dépassement du timer 1
TIM_TypeDef *Instance1 = TIM1;
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

/* Défintions pour les fourches optiques des moteurs */

#define Fork_RF (PB7) // Right-Front : Moteur avant-droit
#define Fork_RR (PB12) // Right-Rear : Moteur arrière-droit
#define Fork_LF (PE4) // Left-Front : Moteur avant-gauche
#define Fork_LR (PB0) // Left-Rear : Moteur arrière-gauche

// Compteurs de fronts des fourches optiques
volatile uint16_t EdgeCountRightFront = 0;
volatile uint16_t EdgeCountRightRear = 0;
volatile uint16_t EdgeCountLeftFront = 0;
volatile uint16_t EdgeCountLeftRear = 0;

/* Défintions des broches pour le contrôle des moteurs (PWM et ponts en H) */

// Breakout L298 droite
#define IN1_RF (3)
#define IN2_RF (2)
#define ENA_RF (PB14) // PWM
#define IN3_RR (1)
#define IN4_RR (0)
#define ENB_RR (PB13) // PWM

// Breakout L298 gauche
#define IN3_LF (5)
#define IN4_LF (4)
#define ENB_LF (PB11) // PWM
#define IN1_LR (7)
#define IN2_LR (6)
#define ENA_LR (PB15) // PWM

// Broches de contrôle du pare-choc avant
#define BMPR (PB4) // (CN10 4)
#define BMPL (PC5) // (CN7 3)

/* Définitions pour les capteurs de temps de vol (ToF) */
#include <VL53L0X.h>  // Bibliothèque disponible ici : https://github.com/pololu/vl53l0x-arduino

// Broches et lignes de contrôle (XSHUT) des ToF
#define XTOF1 (5) // Capteur en bas à droite (robot vu de l'arrière)
#define XTOF2 (4) // Capteur en bas à gauche
#define XTOF3 (7) // Capteur en haut à droite
#define XTOF4 (6) // Capteur en haut à gauche

// Adresses I2C des ToF
#define TOF1_ADDR 0x17
#define TOF2_ADDR 0x18
#define TOF3_ADDR 0x19
#define TOF4_ADDR 0x20

// Instanciation des ToF
VL53L0X ToF1;
VL53L0X ToF2;
VL53L0X ToF3;
VL53L0X ToF4;

// Seuils de détection (en mm) pour l'anti-collision
#define THR_DISTANCE (200)
#define THR_DEPTH (55)

/* Définitions pour les commandes et la cinématique du robot */

// Enumération des commandes gérées
enum COMMAND {
  cmdSTART, // Démarrer
  cmdSTOP, // Arrêter
  cmdFORWARD, // Avancer
  cmdBACKWARD, // Reculer
  cmdRIGHT, // Tourner à droite
  cmdLEFT, // Tourner à gauche
  cmdLEARNING // Lancer l'apprentissage NEAI
};

// On commence avec la commande Stop
enum COMMAND Cmd = cmdSTOP;

// Est-ce que j'ai une commande à traiter ? (oui)
uint8_t Process_CMD = 1;

// Enumération des modes de fonctionnement du robot
enum MODE {
  SLEEPING, // Robot immobile
  ACTIVE // Robot en train de rouler
};

// On commence en mode Sleeping
enum MODE Mode = SLEEPING;

// Enumération des consignes sur le sens de rotation des moteurs
enum MOVEMENT {
  STOPPED, // Moteur arrêté, rotation libre
  FORWARD, // En avant
  BACKWARD, // En arrière
  RIGHT_SLIDE, // Translation à doite
  LEFT_SLIDE // Translation à gauche
};

// Consignes initiales de sens de rotation des moteurs (en provenance de ProcessCommand)
enum MOVEMENT _RightMotorDir = STOPPED;
enum MOVEMENT _LeftMotorDir = STOPPED;

// Consignes corrigées de sens de rotation des moteurs (après appel à Anti_Collisions_System)
enum MOVEMENT RightMotorDir;
enum MOVEMENT LeftMotorDir;

// Valeurs mini et maxi des PWM pour les moteurs
#define PWM_MIN (0)
#define PWM_MAX (168) //255

// Consignes de vitesses initiales (en provenance de ProcessCommand)
uint16_t __SpeedRight = V0; // Consigne de vitesse pour les deux moteurs côté droit
uint16_t __SpeedLeft = V0; // Consigne de vitesse pour les deux moteurs côté gauche

// Consignes de vitesses corrigées (après appel à Anti_Collisions_System)
uint16_t _SpeedRight;
uint16_t _SpeedLeft;

// Vitesses effectives (après régulation PID)
uint16_t SpeedRightFront; // Vitesse moteur avant droit
uint16_t SpeedRightRear; // Vitesse moteur arrière droit
uint16_t SpeedLeftFront; // Vitesse moteur avant gauche
uint16_t SpeedLeftRear; // Vitesse moteur arrière gauche

// Durée d'immobilité maximum, en nombre d'overflow du timer 16
#define TIME_MOTIONLESS 1000
volatile uint32_t MotionlessCounter = 0; // Compteur du temps passé immobile

// Pilote et instanciation de l'accéléromètre
#include <LIS2DW12Sensor.h> // Bibliothèque disponible ici : https://github.com/stm32duino/LIS2DW12
LIS2DW12Sensor Acc(&Wire);

// Fréquence d'échantillonnage de l'accéléromètre (également  fréquence interruption du timer 17)
#define ACC_FREQ_HZ 400

// Instanciation de gestion de l'interruption de dépassement du timer 17
TIM_TypeDef *Instance17 = TIM17;
HardwareTimer *Timer17 = new HardwareTimer(Instance17);
volatile uint8_t TriggerRecord = 0; // Signale qu'il est temps d'enregistrer une mesure d'accélération

// Déclarations pour la détection d'anomalies NanoEdge AI
#include "NanoEdgeAI.h" // Bibliothèque générée avec NanoEdgeAI Studio
uint8_t similarity = 0; // Similarité entre vibrations courantes et vibrations apprises (0 à 100)

#define AXIS_NUMBER 3 // On souhaite enregistrer des triplets d'accélération (ax, ay, az) 
#define SAMPLE_RECORDS_COUNT 128 // Nombre de triplets par ligne d'enregistrement (doit être le même que lors de la génération de la lib NEAI)
#define LEARNING_ROUNDS 10 // Nombre de rounds d'apprentissage
#define SIMILARITY_THRESHOLD 90 // Seuil de détection sur la "similarité" évaluée par NanoEdge AI

int32_t acc_value[3];
float acc_buffer[SAMPLE_RECORDS_COUNT * AXIS_NUMBER] = { 0 };

uint32_t i_record = 0; // Nombre de valeurs ax, ay et az enregistrées
uint8_t learning_completed = 0; // Est-ce que l'étape d'apprentissage a déjà été réalisée ?

// Configuration du module Bluetooth HC06
// RX du HC06 sur D1 (TX de la NUCLEO-WB55)
// TX du HC06 sur D0 (RX de la NUCLEO-WB55)
#define RX_WB55 (D0)
#define TX_WB55 (D1)
#define HC06_BAUDRATE (9600)

HardwareSerial HC06(RX_WB55, TX_WB55);

/* Déclaration pour le ruban Neopixel */
#include <Adafruit_NeoPixel.h> // bibliothèque disponible ici : https://github.com/adafruit/Adafruit_NeoPixel
#define StickLED (PC12) // Broche de contôle de la barette (D8)
#define NbLED (8) // Nombre de LED de la barette
Adafruit_NeoPixel LightBar(NbLED, StickLED, NEO_GRB); // Instanciation de la barette

// Enumération des couleurs
enum COLOR {
  cRED,
  cGREEN,
  cBLUE
};

#if DISP_CYCLES == 1
unsigned long StartTime;
unsigned long CurrentTime;
#endif

/* ----------------------------------- Initialisations ----------------------------------- */
void setup() {

  // Initialisation du port série du ST-LINK
  Serial.begin(ST_LINK_BAUDRATE);
  while (!Serial);

  // Affichage de la révision du code
  Serial.print("Revision du code : ");
  Serial.println(REV);

  // Initialisation du module bluetooth HC06 (COM8)
  HC06.begin(HC06_BAUDRATE);
  while (!HC06);
  Serial.println("Module bluetooth HC06 activé");

  // Initialisation du bus I2C1
  init_I2C1();

  // Initialisation du bus I2C3
  init_I2C3();

  // Initialisation des GPIO expanders
  init_GPIO_Expanders();

  // Initialisation du BLE
  init_BLE();

  // Initialisation de l'interruption périodique du timer 1
  // Pour la gestion du polling BLE
  init_Timer_BLE();

  // Initialisation du pare-chocs
  init_Bumper();

  // Initialisation des des moteurs
  init_Motors();

  // Initialisation de l'interruption périodique du timer 16
  // Pour la créer la base de temps du robot
  init_Timer_TimeBase();

  // Initialisation de l'interruption périodique du timer 17
  // Pour gérer l'enregistrement des mesures d'accélérations
  init_Timer_Accel();

  // Initialisation et démarrage de l'accéléromètre
  init_Accel();

  // Initialisation de NanoEdge AI
  init_NEAI();

  // Initialisation de la barette NeoPixel
  init_LightBar();

  // Initialisation des capteurs de distance (ToF)
#if ACTIVATE_TOF == 1
  init_ToF();
#endif

  // Liste des adresses active sur le bus I2C3
  I2C3_scan();

  // Signale que l'initialisation s'est bien déroulée
  LightBar_Blocking_Signal(1000, cGREEN); // Signal lumineux vert pendant 1000 ms
  Serial.println("Initialisations terminées !");

#if DISP_CYCLES == 1
  StartTime = 0;
  CurrentTime = 0;
#endif

}

/* ----------------------------------- Programme principal ----------------------------------- */

void loop() {

  // Exécution des commandes reçues
  ProcessCommand();

  // Détection d'anomalies NEAI
  FaultDetection();

  // Opérations de contrôle du robot (toutes les TIME_BASE interruptions du timer 16)
  if (ControlCounter > TIME_BASE) {

#if DISP_CYCLES == 1
    CurrentTime = millis();
    Serial.println(CurrentTime - StartTime);
    StartTime = CurrentTime;
#endif

    // Système anti-collisions
    Anti_Collisions_System();

    // Calcul de la vitesse
    Compute_Speed();

    // Régulation PID
    PID_Regulator();

    // On éteint la barre de LED
    LightBar_Reset();

    ControlCounter = 0;
  }

}

/* ---------------------Initialisations ----------------------- */


/****************************************************************/
/* Gestions des deux bus I2C                              */
/****************************************************************/

void init_I2C1(void)
{
  Wire.begin();
  delay(10);
  Serial.println("Bus I2C1 initialisé");
}

void init_I2C3(void)
{
  Wire3.begin();
  delay(10);
  Serial.println("Bus I2C3 initialisé");
}



/************************************************************************/
/*  Scan du bus I2C3                                                    */
/************************************************************************/

void I2C3_scan(void) {
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scan du bus I2C3 en cours ...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {

    Wire3.beginTransmission(address);
    error = Wire3.endTransmission();

    if (error == 0)
    {
      Serial.print("Objet I2C trouvé à l'adresse 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    /*
        else if (error == 4)
        {
          Serial.print("Erreur inconnue à l'adresse 0x");
          if (address < 16)
            Serial.print("0");
          Serial.println(address, HEX);

        }
    */
  }
  if (nDevices == 0)
  {
    Serial.println("Aucun objet I2C3 n'a été trouvé");
    while (1);
  }
  else
    Serial.println("Scan du bus I2C3 terminé");
}

/****************************************************************/
/* Initialisation et gestion de la barette NeoPixel             */
/****************************************************************/
void init_LightBar(void)
{
  LightBar.begin(); // Initialisation de la barre de LED
  LightBar.setBrightness(50);  // On fixe l'intensité des LED à 20% (max = 255)
  Serial.println("Barette NeoPixel activée");
}

/* Signal utilisant delay (bloquant) */
void LightBar_Blocking_Signal(uint32_t blocking_delay_ms, enum COLOR clr)
{

  // Fixe la couleur de toutes les LED à COLOR

  if (clr == cRED)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(150, 0, 0));
  }
  else if (clr == cGREEN)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(0, 150, 0));
  }
  else if (clr == cBLUE)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(0, 0, 150));
  }

  LightBar.show(); // On met à jour l'affichage

  delay(blocking_delay_ms); // On patiente blocking_delay_ms millisecondes

  LightBar.clear(); // On efface les consignes de couleurs de toutes les LED
  LightBar.show(); // On met à jour l'affichage

}

/* Lance un signal (non bloquant) */
void LightBar_Signal(enum COLOR clr)
{
  // Fixe la couleur de toutes les LED à COLOR

  if (clr == cRED)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(150, 0, 0));
  }
  else if (clr == cGREEN)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(0, 150, 0));
  }
  else if (clr == cBLUE)
  {
    for (uint8_t i = 0; i < NbLED; i++)
      LightBar.setPixelColor(i, LightBar.Color(0, 0, 150));
  }

  LightBar.show(); // On met à jour l'affichage

}

/* Eteint la barre */
void LightBar_Reset(void)
{

  LightBar.clear(); // On efface les consignes de couleurs de toutes les LED
  LightBar.show(); // On met à jour l'affichage
}

/****************************************************************/
/* Initialisation de l'accéléromètre                            */
/****************************************************************/
void init_Accel(void)
{
  Acc.begin();
  Acc.Set_X_ODR(ACC_FREQ_HZ);
  Acc.Enable_X();
  Serial.println("Accéléromètre activé");
}

/****************************************************************/
/* Initialisation des GPIO expanders                            */
/****************************************************************/
void init_GPIO_Expanders(void)
{

  Serial.print("Initialisation pcf8574 AP 1 ");
  if (pcf8574_1.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("échec !");
    while (1);
  }

  Serial.print("Initialisation pcf8574 AP 2 ");
  if (pcf8574_2.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("échec !");
    while (1);
  }

  Serial.print("Initialisation pcf8574 AP 3 ");
  if (pcf8574_3.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("échec !");
    while (1);
  }



}

/****************************************************************/
/* Initialisation des ToF                                       */
/****************************************************************/
void init_ToF(void)
{

  Serial.println("Setup des capteurs ToF démarré");

  // On initialise  tous les capteurs de distance
  pcf8574_2.write(XTOF1, LOW);
  pcf8574_2.write(XTOF2, LOW);
  pcf8574_2.write(XTOF3, LOW);
  pcf8574_2.write(XTOF4, LOW);
  delay(10);

  Serial.print("Setup de ToF1");
  pcf8574_2.write(XTOF1, HIGH); // On allume ToF1
  delay(10);
  ToF1.setBus(&Wire3); // Capteur sur le bus I2C3
  ToF1.init(true); // On initialise ToF1
  ToF1.setAddress((uint8_t)TOF1_ADDR); // On attribue à ToF1 l'adresse TOF1_ADDR (par défaut son adresse est 0x29)
  Serial.println(" OK");
  delay(10);

  Serial.print("Setup de ToF2");
  pcf8574_2.write(XTOF2, HIGH); // On allume ToF2
  delay(10);
  ToF2.setBus(&Wire3); // Capteur sur le bus I2C3
  ToF2.init(true); // On initialise ToF2
  ToF2.setAddress((uint8_t)TOF2_ADDR); // On attribue à ToF3 l'adresse TOF2_ADDR (par défaut son adresse est 0x29)
  Serial.println(" OK");
  delay(10);


  Serial.print("Setup de ToF3");
  pcf8574_2.write(XTOF3, HIGH); // On allume ToF3
  delay(10);
  ToF3.setBus(&Wire3); // Capteur sur le bus I2C3
  ToF3.init(true); // On initialise ToF3
  ToF3.setAddress((uint8_t)TOF3_ADDR); // On attribue à ToF3 l'adresse TOF3_ADDR (par défaut son adresse est 0x29)
  Serial.println(" OK");
  delay(10);


  Serial.print("Setup de ToF4");
  pcf8574_2.write(XTOF4, HIGH); // On allume ToF4
  delay(10);
  ToF4.setBus(&Wire3); // Capteur sur le bus I2C3
  ToF4.init(true); // On initialise ToF4
  ToF4.setAddress((uint8_t)TOF4_ADDR); // On attribue à ToF4 l'adresse TOF4_ADDR (par défaut son adresse est 0x29)
  Serial.println(" OK");
  delay(10);


  //ToF1.startContinuous();
  //ToF2.startContinuous();
  ToF3.startContinuous();
  ToF4.startContinuous();

  Serial.println("Setup des capteurs ToF terminé");

}

/****************************************************************/
/* Démarrage de l'interruption du timer 1 (polling BLE)         */
/****************************************************************/
void init_Timer_BLE(void)
{
  Timer1->setOverflow(BLE_POLLING_HZ, HERTZ_FORMAT);
  Timer1->attachInterrupt(Timer1_ISR);
  Timer1->resume();
  Serial.println("IT timer 1 active (polling BLE)");

}

/* Fonction de service de l'interruption de dépassement du timer 1 */

void Timer1_ISR(void)
{
  BLE.poll();
}


/****************************************************************/
/* Démarrage de l'interruption du timer 16 (base de temps)      */
/****************************************************************/
void init_Timer_TimeBase(void)
{
  Timer16->setOverflow(TIME_BASE_HZ, HERTZ_FORMAT);
  Timer16->attachInterrupt(Timer16_ISR);
  Timer16->resume();
  Serial.println("IT timer 16 active (base de temps)");

  // Démarre le robot (et envoie la commande START)
  Cmd = cmdSTART;
  Process_CMD = 1;

}

/* Fonction de service de l'interruption de dépassement du timer 16 */

void Timer16_ISR(void)
{
  ControlCounter++;
  MotionlessCounter++;
}

/****************************************************************/
/* Démarrage de l'interruption du timer 17 (accéléromètre)      */
/****************************************************************/
void init_Timer_Accel(void)
{
  Timer17->setOverflow(TIME_BASE_HZ, HERTZ_FORMAT);
  Timer17->attachInterrupt(Timer17_ISR);
  Timer17->resume();
  Serial.println("IT timer 17 active (mesures d'accélérations)");
}

/* Fonction de service de l'interruption de dépassement du timer 17 */

void Timer17_ISR(void)
{
  TriggerRecord = 1;
}

/****************************************************************/
/* Initialisation du BLE                                        */
/****************************************************************/

void init_BLE(void) {

  if (!BLE.begin()) {
    Serial.println("Echec de démarrage du BLE !");
    LightBar_Blocking_Signal(1000, cRED);
    while (1);
  }

  // Création d'un identifiant unique pour le robot
  String device_address = String(BLE.address());

  // Enlève les ":" de l'adresse MAC virtuelle
  device_address.remove(2, 1);
  device_address.remove(4, 1);
  device_address.remove(6, 1);
  device_address.remove(8, 1);
  device_address.remove(10, 1);

  device_address = "Robot_" + device_address;
  char buf[24];
  device_address.toCharArray(buf, 24);

  // Nom du service exposé par l'advertising
  BLE.setLocalName(buf);

  // Fixe l'UUID du service de réception
  BLE.setAdvertisedService(remoteService);

  // Ajoute la caractéristique de réception Rx
  // Les messages reçus du central connecté par la Nucleo-WB55 seront écrits dans celle-ci.
  remoteService.addCharacteristic(rxCharacteristic);

  // Ajoute la caractéristique d'émission Tx
  // Pour envoyer un message à une central connecté, la Nucleo-WB55 devra écrire dans celle-ci.
  remoteService.addCharacteristic(txCharacteristic);

  // Ajoute le service de réception
  BLE.addService(remoteService);

  // Gestionnaire des évènements central connecté ou déconnecté au périphérique (le robot)
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Gestionnaire des évènements d'écriture par le central dans la caractéristique Rx
  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  // Valeur initiale des charactéristiques
  rxCharacteristic.setValue("");
  txCharacteristic.setValue("");

  // Démarre la publication du service et des caractéristiques
  BLE.advertise();

  Serial.println("BLE en attente de connexion");
}

/* Gestionnaire de l'évènement de connexion au central */
void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connecté au central ");
  Serial.println(central.address());
  LightBar_Blocking_Signal(100, cBLUE); // Signal lumineux bleu pendant 100 ms
}

/* Gestionnaire de l'évènement de déconnexion du central */
void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.println("Déconnecté du central");
  LightBar_Blocking_Signal(100, cBLUE); // Signal lumineux bleu pendant 100 ms
}

/* Gestionnaire de l'évènement d'écriture par le central dans la caractéristique de réception Rx */
void rxCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  String BLE_RX = rxCharacteristic.value();

  // Suppose que le message reçu provient de l'application Android BLE Simple Remote
  // Le traduit en une commande reconnue par robot, envoie cette commande

  char Channel = BLE_RX[0] ;

  if (Channel == '0') // Si canal 0
  {

    // Channel 0, en avant : "0:0,80,0"
    if (BLE_RX[4] == '8')
    {
      Serial.println("Commande : FORWARD");
      Cmd = cmdFORWARD;
      Process_CMD = 1;
      return;
    }

    // Channel 0, en arrière : "0:0,-80,0"
    if (BLE_RX[5] == '8')
    {
      Serial.println("Commande : BACKWARD");
      Cmd = cmdBACKWARD;
      Process_CMD = 1;
      return;
    }

    // Channel 0, à droite : "0:80,0,0"
    if (BLE_RX[2] == '8')
    {
      Serial.println("Commande : TURN RIGHT");
      Cmd = cmdRIGHT;
      Process_CMD = 1;
      return;
    }

    // Channel 0, à gauche : "0:-80,0,0"
    if (BLE_RX[3] == '8')
    {
      Serial.println("Commande : TURN LEFT");
      Cmd = cmdLEFT;
      Process_CMD = 1;
      return;
    }

    // Channel 0, stop : "0:0,0,0"
    if (BLE_RX[2] == '0' && BLE_RX[4] == '0' && BLE_RX[6] == '0')
    {
      Serial.println("Commande : START");
      Cmd = cmdSTART;
      Process_CMD = 1;
      return;
    }

  } // fin si canal 0

  else if (Channel  == '1')  // Si canal 1
  {
    // Phase d'apprentissage
    Serial.println("Commande : LEARNING");
    Cmd = cmdLEARNING;
    Process_CMD = 1;
    return;

  } // fin si canal 1

}


/****************************************************************/
/* Initialisation du pare-chocs avant                           */
/****************************************************************/

void init_Bumper(void)
{
  // Initialisation des broches pour le pare-chocs
  pinMode(BMPR, INPUT_PULLUP);
  pinMode(BMPL, INPUT_PULLUP);

  // On attache les routines de service des interruptions du pare-chocs
  attachInterrupt(digitalPinToInterrupt(BMPR), Bumper_ISR, FALLING);    // Interruption sur contact bumper à droite
  attachInterrupt(digitalPinToInterrupt(BMPL), Bumper_ISR, FALLING);    // Interruption sur contact bumper à gauche

  Serial.println("Pare-chocs actif");
}

/* Routine de service des interruptions du pare-chocs */
void Bumper_ISR(void)
{
  // On arrête les moteurs immédiatement et on lance la commande STOP
  Motors_Stopped();
  Cmd = cmdSTOP;
  Process_CMD = 1;
}

/*----------------- Pilotage des moteurs ----------------------- */

/****************************************************************/
/* Initialisation des fourches optiques des moteurs             */
/****************************************************************/
void init_Motors(void)
{
  // Au départ, les moteurs sont arrêtés
  Motors_Stopped();

  // Configuration des interruptions externes des fourches optiques des moteurs (fronts montants et descendants)
  attachInterrupt(digitalPinToInterrupt(Fork_RF), Edges_Count_RF_ISR, CHANGE); // Fourche avant-droite
  attachInterrupt(digitalPinToInterrupt(Fork_RR), Edges_Count_RR_ISR, CHANGE); // Fourche arrière-droite
  attachInterrupt(digitalPinToInterrupt(Fork_LF), Edges_Count_LF_ISR, CHANGE); // Fourche avant-gauche
  attachInterrupt(digitalPinToInterrupt(Fork_LR), Edges_Count_LR_ISR, CHANGE); // Fourche arrière-gauche

  Serial.println("Fourches optiques actives");
}

/* Fonctions de service des interruptions externes des fourches optiques */

// On incrémente le compteur de fronts du codeur avant-droit
void Edges_Count_RF_ISR() {
  EdgeCountRightFront++;
}

// On incrémente le compteur de fronts du codeur arrière-droit
void Edges_Count_RR_ISR() {
  EdgeCountRightRear++;
}

// On incrémente le compteur de fronts du codeur avant-gauche
void Edges_Count_LF_ISR() {
  EdgeCountLeftFront++;
}

// On incrémente le compteur de fronts du codeur arrière-gauche
void Edges_Count_LR_ISR() {
  EdgeCountLeftRear++;
}

/****************************************************************/
/* Arrêt simultané des quatre moteurs                           */
/****************************************************************/
void Motors_Stopped(void)
{
  Motor_RF_Stopped();
  Motor_LF_Stopped();
  Motor_RR_Stopped();
  Motor_LR_Stopped();
}

/****************************************************************************/
/* Démarrage simultané des quatre moteurs en marche avant                   */
/* Paramètre d'entrée : Duty, un entier entre 0 et 255 proportionnel au %   */
/* de la période dans l'état up (rapport cyclique) de la PWM.               */
/****************************************************************************/
void Motors_Forward(uint8_t Duty)
{
  Motor_RF_Forward(Duty);
  Motor_LF_Forward(Duty);
  Motor_RR_Forward(Duty);
  Motor_LR_Forward(Duty);
}

/****************************************************************************/
/* Pilotes des moteurs                                                      */
/****************************************************************************/

/* Moteur avant-droit arrêté */
void Motor_RF_Stopped(void) {
  pcf8574_1.write(IN1_RF, LOW);
  pcf8574_1.write(IN2_RF, LOW);
  digitalWrite(ENA_RF, LOW);
}

/* Moteur avant-gauche arrêté */
void Motor_LF_Stopped(void) {
  pcf8574_1.write(IN4_LF, LOW);
  pcf8574_1.write(IN3_LF, LOW);
  digitalWrite(ENB_LF, LOW);
}

/* Moteur arrière-droit arrêté */
void Motor_RR_Stopped(void) {
  pcf8574_1.write(IN4_RR, LOW);
  pcf8574_1.write(IN3_RR, LOW);
  digitalWrite(ENB_RR, LOW);
}

/* Moteur arrière-gauche arrêté */
void Motor_LR_Stopped(void) {
  pcf8574_1.write(IN1_LR, LOW);
  pcf8574_1.write(IN2_LR, LOW);
  digitalWrite(ENA_LR, LOW);
}

/* Moteur avant-droit avance avec un rapport cyclique "Duty" */
void Motor_RF_Forward(uint8_t Duty) {
  pcf8574_1.write(IN1_RF, LOW);
  pcf8574_1.write(IN2_RF, HIGH);
  analogWrite(ENA_RF, Duty);
}

/* Moteur avant-gauche avance avec un rapport cyclique "Duty" */
void Motor_LF_Forward(uint8_t Duty) {
  pcf8574_1.write(IN4_LF, LOW);
  pcf8574_1.write(IN3_LF, HIGH);
  analogWrite(ENB_LF, Duty);
}

/* Moteur arrière-droit avance avec un rapport cyclique "Duty" */
void Motor_RR_Forward(uint8_t Duty) {
  pcf8574_1.write(IN4_RR, LOW);
  pcf8574_1.write(IN3_RR, HIGH);
  analogWrite(ENB_RR, Duty);
}

/* Moteur arrière-gauche avance avec un rapport cyclique "Duty" */
void Motor_LR_Forward(uint8_t Duty) {
  pcf8574_1.write(IN1_LR, LOW);
  pcf8574_1.write(IN2_LR, HIGH);
  analogWrite(ENA_LR, Duty);
}

/* Moteur avant-droit recule avec un rapport cyclique "Duty" */
void Motor_RF_Backward(uint8_t Duty) {
  pcf8574_1.write(IN1_RF, HIGH);
  pcf8574_1.write(IN2_RF, LOW);
  analogWrite(ENA_RF, Duty);
}

/* Moteur avant-gauche recule avec un rapport cyclique "Duty" */
void Motor_LF_Backward(uint8_t Duty) {
  pcf8574_1.write(IN4_LF, HIGH);
  pcf8574_1.write(IN3_LF, LOW);
  analogWrite(ENB_LF, Duty);
}

/* Moteur arrière-droit recule avec un rapport cyclique "Duty" */
void Motor_RR_Backward(uint8_t Duty) {
  pcf8574_1.write(IN4_RR, HIGH);
  pcf8574_1.write(IN3_RR, LOW);
  analogWrite(ENB_RR, Duty);
}

/* Moteur arrière-gauche recule avec un rapport cyclique "Duty" */
void Motor_LR_Backward(uint8_t Duty) {
  pcf8574_1.write(IN1_LR, HIGH);
  pcf8574_1.write(IN2_LR, LOW);
  analogWrite(ENA_LR, Duty);
}

/************************************************************************/
/* Machine d'états pour le traitement des commandes                     */
/************************************************************************/
void ProcessCommand(void) {
  // Si aucune commande reçue, quitte la fonction
  if (!Process_CMD) return;

  // Enumération des états possibles
  enum STATE {
    Standby, // Veille
    StopMotor, // Arrêt
    ForwardSpeed1, // Avancer, vitesse 1
    ForwardSpeed2, // Avancer, vitesse 2
    ForwardSpeed3, // Avancer, vitesse 3
    BackwardSpeed1, // Reculer, vitesse 1
    BackwardSpeed2, // Reculer, vitesse 2
    BackwardSpeed3, // Reculer, vitesse 3
    RightSpeed1, // A droite, vitesse 1
    RightSpeed2, // A droite, vitesse 2
    RightSpeed3, // A droite, vitesse 3
    LeftSpeed1, // A gauche, vitesse 1
    LeftSpeed2, // A gauche, vitesse 2
    LeftSpeed3 // A gauche, vitesse 3
  };

  static enum STATE State = Standby;

  switch (Cmd) { // Commande reçue

    case cmdSTOP: { // Commande "Arrêter" reçue
        Serial.println("Next state : Standby, Mode : SLEEPING");
        _RightMotorDir = STOPPED;
        _LeftMotorDir = STOPPED;
        __SpeedRight = V0;
        __SpeedLeft = V0;
        State = Standby;  // Prochain état
        Mode = SLEEPING;
        // Si une commande est reçue, réarme la réception
        // Ceci permet aussi de n'exécuter la commande qu'une seule fois
        Process_CMD  = 0;
        break;
      }

    case cmdSTART: { // Commande "Démarrer" reçue
        Serial.println("Next state : StopMotor, Mode : SLEEPING");
        State = StopMotor;  // Prochain état
        _RightMotorDir = STOPPED;
        _LeftMotorDir = STOPPED;
        __SpeedRight = V0;
        __SpeedLeft = V0;
        Mode = SLEEPING;
        Process_CMD  = 0;
        break;
      }

    case cmdFORWARD: { // Commande "Avancer tout droit" reçue

        switch (State) { // Début Switch State

          // Si l'état est "STANDBY" (donc, si on n'a pas reçu la commande "Démarrer")
          // reste dans létat "STANDBY".
          case Standby: {
              Serial.println("Next state : Standby, Mode : SLEEPING");
              State = Standby; // Prochain état
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }

          // Si l'état est "StopMotor " (éveillé mais immobile après réception de la commande "Démarrer")
          // alors le prochain état sera "ForwardSpeed1".
          case StopMotor: {
              Serial.println("Next state : ForwardSpeed1, Mode : ACTIVE");
              State = ForwardSpeed1; // Prochain état
              _RightMotorDir = FORWARD; // Consigne sens de rotation prochain état
              _LeftMotorDir = FORWARD; // Consigne sens de rotation prochain état
              __SpeedRight = V1; // Consigne vitesse prochain état
              __SpeedLeft = V1; // Consigne vitesse prochain état
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "ForwardSpeed1" alors le prochain état sera "ForwardSpeed2".
          case ForwardSpeed1: {
              Serial.println("Next state : ForwardSpeed2, Mode : ACTIVE");
              State = ForwardSpeed2; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "ForwardSpeed2" alors le prochain état sera "ForwardSpeed3".
          case ForwardSpeed2: {
              Serial.println("Next state : ForwardSpeed3, Mode : ACTIVE");
              State = ForwardSpeed3; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "ForwardSpeed3" alors le prochain état sera "ForwardSpeed3".
          // Pas de changement ; pas de vitesse au-delà de V3.
          case ForwardSpeed3: {
              Serial.println("Next state : ForwardSpeed3, Mode : ACTIVE");
              State = ForwardSpeed3; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "BackwardSpeed1" alors le prochain état sera "StopMotor".
          // (Un robot qui recule et reçoit la commande avancer réduit sa vitesse d'un rang)
          case BackwardSpeed1: {
              Serial.println("Next state : StopMotor, Mode : SLEEPING");
              State = StopMotor; // Prochain état
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }

          // Si l'état est "BackwardSpeed2" alors le prochain état sera "BackwardSpeed1".
          case BackwardSpeed2: {
              Serial.println("Next state : BackwardSpeed1, Mode : ACTIVE");
              State = BackwardSpeed1; // Prochain état
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "BackwardSpeed3" alors le prochain état sera "BackwardSpeed2".
          case BackwardSpeed3: {
              Serial.println("Next state : BackwardSpeed2, Mode : ACTIVE");
              State = BackwardSpeed2; // Prochain état
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "RightSpeed1" alors le prochain état sera "ForwardSpeed1"
          case RightSpeed1: {
              Serial.println("Next state : ForwardSpeed1, Mode : ACTIVE");
              State = ForwardSpeed1; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "RightSpeed2" alors le prochain état sera "ForwardSpeed2"
          case RightSpeed2: {
              Serial.println("Next state : ForwardSpeed2, Mode : ACTIVE");
              State = ForwardSpeed2; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "RightSpeed3" alors le prochain état sera "ForwardSpeed3"
          case RightSpeed3: {
              Serial.println("Next state : ForwardSpeed3, Mode : ACTIVE");
              State = ForwardSpeed3; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "LeftSpeed1" alors le prochain état sera "ForwardSpeed1"
          case LeftSpeed1: {
              Serial.println("Next state : ForwardSpeed1, Mode : ACTIVE");
              State = ForwardSpeed1; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "LeftSpeed2" alors le prochain état sera "ForwardSpeed2"
          case LeftSpeed2: {
              Serial.println("Next state : ForwardSpeed2, Mode : ACTIVE");
              State = ForwardSpeed2; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }

          // Si l'état est "LeftSpeed3" alors le prochain état sera "ForwardSpeed3"
          case LeftSpeed3: {
              Serial.println("Next state : ForwardSpeed3, Mode : ACTIVE");
              State = ForwardSpeed3; // Prochain état
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

        } // Fin Switch State

        Process_CMD  = 0;
        break; // Critique, ne pas l'oublier autrement, on enchaîne sur le bloc suivant !

      } // Fin case cmdFORWARD


    case cmdBACKWARD: { // Commande "Reculer tout droit" reçue

        switch (State) { // Début Switch State

          case Standby: {
              Serial.println("Next state : Standby, Mode : SLEEPING");
              State = Standby;
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }
          case StopMotor: {
              Serial.println("Next state : BackwardSpeed1, Mode : ACTIVE");
              State = BackwardSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed1: {
              Serial.println("Next state : StopMotor, Mode : SLEEPING");
              State = StopMotor;
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }
          case ForwardSpeed2: {
              Serial.println("Next state : ForwardSpeed1, Mode : ACTIVE");
              State = ForwardSpeed1;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed3: {
              Serial.println("Next state : ForwardSpeed2, Mode : ACTIVE");
              State = ForwardSpeed2;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed1: {
              Serial.println("Next state : BackwardSpeed2, Mode : ACTIVE");
              State = BackwardSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed2: {
              Serial.println("Next state : BackwardSpeed3, Mode : ACTIVE");
              State = BackwardSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed3: {
              Serial.println("Next state : BackwardSpeed3, Mode : ACTIVE");
              State = BackwardSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed1: {
              Serial.println("Next state : BackwardSpeed1, Mode : ACTIVE");
              State = BackwardSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed2: {
              Serial.println("Next state : BackwardSpeed2, Mode : ACTIVE");
              State = BackwardSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed3: {
              Serial.println("Next state : BackwardSpeed3, Mode : ACTIVE");
              State = BackwardSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed1: {
              Serial.println("Next state : BackwardSpeed1, Mode : ACTIVE");
              State = BackwardSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed2: {
              Serial.println("Next state : BackwardSpeed2, Mode : ACTIVE");
              State = BackwardSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed3: {
              Serial.println("Next state : BackwardSpeed3, Mode : ACTIVE");
              State = BackwardSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

        } // Fin Switch State

        Process_CMD  = 0;
        break;  // Critique, ne pas l'oublier autrement, on enchaîne sur le bloc suivant !

      } // Fin case cmdBACKWARD

    case cmdRIGHT: { // Commande "Tourner à droite" reçue

        switch (State) { // Début Switch State

          case Standby: {
              Serial.println("Next state : Standby, Mode : SLEEPING");
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              State = Standby;
              Mode = SLEEPING;
              break;
            }
          case StopMotor: {
              Serial.println("Next state : RightSpeed1, Mode : ACTIVE");
              State = RightSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed1: {
              Serial.println("Next state : RightSpeed1, Mode : ACTIVE");
              State = RightSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed2: {
              Serial.println("Next state : RightSpeed2, Mode : ACTIVE");
              State = RightSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed3: {
              Serial.println("Next state : RightSpeed3, Mode : ACTIVE");
              State = RightSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed1: {
              Serial.println("Next state : RightSpeed1, Mode : ACTIVE");
              State = RightSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed2: {
              Serial.println("Next state : RightSpeed2, Mode : ACTIVE");
              State = RightSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed3: {
              Serial.println("Next state : RightSpeed3, Mode : ACTIVE");
              State = RightSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed1: {
              Serial.println("Next state : RightSpeed2, Mode : ACTIVE");
              State = RightSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed2: {
              Serial.println("Next state : RightSpeed3, Mode : ACTIVE");
              State = RightSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed3: {
              Serial.println("Next state : RightSpeed3, Mode : ACTIVE");
              State = RightSpeed3;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed1: {
              Serial.println("Next state : LeftSpeed1, Mode : SLEEPING");
              State = StopMotor;
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }
          case LeftSpeed2: {
              Serial.println("Next state : LeftSpeed1, Mode : ACTIVE");
              State = LeftSpeed1;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed3: {
              Serial.println("Next state : LeftSpeed2, Mode : ACTIVE");
              State = LeftSpeed2;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }

        } // Fin Switch State

        Process_CMD  = 0;
        break;  // Critique, ne pas l'oublier autrement, on enchaîne sur le bloc suivant !

      } // Fin case cmdRIGHT

    case cmdLEFT: { // Commande "Tourner à gauche" reçue

        switch (State) { // Début Switch State

          case Standby: {
              Serial.println("Next state : Standby, Mode : SLEEPING");
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              State = Standby;
              Mode = SLEEPING;
              break;
            }
          case StopMotor: {
              Serial.println("Next state : LeftSpeed1, Mode : ACTIVE");
              State = LeftSpeed1;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed1: {
              Serial.println("Next state : LeftSpeed1, Mode : ACTIVE");
              State = LeftSpeed1;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed2: {
              Serial.println("Next state : LeftSpeed2, Mode : ACTIVE");
              State = LeftSpeed2;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case ForwardSpeed3: {
              Serial.println("Next state : LeftSpeed3, Mode : ACTIVE");
              State = LeftSpeed3;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed1: {
              Serial.println("Next state : LeftSpeed1, Mode : ACTIVE");
              State = LeftSpeed1;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed2: {
              Serial.println("Next state : LeftSpeed2, Mode : ACTIVE");
              State = LeftSpeed2;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case BackwardSpeed3: {
              Serial.println("Next state : LeftSpeed3, Mode : ACTIVE");
              State = LeftSpeed3;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed1: {
              Serial.println("Next state : StopMotor, Mode : SLEEPING");
              State = StopMotor;
              _RightMotorDir = STOPPED;
              _LeftMotorDir = STOPPED;
              __SpeedRight = V0;
              __SpeedLeft = V0;
              Mode = SLEEPING;
              break;
            }
          case RightSpeed2: {
              Serial.println("Next state : RightSpeed1, Mode : ACTIVE");
              State = RightSpeed1;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V1;
              __SpeedLeft = V1;
              Mode = ACTIVE;
              break;
            }
          case RightSpeed3: {
              Serial.println("Next state : RightSpeed2, Mode : ACTIVE");
              State = RightSpeed2;
              _RightMotorDir = BACKWARD;
              _LeftMotorDir = FORWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed1: {
              Serial.println("Next state : LeftSpeed2, Mode : ACTIVE");
              State = LeftSpeed2;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V2;
              __SpeedLeft = V2;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed2: {
              Serial.println("Next state : LeftSpeed3, Mode : ACTIVE");
              State = LeftSpeed3;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }
          case LeftSpeed3: {
              Serial.println("Next state : LeftSpeed3, Mode : ACTIVE");
              State = LeftSpeed3;
              _RightMotorDir = FORWARD;
              _LeftMotorDir = BACKWARD;
              __SpeedRight = V3;
              __SpeedLeft = V3;
              Mode = ACTIVE;
              break;
            }

        } // Fin Switch State

        Process_CMD  = 0;
        break; // Critique, ne pas l'oublier autrement, on enchaîne sur le bloc suivant !

      } // Fin case cmdLEFT

    case cmdLEARNING: { // Commande "Apprentissage des accélérations" reçue
        Learning();
        break;
      } // Fin case cmdLEARNING

    default: // Si c'est une autre commande qui se trouve dans Cmd ...
      break; // ... ne fais rien !

  } // Fin Switch Cmd
}

/************************************************************************/
/* Machine d'états pour le système anti-collisions                      */
/************************************************************************/

void Anti_Collisions_System(void) {

  enum STATE {
    Active,
    Inactive
  };

  static enum STATE State = Inactive;

  switch (State) {

    case Inactive: {
        if (Mode == ACTIVE)
          State = Active;
        else {
          _SpeedRight = __SpeedRight;
          _SpeedLeft = __SpeedLeft;
          RightMotorDir = _RightMotorDir;
          LeftMotorDir = _LeftMotorDir;
        }
        break;
      }

    case Active: {

        if (Mode == SLEEPING) State = Inactive;

        // Relevé de distances (en mm) avec les capteurs de temps de vol

#if ACTIVATE_TOF == 1

        //uint16_t depth_front_right = ToF1.readRangeContinuousMillimeters();
        //uint16_t depth_front_left =  ToF2.readRangeContinuousMillimeters();
        uint16_t dist_front_right = ToF3.readRangeContinuousMillimeters();
        uint16_t dist_front_left = ToF4.readRangeContinuousMillimeters();

#else

        const int16_t dist_front_right = 500;
        const int16_t dist_front_left =  500;

#endif

        // Si on avance en ligne droite
        if (_RightMotorDir == FORWARD && _LeftMotorDir == FORWARD)
        {

          // Si un vide est détecté à droite où à gauche
          /*
            if ((depth_front_right > THR_DEPTH) || (depth_front_left > THR_DEPTH))
            {
            // On lance la commande STOP
            Cmd = cmdSTOP;
            Process_CMD = 1;
            Serial.println("Vide détecté !");

            } else
          */

          // Si on ne détecte aucun obstacle plus proche que la distance de collision
          if ((dist_front_right > THR_DISTANCE) && (dist_front_left > THR_DISTANCE))
          {
            // On conserve les consignes issues de ProcessCommand.
            _SpeedRight = __SpeedRight;
            _SpeedLeft = __SpeedLeft;
            RightMotorDir = _RightMotorDir;
            LeftMotorDir = _LeftMotorDir;
          }

          // Si obstacle détecté à gauche
          else if ((dist_front_right > THR_DISTANCE) && (dist_front_left < THR_DISTANCE))
          {
            // On translate à droite
            _SpeedRight = V1;
            _SpeedLeft = V1;
            RightMotorDir = RIGHT_SLIDE;
            LeftMotorDir = RIGHT_SLIDE;
            Serial.print("Obstacle détecté à gauche (mm) : ");
            Serial.println(dist_front_left);
          }

          // Si obstacle détecté à droite
          else if ((dist_front_right < THR_DISTANCE) && (dist_front_left > THR_DISTANCE))
          {
            // On translate à gauche
            _SpeedRight = V1;
            _SpeedLeft = V1;
            RightMotorDir = LEFT_SLIDE;
            LeftMotorDir = LEFT_SLIDE;
            Serial.print("Obstacle détecté à droite (mm) : ");
            Serial.println(dist_front_right);
          }

          // Si obstacle détecté devant
          else if ((dist_front_right < THR_DISTANCE) && (dist_front_left < THR_DISTANCE))
          {
            // On lance la commande STOP
            Cmd = cmdSTOP;
            Process_CMD = 1;

            Serial.println("Obstacle détecté devant");
            Serial.print("Distance à droite (mm) : ");
            Serial.println(dist_front_right);
            Serial.print("Distance à gauche (mm) : ");
            Serial.println(dist_front_left);
          }

        }
        /*
          // Si on recule en ligne droite (cas non traité car pas de capteurs à l'arrière)
          else if (_RightMotorDir == BACKWARD && _LeftMotorDir == BACKWARD)
          {

          }
        */
        // Pour tout autre mouvement, on ne change rien :
        // on conserve les consignes issues de ProcessCommand.
        else
        {
          _SpeedRight = __SpeedRight;
          _SpeedLeft = __SpeedLeft;
          RightMotorDir = _RightMotorDir;
          LeftMotorDir = _LeftMotorDir;
        }

        break;
      }

  } // Fin switch State

}

/************************************************************************/
/* Calcul de la vitesse de déplacement (linéaire)                       */
/************************************************************************/

void Compute_Speed(void) {

  if (Mode == ACTIVE) {

    // Obtention des vitesses (valeurs des compteurs des fronts de fourches optiques)
    SpeedRightFront = EdgeCountRightFront;
    SpeedRightRear = EdgeCountRightRear;
    SpeedLeftFront = EdgeCountLeftFront;
    SpeedLeftRear = EdgeCountLeftRear;

    // Réinitialisation des compteurs des fronts des fourches optiques
    EdgeCountRightFront = 0;
    EdgeCountRightRear = 0;
    EdgeCountLeftFront = 0;
    EdgeCountLeftRear = 0;

#if DEBUG == 1 
    Serial.print("Vitesse moteur avant-droit : ");
    Serial.println(SpeedRightFront);
    Serial.print("Vitesse moteur arrière-droit : ");
    Serial.println(SpeedRightRear);
    Serial.print("Vitesse moteur avant-gauche : ");
    Serial.println(SpeedLeftFront);
    Serial.print("Vitesse moteur arrière-gauche : ");
    Serial.println(SpeedLeftRear);
#endif

#if DISP_CODERS == 1
    Serial.print(SpeedRightFront);
    Serial.print(" ");
    Serial.print(SpeedRightRear);
    Serial.print(" ");
    Serial.print(SpeedLeftFront);
    Serial.print(" ");
    Serial.println(SpeedLeftRear);
#endif

  }

}

/************************************************************************/
/* Calcul de la rétroaction / régulation PID                            */
/************************************************************************/
void PID_Regulator(void)
{
  enum STATE {
    On, // Veille
    Off // Arrêt
  };

  static enum STATE State = Off;

  // Glossaire
  // RF : Rigth Front motor (moteur avant droit)
  // RR : Rigth Rear motor (moteur arrière droit)
  // LF : Left Front motor (moteur avant gauche)
  // LR : Left Rear motor (moteur arrière gauche)

  // Ecarts entre vitesses mesurées et consignes de vitesses
  static float Err_RF = 0;
  static float Err_RR = 0;
  static float Err_LF = 0;
  static float Err_LR = 0;

  static float Err_RF_old = 0;
  static float Err_RR_old = 0;
  static float Err_LF_old = 0;
  static float Err_LR_old = 0;

  // Intégrales des écarts entre consignes et mesures de vitesses
  static float sum_Err_RF = 0;
  static float sum_Err_RR = 0;
  static float sum_Err_LF = 0;
  static float sum_Err_LR = 0;

  // Dérivées des écarts entre consignes et mesures de vitesses
  static float dif_Err_RF = 0;
  static float dif_Err_RR = 0;
  static float dif_Err_LF = 0;
  static float dif_Err_LR = 0;

  switch (State) {

    case Off: { // Si contrôle PID inactif

        if (Mode == ACTIVE) {
          State = On;
          Serial.println("Régulateur PID activé");
        }
        else {

          // Arrête les 4 moteurs (roues livres)
          Motors_Stopped();

          // On démarre le compteur du temps passé immobile
          MotionlessCounter = 0;

        }

        break; // Sortie case Off
      }

    case On: { // Si contrôle PID actif

        // Correction PID active uniquement lorsque le robot progresse en ligne droite
        // Si la consigne de vitesse est non-nulle, réinitialise le compteur du temps passé immobile
        if ((_SpeedRight != V0) && (_SpeedLeft != V0)) MotionlessCounter = 0;

        // Si on est en mode sommeil et immobile depuis plus que deux secondes...
        if ((Mode == SLEEPING) && (_SpeedRight == V0) && (_SpeedLeft == V0) && (MotionlessCounter > TIME_MOTIONLESS))
        {
          State = Off; // ... on bascule dans l'état Off
          Serial.println("Régulateur PID désactivé");
        }
        // Autrement, calcul de la correction PID
        else {

          // Ecarts entre les consignes de vitesse des moteurs et leurs vitesses constatées (accélérations)
          Err_RF = (float)_SpeedRight - (float)SpeedRightFront;
          Err_RR = (float)_SpeedRight - (float)SpeedRightRear;
          Err_LF = (float)_SpeedLeft - (float)SpeedLeftFront;
          Err_LR = (float)_SpeedLeft - (float)SpeedLeftRear;

          // Sommes (intégrales discrêtes) des écarts aux consignes de vitesses
          sum_Err_RF += Err_RF;
          sum_Err_RR += Err_RR;
          sum_Err_LF += Err_LF;
          sum_Err_LR += Err_LR;

          // Différences (dérivées discrêtes)des écarts aux consignes de vitesses
          dif_Err_RF = Err_RF - Err_RF_old;
          dif_Err_RR = Err_RR - Err_RR_old;
          dif_Err_LF = Err_LF - Err_LF_old;
          dif_Err_LR = Err_LR - Err_LR_old;

          // Sauvegarde pour le calcul de la dérivée des écarts au pas de temps suivant
          Err_RF_old = Err_RF;
          Err_RR_old = Err_RR;
          Err_LF_old = Err_LF;
          Err_LR_old = Err_LR;

          // Calcul des nouvelles commandes de vitesses
          float Cmd_SpeedRightFront = CKp * Err_RF + CKi * sum_Err_RF + CKd * dif_Err_RF;
          float Cmd_SpeedRightRear = CKp * Err_RR + CKi * sum_Err_RR + CKd * dif_Err_RR;
          float Cmd_SpeedLeftFront = CKp * Err_LF + CKi * sum_Err_LF + CKd * dif_Err_LF;
          float Cmd_SpeedLeftRear = CKp * Err_LR + CKi * sum_Err_LR + CKd * dif_Err_LR;

          // On limite les commandes à l'intervalle [PWM_MIN, PWM_MAX] (rapports cycliques)

          if (Cmd_SpeedRightFront < PWM_MIN) Cmd_SpeedRightFront = PWM_MIN;
          else if (Cmd_SpeedRightFront > PWM_MAX) Cmd_SpeedRightFront = PWM_MAX;

          if (Cmd_SpeedRightRear < PWM_MIN) Cmd_SpeedRightRear = PWM_MIN;
          else if (Cmd_SpeedRightRear > PWM_MAX) Cmd_SpeedRightRear = PWM_MAX;

          if (Cmd_SpeedLeftFront < PWM_MIN) Cmd_SpeedLeftFront = PWM_MIN;
          else if (Cmd_SpeedLeftFront > PWM_MAX) Cmd_SpeedLeftFront = PWM_MAX;

          if (Cmd_SpeedLeftRear < PWM_MIN) Cmd_SpeedLeftRear = PWM_MIN;
          else if (Cmd_SpeedLeftRear > PWM_MAX) Cmd_SpeedLeftRear = PWM_MAX;

          // Application de la commande aux moteurs

          // Gestion des moteurs droits
          switch (RightMotorDir) {

            case STOPPED: {
                Motor_RF_Stopped();
                Motor_RR_Stopped();
                break; // sortie case STOPPED
              }

            case FORWARD: {
                Motor_RF_Forward((uint8_t)Cmd_SpeedRightFront);
                Motor_RR_Forward((uint8_t)Cmd_SpeedRightRear);
                break; // sortie case FORWARD
              }

            case BACKWARD: {
                Motor_RF_Backward((uint8_t)Cmd_SpeedRightFront);
                Motor_RR_Backward((uint8_t)Cmd_SpeedRightRear);
                break; // sortie case backward
              }

            case RIGHT_SLIDE: {
                Motor_RF_Backward((uint8_t)Cmd_SpeedRightFront);
                Motor_RR_Forward((uint8_t)Cmd_SpeedRightRear);
                break; // sortie case right_slide
              }

            case LEFT_SLIDE: {
                Motor_RF_Forward((uint8_t)Cmd_SpeedRightFront);
                Motor_RR_Backward((uint8_t)Cmd_SpeedRightRear);
                break; // sortie case left_slide
              }

          } // fin switch RightMotorDir

          // Gestion des moteurs gauches
          switch (LeftMotorDir) {

            case STOPPED: {
                Motor_LF_Stopped();
                Motor_LR_Stopped();
                break; // sortie case STOPPED
              }

            case FORWARD: {
                Motor_LF_Forward((uint8_t)Cmd_SpeedLeftFront);
                Motor_LR_Forward((uint8_t)Cmd_SpeedLeftRear);
                break; // sortie case FORWARD
              }

            case BACKWARD: {
                Motor_LF_Backward((uint8_t)Cmd_SpeedLeftFront);
                Motor_LR_Backward((uint8_t)Cmd_SpeedLeftRear);
                break; // sortie case backward
              }

            case RIGHT_SLIDE: {
                Motor_LR_Backward((uint8_t)Cmd_SpeedLeftRear);
                Motor_LF_Forward((uint8_t)Cmd_SpeedLeftFront);
                break; // sortie case right_slide
              }

            case LEFT_SLIDE: {
                Motor_LR_Forward((uint8_t)Cmd_SpeedLeftRear);
                Motor_LF_Backward((uint8_t)Cmd_SpeedLeftFront);
                break; // sortie case left_slide
              }

          } // fin switch LeftMotorDir

        } // Fin else calcul correction PID

        break; // Sortie case On

      } // Fin switch State
  }
}



/*---------------------------- Mise en oeuvre de NanoEdge AI -----------------------------------*/

/************************************************************************/
/*  Apprentissage                                                       */
/************************************************************************/

void Learning(void)
{

  const uint32_t nb_acc_buffer = SAMPLE_RECORDS_COUNT * AXIS_NUMBER;
  enum STATE {STATE_START, STATE_START_MOTORS, STATE_LEARNING, STATE_STOP_MOTORS, STATE_END};
  static enum STATE State = STATE_START; // Prochain état

  static uint32_t LocalCounter;
  static uint32_t learning_count;

  switch (State) {
    case STATE_START: {

        learning_count = 0;
        learning_completed = 0;
        i_record = 0;

        Mode = ACTIVE;

#if DEBUG == 1
        Serial.println("Apprentissage démarré");
        Serial.print("Taille du buffer : ");
        Serial.println(nb_acc_buffer);
#else
        HC06.println("Apprentissage démarré");
        HC06.print("Taille du buffer : ");
        HC06.println(nb_acc_buffer);
#endif

        // Démarre les moteurs
        State = STATE_START_MOTORS; // Prochain état

        break;
      }

    case STATE_START_MOTORS: {

        // Ordre de lancement des moteurs
        _RightMotorDir = FORWARD; // Consigne sens de rotation prochain état
        _LeftMotorDir = FORWARD; // Consigne sens de rotation prochain état
        __SpeedRight = V1; // Consigne vitesse prochain état
        __SpeedLeft = V1; // Consigne vitesse prochain état


        // Attends 10 bases de temps avant de lancer le log des accélérations
        // l'objectif est d'avoir une vitesse stable
        if (TriggerRecord)
        {
          LocalCounter++;
          TriggerRecord = 0;
        }

        if (LocalCounter > 10 * (uint32_t)TIME_BASE)
        {
          // Démarre l'enregistrement
          State = STATE_LEARNING; // Prochain état
          LocalCounter = 0;
        }

        State = STATE_LEARNING; // Prochain état
        break;

      }

    case STATE_LEARNING: {

#if DISP_ACCELS == 1
        Serial.print("STATE_LEARNING : ");
        Serial.print(i_record);
        Serial.print("/");
        Serial.println(nb_acc_buffer);
#endif

        // Aussi longtemps que le tableau buffer n'est pas plein
        if (i_record < nb_acc_buffer)
        {
          // Remplis le buffer des accélérations
          fill_acc_buffer();
        }
        else
        {

          // Apprentissage la lib NanoEdge AI
          enum neai_state error_code = neai_anomalydetection_learn(acc_buffer);

          if (error_code != NEAI_OK) {

            learning_count = LEARNING_ROUNDS - 1;

#if DEBUG == 1
            Serial.print("La fonction neai_anomalydetection_learn signale l'erreur ");
            Serial.println(Get_NEAI_Error(error_code));
#else
            HC06.print("La fonction neai_anomalydetection_learn signale l'erreur ");
            HC06.println(Get_NEAI_Error(error_code));
#endif
          }

          i_record = 0;
          learning_count++;

          // Prochain état
          if (learning_count == LEARNING_ROUNDS) {
            State = STATE_STOP_MOTORS;
          }

        }

        break;

      }

    case STATE_STOP_MOTORS: {

        // Ordre d'arrêt des moteurs
        _RightMotorDir = STOPPED;
        _LeftMotorDir = STOPPED;
        __SpeedRight = V0;
        __SpeedLeft = V0;
        State = STATE_END; // Prochain état

        break;

      }

    case STATE_END: {

#if DEBUG == 1
        Serial.print("Apprentissage terminé après ");
        Serial.print(learning_count);
        Serial.println(" rounds");
#else
        HC06.print("Apprentissage terminé après ");
        HC06.print(learning_count);
        HC06.println(" rounds");
#endif


#if DISP_ACCELS == 1
        for (uint32_t i = 0; i < nb_acc_buffer; i++)
        {
          Serial.print(i);
          Serial.print(" : ");
          Serial.println(acc_buffer[i]);
        }
#endif

        Mode = SLEEPING;
        State = STATE_START;
        i_record = 0;
        learning_completed = 1;
        learning_count = 0;

        // Autorise la réception de nouvelles commandes
        Process_CMD  = 0;

        break;
      }
  } // Fin switch (State)


}


/************************************************************************/
/*  Détection d'anomalies                                               */
/************************************************************************/
void FaultDetection(void)
{

  const uint32_t nb_acc_buffer = SAMPLE_RECORDS_COUNT * AXIS_NUMBER;

  // Si on est en mouvement et que l'on a une vitesse proche de la consigne V1
  // (ce qui correspond à la vitesse de génération de la lib NEAI)

  if (learning_completed && (Mode == ACTIVE) && (abs(SpeedRightFront - V1) < 2) && (abs(SpeedLeftFront - V1) < 2))
  {

    // Aussi longtemps que le tableau buffer n'est pas plein
    if (i_record < nb_acc_buffer )
    {
      // Remplis le buffer des accélérations
      fill_acc_buffer();
    }
    else
    {
      // Effectue une mesure d'anomalie
      enum neai_state error_code = neai_anomalydetection_detect(acc_buffer, &similarity);

      i_record = 0;

      if (error_code != NEAI_OK) {
#if DEBUG == 1
        Serial.print("la fonction neai_anomalydetection_detect signale l'erreur ");
        Serial.println(Get_NEAI_Error(error_code));
#else
        HC06.print("la fonction neai_anomalydetection_detect signale l'erreur ");
        HC06.println(Get_NEAI_Error(error_code));
#endif
      }

      // Si on estime avoir détecté une anomalie
      if (similarity < SIMILARITY_THRESHOLD)
      {
#if DEBUG == 1
        Serial.println("Anomalie détectée");
        Serial.print("Similarité : ");
        Serial.println(similarity);
#else
        LightBar_Signal(cRED); // Flash rouge sur la barre de LED
        HC06.println("Anomalie détectée");
        HC06.print("Similarité : ");
        HC06.println(similarity);
#endif
      }

    }

  }
}

/************************************************************************/
/* Remplis le tableau tampon des accélérations                          */
/************************************************************************/
void fill_acc_buffer(void)
{

  // A chaque interruption, acquière et enregistre les accélérations sur les 3 axes

  if (TriggerRecord)
  {
    Acc.Get_X_Axes(acc_value);
    acc_buffer[i_record] = (float)acc_value[0];
    acc_buffer[i_record + 1] = (float)acc_value[1];
    acc_buffer[i_record + 2] = (float)acc_value[2];
    i_record += 3;

#if DISP_ACCELS == 1
    Serial.print(acc_value[0]);
    Serial.print(" ");
    Serial.print(acc_value[1]);
    Serial.print(" ");
    Serial.println(acc_value[2]);
#endif

    TriggerRecord = 0;
  }
}

/************************************************************************/
/*  Traduction des code d'erreur de NanoEdge AI                         */
/************************************************************************/
String Get_NEAI_Error(enum neai_state error_code)
{
  switch (error_code)
  {
    case NEAI_INIT_FCT_NOT_CALLED: {
        return  "NEAI_INIT_FCT_NOT_CALLED";
      }
    case NEAI_BOARD_ERROR: {
        return "NEAI_BOARD_ERROR";
      }
    case NEAI_KNOWLEDGE_BUFFER_ERROR: {
        return "NEAI_KNOWLEDGE_BUFFER_ERROR";
      }
    case NEAI_NOT_ENOUGH_CALL_TO_LEARNING: {
        return "AI_NOT_ENOUGH_CALL_TO_LEARNING";
      }
    case NEAI_UNKNOWN_ERROR: {
        return "NEAI_UNKNOWN_ERROR";
      }
  }
}

/************************************************************************/
/*  Initialisation du moteur de NanoEdge AI                             */
/************************************************************************/
void init_NEAI(void)
{
  enum neai_state error_code = neai_anomalydetection_init();

  if (error_code != NEAI_OK) {
    Serial.print("La fonction neai_anomalydetection_init signale l'erreur ");
    Serial.println(Get_NEAI_Error(error_code));
    while (1);
  }
  Serial.println("Détection d'anomalies NEAI active");

}

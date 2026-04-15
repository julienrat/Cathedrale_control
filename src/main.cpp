#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>

constexpr uint8_t LED_COUNT = 8;
// Note: L'ESP32-H2 n'a que 6 canaux PWM. Seules les 6 premières LEDs fonctionneront en variation.
constexpr uint8_t LED_PINS[LED_COUNT] = {26, 27, 10, 11, 0, 1, 2, 3}; 
uint8_t ledValues[LED_COUNT] = {0};
constexpr uint8_t BUTTON_PIN = 22; // Le bouton est maintenant en 22

// --- UTILITAIRES ---
template <typename T>
T clampValue(T value, T minValue, T maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void writeLedLevel(uint8_t ledIndex, uint8_t value) {
  if (ledIndex >= LED_COUNT) {
    return;
  }
  uint8_t pin = LED_PINS[ledIndex];
  ledValues[ledIndex] = value;

  if (value == 0) {
    // Si la luminosité est à 0, on libère le canal PWM pour les autres LEDs
    ledcDetach(pin);
    digitalWrite(pin, LOW);
  } else {
    // analogWrite va automatiquement tenter d'allouer un des 6 canaux PWM disponibles
    analogWrite(pin, value);
  }
}

void allLedsOff() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    writeLedLevel(i, 0);
  }
}

void setupLedOutputs() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    analogWriteResolution(LED_PINS[i], 8); // Définit la résolution pour chaque broche (0-255)
    writeLedLevel(i, 0);
  }
}

// --- CONFIGURATION ET STOCKAGE ---
constexpr size_t MAX_NODES = 40;
constexpr size_t JSON_CAPACITY = 16384;
constexpr char PREF_NAMESPACE[] = "led_flow";
constexpr char PREF_KEY[] = "graph";

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Buffer pour la réception Bluetooth (JSON fragmenté)
String bleIncomingBuffer = "";
bool newSequenceReady = false;

struct SequenceNode {
  int id;
  uint8_t ledMask; // Masque de bits pour gérer plusieurs LEDs (bit 0 = LED 1, etc.)
  uint8_t target;
  uint16_t fadeMs;
  uint16_t holdMs;
  int next;
  int16_t x;
  int16_t y;
};

// --- VARIABLES GLOBALES ---
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
Preferences prefs;
SequenceNode nodes[MAX_NODES];
size_t nodeCount = 0;
int startNodeId = -1;
bool running = false;
int activeNodeId = -1;
bool stepPrimed = false;
unsigned long stepStartAtMs = 0;
uint8_t stepStartValues[LED_COUNT] = {0}; // On stocke le départ de chaque LED

int findNodeIndexById(int id) {
  for (size_t i = 0; i < nodeCount; i++) {
    if (nodes[i].id == id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool nodeIdExistsInBuffer(const SequenceNode *buffer, size_t count, int id) {
  for (size_t i = 0; i < count; i++) {
    if (buffer[i].id == id) {
      return true;
    }
  }
  return false;
}

void setDefaultSequence() {
  nodeCount = 0;
  int id = 1;

  for (uint8_t led = 0; led < LED_COUNT && nodeCount + 1 < MAX_NODES; led++) {
    const int row = led / 2;
    const int column = led % 2;
    const int baseX = 28 + column * 360;
    const int baseY = 28 + row * 150;

    nodes[nodeCount++] = {
        id,
        static_cast<uint8_t>(1 << led),
        255,
        420,
        80,
        id + 1,
        static_cast<int16_t>(baseX),
        static_cast<int16_t>(baseY),
    };

    id++;

    const int nextId = (led == LED_COUNT - 1) ? 1 : id + 1;
    nodes[nodeCount++] = {
        id,
        static_cast<uint8_t>(1 << led),
        0,
        340,
        50,
        nextId,
        static_cast<int16_t>(baseX + 190),
        static_cast<int16_t>(baseY),
    };

    id++;
  }

  startNodeId = (nodeCount > 0) ? nodes[0].id : -1;
  running = false;
  activeNodeId = startNodeId;
  stepPrimed = false;
}

// --- GESTION JSON ---
bool parseSequenceJson(const String &payload, String &errorMessage) {
  DynamicJsonDocument doc(JSON_CAPACITY);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    errorMessage = "JSON invalide";
    return false;
  }

  JsonArray arr = doc["nodes"].as<JsonArray>();
  if (arr.isNull()) {
    errorMessage = "Le tableau nodes est requis";
    return false;
  }

  if (arr.size() > MAX_NODES) {
    errorMessage = "Trop de blocs";
    return false;
  }

  SequenceNode parsed[MAX_NODES];
  size_t parsedCount = 0;

  for (JsonVariant entry : arr) {
    if (!entry.is<JsonObject>()) {
      continue;
    }

    const int id = entry["id"] | static_cast<int>(parsedCount + 1);
    
    // Conversion du tableau "leds" [0, 2] en masque de bits
    uint8_t mask = 0;
    JsonArray ledsArr = entry["leds"].as<JsonArray>();
    for (JsonVariant l : ledsArr) {
      int ledIdx = l.as<int>();
      if (ledIdx >= 0 && ledIdx < LED_COUNT) {
        mask |= (1 << ledIdx);
      }
    }

    const int target = clampValue<int>(entry["target"] | 0, 0, 255);
    const int fadeMs = clampValue<int>(entry["fadeMs"] | 0, 0, 60000);
    const int holdMs = clampValue<int>(entry["holdMs"] | 0, 0, 60000);
    const int next = entry.containsKey("next") ? static_cast<int>(entry["next"]) : -1;
    const int x = clampValue<int>(entry["x"] | 20, 0, 2500);
    const int y = clampValue<int>(entry["y"] | 20, 0, 2500);

    parsed[parsedCount++] = {
        id,
        mask,
        static_cast<uint8_t>(target),
        static_cast<uint16_t>(fadeMs),
        static_cast<uint16_t>(holdMs),
        next,
        static_cast<int16_t>(x),
        static_cast<int16_t>(y),
    };
  }

  if (parsedCount == 0) {
    errorMessage = "La sequence ne contient aucun bloc";
    return false;
  }

  for (size_t i = 0; i < parsedCount; i++) {
    for (size_t j = i + 1; j < parsedCount; j++) {
      if (parsed[i].id == parsed[j].id) {
        errorMessage = "Chaque bloc doit avoir un id unique";
        return false;
      }
    }
  }

  int parsedStart = doc["startId"] | parsed[0].id;
  if (!nodeIdExistsInBuffer(parsed, parsedCount, parsedStart)) {
    parsedStart = parsed[0].id;
  }

  for (size_t i = 0; i < parsedCount; i++) {
    if (parsed[i].next >= 0 && !nodeIdExistsInBuffer(parsed, parsedCount, parsed[i].next)) {
      parsed[i].next = -1;
    }
  }

  for (size_t i = 0; i < parsedCount; i++) {
    nodes[i] = parsed[i];
  }

  nodeCount = parsedCount;
  startNodeId = parsedStart;
  running = false;
  activeNodeId = startNodeId;
  stepPrimed = false;
  return true;
}

String buildSequenceJson() {
  DynamicJsonDocument doc(JSON_CAPACITY);
  doc["startId"] = startNodeId;
  doc["running"] = running;

  JsonArray pins = doc.createNestedArray("pins");
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    pins.add(LED_PINS[i]);
  }

  JsonArray arr = doc.createNestedArray("nodes");
  for (size_t i = 0; i < nodeCount; i++) {
    JsonObject entry = arr.createNestedObject();
    entry["id"] = nodes[i].id;
    JsonArray leds = entry.createNestedArray("leds");
    for (int l = 0; l < LED_COUNT; l++) {
      if (nodes[i].ledMask & (1 << l)) {
        leds.add(l);
      }
    }
    entry["target"] = nodes[i].target;
    entry["fadeMs"] = nodes[i].fadeMs;
    entry["holdMs"] = nodes[i].holdMs;
    entry["next"] = nodes[i].next;
    entry["x"] = nodes[i].x;
    entry["y"] = nodes[i].y;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// Fonction pour envoyer le JSON complet par morceaux via Bluetooth
void sendFullState() {
  if (!deviceConnected || !pCharacteristic) return;
  String json = buildSequenceJson();
  Serial.printf("[BLE] Envoi de la configuration (%u octets)...\n", (unsigned int)json.length());

  // On envoie par blocs de 128 octets pour ne pas saturer le MTU Bluetooth
  for (size_t i = 0; i < json.length(); i += 128) {
    String chunk = json.substring(i, i + 128);
    pCharacteristic->setValue(chunk.c_str());
    pCharacteristic->notify();
    delay(40); // Délai pour laisser le temps au client de traiter
  }

  // Envoi explicite du caractère nul pour terminer le message
  uint8_t terminator = 0;
  pCharacteristic->setValue(&terminator, 1);
  pCharacteristic->notify();

  Serial.println("[BLE] Envoi terminé");
}

// Envoie l'ID du bloc actif vers le Web Bluetooth
void notifyActiveNode(int id) {
  if (deviceConnected && pCharacteristic) {
    StaticJsonDocument<32> statusDoc;
    statusDoc["active"] = id;
    char buffer[32];
    serializeJson(statusDoc, buffer);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
  }
}

// --- MOTEUR DE SÉQUENCE ---
void setRunState(bool shouldRun) {
  if (!shouldRun || nodeCount == 0 || startNodeId < 0) {
    running = false;
    activeNodeId = startNodeId;
    stepPrimed = false;
    notifyActiveNode(-1);
    return;
  }

  running = true;
  activeNodeId = startNodeId;
  stepPrimed = false;
  notifyActiveNode(activeNodeId);
}

void updateSequenceEngine() {
  if (!running || nodeCount == 0) {
    return;
  }

  const int nodeIndex = findNodeIndexById(activeNodeId);
  if (nodeIndex < 0) {
    running = false;
    stepPrimed = false;
    return;
  }

  SequenceNode &node = nodes[nodeIndex];

  if (!stepPrimed) {
    stepPrimed = true;
    stepStartAtMs = millis();
    // Sauvegarde des valeurs de départ pour toutes les LEDs du masque
    for (int i = 0; i < LED_COUNT; i++) {
      stepStartValues[i] = ledValues[i];
    }
  }

  const unsigned long elapsed = millis() - stepStartAtMs;
  const unsigned long fadeDuration = static_cast<unsigned long>(node.fadeMs);
  const unsigned long holdDuration = static_cast<unsigned long>(node.holdMs);

  if (fadeDuration == 0 || elapsed >= fadeDuration) {
    for (int i = 0; i < LED_COUNT; i++) {
      if (node.ledMask & (1 << i)) writeLedLevel(i, node.target);
    }
  } else {
    const float progress = static_cast<float>(elapsed) / static_cast<float>(fadeDuration);
    for (int i = 0; i < LED_COUNT; i++) {
      if (node.ledMask & (1 << i)) {
        const int value = static_cast<int>(stepStartValues[i] + (node.target - stepStartValues[i]) * progress + 0.5f);
        writeLedLevel(i, static_cast<uint8_t>(clampValue(value, 0, 255)));
      }
    }
  }

  if (elapsed < fadeDuration + holdDuration) {
    return;
  }

  if (node.next < 0) {
    running = false;
    stepPrimed = false;
    return;
  }

  activeNodeId = node.next;
  notifyActiveNode(activeNodeId);
  stepPrimed = false;
}

void saveCurrentSequence() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString(PREF_KEY, buildSequenceJson());
  prefs.end();
}

bool loadSavedSequence(String &errorMessage) {
  prefs.begin(PREF_NAMESPACE, true);
  String payload = prefs.getString(PREF_KEY, "");
  prefs.end();

  if (payload.length() == 0) {
    errorMessage = "Aucune sequence sauvee";
    return false;
  }

  return parseSequenceJson(payload, errorMessage);
}

// --- GESTION BLUETOOTH (BLE) ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    BLEDevice::startAdvertising(); // Relancer l'annonce pour permettre une reconnexion
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue(); // Correction: Utilisation de String (Arduino)
    for (size_t i = 0; i < value.length(); i++) {
      if (value[i] == '\0') { // Fin du message JSON envoyée par le JS
        newSequenceReady = true;
        break;
      }
      bleIncomingBuffer += (char)value[i];
    }
  }
};

void processIncomingCommand() {
  if (!newSequenceReady) return;

  DynamicJsonDocument doc(JSON_CAPACITY);
  DeserializationError error = deserializeJson(doc, bleIncomingBuffer);

  if (!error) {
    String cmd = doc["cmd"] | "";
    if (cmd == "sequence") {
      String err;
      String dataStr;
      serializeJson(doc["data"], dataStr);
      parseSequenceJson(dataStr, err);
      Serial.println("[BLE] Nouvelle séquence reçue");
    } 
    else if (cmd == "get") {
      sendFullState();
      Serial.println("[BLE] État envoyé au client");
    }
    else if (cmd == "load") {
      String err;
      if (loadSavedSequence(err)) {
        sendFullState();
        Serial.println("[BLE] Séquence rechargée depuis la Flash");
      }
    }
    else if (cmd == "start") { setRunState(true); Serial.println("[BLE] Start"); }
    else if (cmd == "stop")  { setRunState(false); Serial.println("[BLE] Stop"); }
    else if (cmd == "save")  { saveCurrentSequence(); Serial.println("[BLE] Saved to Flash"); }
  }

  bleIncomingBuffer = "";
  newSequenceReady = false;
}

// --- SETUP & LOOP ---
void setup() {
  Serial.begin(115200);
  setupLedOutputs();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Charger la séquence sauvegardée ou mettre celle par défaut
  String err;
  if (!loadSavedSequence(err)) {
    setDefaultSequence();
  }

  // Initialisation BLE
  BLEDevice::init("LED-Flow-H2");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[SYSTEM] ESP32-H2 prêt. En attente de connexion Bluetooth...");
}

void loop() {
  processIncomingCommand();

  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Détection de l'appui (Front descendant car INPUT_PULLUP)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("[BUTTON] Appuyé : Lancement de la séquence");
    setRunState(true);
  } 
  // Détection du relâchement (Front montant)
  else if (currentButtonState == HIGH && lastButtonState == LOW) {
    Serial.println("[BUTTON] Relâché : Arrêt et extinction");
    setRunState(false);
    allLedsOff();
  }

  lastButtonState = currentButtonState;
  updateSequenceEngine();
  delay(1);
}

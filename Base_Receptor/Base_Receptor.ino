/***************************************************
   Exemplo de uso do ESP32 com:
   - Módulo LoRa E220 (Ebyte)
   - Display OLED (SSD1306)
   - GPS NEO-6M
   - Envio de dados para Supabase via Wi-Fi

   Ajuste este código de acordo com seus pinos,
   credenciais de Wi-Fi e formato de mensagem LoRa.
****************************************************/

#include <WiFi.h>
#include <WiFiClientSecure.h> // ou WiFiClient, dependendo de SSL/TLS
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =======================================================
// === Definições de Pinos e Configurações de Hardware ===
// =======================================================

// --- E220 (LoRa) ---
// Ajuste de acordo com o seu esquema de ligação
#define E220_RX_PIN 16  // RX do ESP32 <- TX do E220
#define E220_TX_PIN 17  // TX do ESP32 -> RX do E220
#define E220_AUX_PIN 34 // AUX
#define E220_M0_PIN  26 // M0
#define E220_M1_PIN  25 // M1

// --- GPS NEO-6M ---
#define GPS_RX_PIN 4  // RX do ESP32 <- TX do GPS
#define GPS_TX_PIN 2  // TX do ESP32 -> RX do GPS
// Obs.: Ajuste se necessário. No ESP32, Serial2 é comum em pinos 16/17 também.
//       Certifique-se de não conflitar com E220.

// --- Display OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1  // Se seu display não tiver reset, pode deixar -1
// Pinos I2C padrão no ESP32 DevKit V1 costumam ser SDA = 21, SCL = 22.

// ===================================
// === Objetos globais / Variáveis ===
// ===================================

// Objeto OLED (endereço I2C 0x3C é o mais comum para SSD1306 128x64)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Objeto GPS
TinyGPSPlus gps;

// Serial para GPS (usando a Serial1 ou Serial2 do ESP32)
HardwareSerial SerialGPS(1); // Podemos usar Serial1(1) ou Serial2(2). Ajustar conforme necessidade.

// Serial para o E220:
//  - Podemos usar a Serial2(2) ou instanciar outro HardwareSerial.
//  - Exemplo: usaremos Serial2(2) para E220 e Serial1(1) para GPS, ou vice-versa.
//  - Aqui, para exemplo, vamos supor E220 na Serial2 e GPS na Serial1. Ajuste conforme necessidade.
HardwareSerial SerialE220(2);

// Variáveis para armazenar dados recebidos via LoRa
float apogeu = 0.0;
float maxVelocidade = 0.0;
float maxAceleracao = 0.0;
float posX = 0.0, posY = 0.0, posZ = 0.0;
float latFoguete = 0.0;
float lonFoguete = 0.0;
int   forcaConexao = 0; // RSSI ou similar

// Variáveis para armazenar localização do Receptor (ESP)
float latReceptor = 0.0;
float lonReceptor = 0.0;

// Credenciais Wi-Fi
const char* ssid     = "SEU_SSID";
const char* password = "SUA_SENHA";

// Configuração Supabase (exemplo)
const char* supabaseHost         = "SEU_PROJETO.supabase.co";
const int   supabasePort         = 443; // 80 se for HTTP sem TLS
const char* supabaseApiEndpoint  = "/rest/v1/dadosFoguete"; // Ajuste a rota
// Chave de API e Bearer, se necessário
const char* supabaseApiKey       = "SUA_SUPABASE_ANON_OR_SERVICE_KEY";
const char* supabaseAuthBearer   = "Bearer SUA_SUPABASE_ANON_OR_SERVICE_KEY";

// ========================================================
// === Função de Cálculo de Distância (Haversine) em KM ===
// ========================================================
double distanciaGPS(double lat1, double lon1, double lat2, double lon2) {
  // Converter graus para radianos
  double radLat1 = lat1 * M_PI / 180.0;
  double radLat2 = lat2 * M_PI / 180.0;
  double dLat    = radLat2 - radLat1;
  double dLon    = (lon2 - lon1) * M_PI / 180.0;
  
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(radLat1) * cos(radLat2) *
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  // Raio médio da Terra em km ~ 6371
  double distancia = 6371.0 * c;
  return distancia;
}

// =====================================
// === Envio dos Dados para Supabase ===
// =====================================
void enviarDadosSupabase() {
  // Exemplo usando WiFiClientSecure (TLS). Se for HTTP simples, use WiFiClient.
  WiFiClientSecure client;
  client.setInsecure(); // Para não verificar certificado (não recomendado em produção)

  Serial.println("[HTTP] Conectando ao host Supabase...");
  if(!client.connect(supabaseHost, supabasePort)) {
    Serial.println("Conexão falhou.");
    return;
  }
  Serial.println("Conexão estabelecida com sucesso!");

  // Montar corpo em JSON (exemplo)
  // Ajuste conforme o esquema do seu banco/tabela
  String jsonBody = String("{") +
    "\"apogeu\":" + apogeu + "," +
    "\"maxVelocidade\":" + maxVelocidade + "," +
    "\"maxAceleracao\":" + maxAceleracao + "," +
    "\"posX\":" + posX + "," +
    "\"posY\":" + posY + "," +
    "\"posZ\":" + posZ + "," +
    "\"latFoguete\":" + latFoguete + "," +
    "\"lonFoguete\":" + lonFoguete + "," +
    "\"forcaConexao\":" + forcaConexao +
  "}";

  // Cabeçalhos HTTP
  String request = String("POST ") + supabaseApiEndpoint + " HTTP/1.1\r\n" +
                   "Host: " + supabaseHost + "\r\n" +
                   "Content-Type: application/json\r\n" +
                   "apikey: " + supabaseApiKey + "\r\n" +
                   "Authorization: " + supabaseAuthBearer + "\r\n" +
                   "Content-Length: " + jsonBody.length() + "\r\n" +
                   "Connection: close\r\n\r\n" +
                   jsonBody + "\r\n";

  // Enviar requisição
  client.print(request);
  Serial.println("[HTTP] Enviando requisição:\n" + request);

  // Ler resposta (opcional, mas útil para debug)
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      break;
    }
    Serial.println(line);
  }

  // Pode ler o corpo da resposta também, se necessário
  Serial.println("Resposta do servidor:");
  while (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
  }

  client.stop();
  Serial.println("[HTTP] Conexão encerrada.");
}

// ========================
// === Setup do Sistema ===
// ========================
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");

  // --- Inicializar OLED ---
  Wire.begin(21, 22); // Ajuste pinos I2C, se necessário
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Endereço I2C comum é 0x3C
    Serial.println("Falha ao inicializar OLED!");
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.display();

  // --- Inicializar GPS (Serial1, por exemplo) ---
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // --- Inicializar E220 (Serial2, por exemplo) ---
  pinMode(E220_AUX_PIN, INPUT);
  pinMode(E220_M0_PIN, OUTPUT);
  pinMode(E220_M1_PIN, OUTPUT);
  
  // Definir M0 e M1 em LOW para modo normal (transparent mode)
  digitalWrite(E220_M0_PIN, LOW);
  digitalWrite(E220_M1_PIN, LOW);

  SerialE220.begin(9600, SERIAL_8N1, E220_RX_PIN, E220_TX_PIN);
  delay(100);

  // --- Conectar ao WiFi ---
  WiFi.begin(ssid, password);
  Serial.println("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado com IP: " + WiFi.localIP().toString());

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("WiFi conectado");
  display.display();
  delay(1000);
}

// =======================================
// === Loop Principal do Programa ===
// =======================================
void loop() {
  // 1. Ler dados do GPS local (para saber latReceptor, lonReceptor)
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    latReceptor = gps.location.lat();
    lonReceptor = gps.location.lng();
  }

  // 2. Ler dados recebidos do E220 (LoRa).
  //    O formato de leitura depende de como o foguete envia.
  //    Aqui vamos assumir que recebemos uma string CSV ou JSON.
  if (SerialE220.available()) {
    String loraData = SerialE220.readStringUntil('\n'); // Exemplo
    loraData.trim();

    if(loraData.length() > 0) {
      Serial.print("Recebido via LoRa: ");
      Serial.println(loraData);

      // Exemplo: apogeu,maxVelocidade,maxAceleracao,posX,posY,posZ,lat,lon,forcaConexao
      // Ex: "123.4,567.8,9.10,1.2,3.4,5.6,-22.1234,-45.1234,-90"
      // Fazer parse (simples split):
      // Obs: Ajuste para JSON ou outro formato que vc use.

      const int NUM_CAMPOS = 9;
      float campos[NUM_CAMPOS];
      int i = 0;
      int startIdx = 0;

      for(int c = 0; c < loraData.length(); c++){
        if(loraData.charAt(c) == ',' || c == loraData.length()-1){
          String valor = loraData.substring(startIdx, (c == loraData.length()-1) ? c+1 : c);
          campos[i] = valor.toFloat();
          i++;
          startIdx = c+1;
          if(i >= NUM_CAMPOS) break;
        }
      }

      // Atribuir aos dados
      apogeu        = campos[0];
      maxVelocidade = campos[1];
      maxAceleracao = campos[2];
      posX          = campos[3];
      posY          = campos[4];
      posZ          = campos[5];
      latFoguete    = campos[6];
      lonFoguete    = campos[7];
      forcaConexao  = (int)campos[8];

      // 3. Calcular distância entre Receptor e Foguete
      double dist = distanciaGPS(latReceptor, lonReceptor, latFoguete, lonFoguete);

      Serial.print("Distancia (km): ");
      Serial.println(dist);

      // 4. Mostrar no OLED
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Dist (km): ");
      display.println(dist, 3); // 3 casas decimais
      display.print("Apogeu: ");
      display.println(apogeu);
      display.print("Vel: ");
      display.println(maxVelocidade);
      display.display();

      // 5. Enviar dados para Supabase
      enviarDadosSupabase();
    }
  }

  // Atraso para não ficar tão rápido (ajuste conforme necessidade)
  delay(1000);
}


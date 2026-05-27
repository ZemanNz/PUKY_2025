#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
using namespace lx16a;

auto& bus = rkSmartServoBus(2); // nebo číslo busu dle potřeby

// Pomocná funkce pro parsování parametrů z řetězce
bool parseParams(const String& params, int* out, int count) {
  int idx = 0, last = 0;
  for (int i = 0; i < count; ++i) {
    idx = params.indexOf(',', last);
    String val = (idx == -1) ? params.substring(last) : params.substring(last, idx);
    val.trim();
    if (!val.length()) return false;
    out[i] = val.toInt();
    last = idx + 1;
    if (idx == -1 && i < count - 1) return false;
  }
  return true;
}

void processCommand(const String &cmd) {
  if (cmd.startsWith("s_s_init(")) {
    int params[3];
    int start = cmd.indexOf('(') + 1;
    int end = cmd.indexOf(')');
    if (start < 1 || end < 0) {
      Serial1.println("Chybný formát s_s_init");
      return;
    }
    String paramStr = cmd.substring(start, end);
    if (!parseParams(paramStr, params, 3)) {
      Serial1.println("Chyba v parametrech s_s_init");
      return;
    }
    s_s_init(bus, params[0], params[1], params[2]);
    Serial1.println("s_s_init zavoláno");
    return;
  }
  if (cmd.startsWith("s_s_move(")) {
    int params[3] = {0, 0, 200}; // id, angle, speed (volitelný)
    int start = cmd.indexOf('(') + 1;
    int end = cmd.indexOf(')');
    if (start < 1 || end < 0) {
      Serial1.println("Chybný formát s_s_move");
      return;
    }
    String paramStr = cmd.substring(start, end);
    int paramCount = 0;
    for (int i = 0, last = 0; i < 3 && last < paramStr.length(); ++i) {
      int idx = paramStr.indexOf(',', last);
      String val = (idx == -1) ? paramStr.substring(last) : paramStr.substring(last, idx);
      val.trim();
      if (val.length()) {
        params[i] = val.toInt();
        paramCount = i + 1;
      }
      last = idx + 1;
      if (idx == -1) break;
    }
    if (paramCount < 2) {
      Serial1.println("Chyba v parametrech s_s_move");
      return;
    }
    s_s_move(bus, params[0], params[1], params[2]);
    Serial1.println("s_s_move zavoláno");
    return;
  }
  if (cmd.startsWith("s_s_soft_move(")) {
    int params[3];
    int start = cmd.indexOf('(') + 1;
    int end = cmd.indexOf(')');
    if (start < 1 || end < 0) {
      Serial1.println("Chybný formát s_s_soft_move");
      return;
    }
    String paramStr = cmd.substring(start, end);
    if (!parseParams(paramStr, params, 3)) {
      Serial1.println("Chyba v parametrech s_s_soft_move");
      return;
    }
    s_s_soft_move(bus, params[0], params[1], params[2]);
    Serial1.println("s_s_soft_move zavoláno");
    return;
  }
  Serial1.println("Neznámý příkaz");
}

void setup() {
  Serial1.begin(115200, 0, 3, 1); // RX=3, TX=1
  rkConfig cfg;
  rkSetup(cfg);
  delay(1000);
  rkLedRed(true); // Turn on red LED
  rkLedBlue(true); // Turn on blue LED
  printf("Robotka started!\n");
  
}

void loop() {
  // Pokud je aspoň jeden znak k dispozici, přečteme až po '\n'
  if (Serial1.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();               // odstraní CR/LF a mezery navíc
    if (line.length() > 0) {
      Serial1.print("Dostal jsem řetězec: "); 
      Serial1.println(line);
      processCommand(line);
    }
  }
}
/*
s_s_init(1, 0, 160)
s_s_move(1, 90)
s_s_soft_move(1, 150, 150)

GitHub Copilot
Ano, pokud používáš knihovnu Robotka pro RBCX, ta si často přebírá některé UART porty a standardní Serial (nebo Serial1, Serial2) nemusí být dostupné pro běžné použití v Serial Monitoru. To je omezení dané tím, jak je knihovna navržená a jaké periferie používá.

Možnosti řešení:

Použít jiný UART port:
Pokud Robotka používá např. Serial1, můžeš zkusit použít Serial2 (nebo naopak), ale musíš zkontrolovat, které piny a porty jsou na desce volné a nejsou obsazené knihovnou.

Použít vlastní softwarový UART:
Můžeš použít knihovnu jako SoftwareSerial (na ESP32 je to ESPSoftwareSerial), která umožní vytvořit další sériový port na libovolných pinech.
Například:

Použít jiný způsob komunikace:
Pokud potřebuješ ladit nebo ovládat robota, můžeš použít například:

I2C (pokud je volný)
UART přes jiné piny (viz výše)
Bluetooth (ESP32 má Bluetooth i BLE)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
WiFi (webový server, telnet, MQTT...)
Ladit přes LED nebo displej:
Pro jednoduché ladění můžeš používat LEDky, buzzer nebo displej, pokud je na desce.
*/
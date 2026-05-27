#include <Arduino.h>
#include "motor_commands.h"

// Pomocná funkce pro parsování parametrů z řetězce
bool parseParams(const String& params, float* out, int count) {
    int idx = 0, last = 0;
    for (int i = 0; i < count; ++i) {
        idx = params.indexOf(',', last);
        String val = (idx == -1) ? params.substring(last) : params.substring(last, idx);
        val.trim();
        if (!val.length()) return false;
        out[i] = val.toFloat();
        last = idx + 1;
        if (idx == -1 && i < count - 1) return false;
    }
    return true;
}

void processCommand(const String &cmd) {
    if (cmd.startsWith("encodery()")) {
        encodery();
        Serial.println("encodery zavoláno");
        return;
    }
    if (cmd.startsWith("forward(")) {
        float params[2];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát forward");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 2)) {
            Serial.println("Chyba v parametrech forward");
            return;
        }
        forward(params[0], params[1]);
        Serial.println("forward zavoláno");
        return;
    }
    if (cmd.startsWith("radius_r(")) {
        float params[3];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát radius_r");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 3)) {
            Serial.println("Chyba v parametrech radius_r");
            return;
        }
        radius_r(params[0], params[1], params[2]);
        Serial.println("radius_r zavoláno");
        return;
    }
    if (cmd.startsWith("radius_l(")) {
        float params[3];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát radius_l");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 3)) {
            Serial.println("Chyba v parametrech radius_l");
            return;
        }
        radius_l(params[0], params[1], params[2]);
        Serial.println("radius_l zavoláno");
        return;
    }
    if (cmd.startsWith("turn_on_spot(")) {
        float params[1];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát turn_on_spot");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 1)) {
            Serial.println("Chyba v parametrech turn_on_spot");
            return;
        }
        turn_on_spot(params[0]);
        Serial.println("turn_on_spot zavoláno");
        return;
    }
    if (cmd.startsWith("back_buttons(")) {
        float params[1];
        int start = cmd.indexOf('(') + 1;
        int end = cmd.indexOf(')');
        if (start < 1 || end < 0) {
            Serial.println("Chybný formát back_buttons");
            return;
        }
        String paramStr = cmd.substring(start, end);
        if (!parseParams(paramStr, params, 1)) {
            Serial.println("Chyba v parametrech back_buttons");
            return;
        }
        back_buttons(params[0]);
        Serial.println("back_buttons zavoláno");
        return;
    }
    Serial.println("Neznámý příkaz");
}

void setup() {
    Serial.begin(115200);
    Serial.println("Zadej příkaz pro motor:");
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim(); // odstraní CR/LF a mezery navíc
        if (line.length() > 0) {
            Serial.print("Dostal jsem řetězec: ");
            Serial.println(line);
            processCommand(line);
        }
    }
}
/*
encodery()
forward(100, 50)
forward(-50, 30)
radius_r(90, 100, 40)
radius_r(-45, 80, 30)
radius_l(90, 100, 40)
radius_l(-45, 80, 30)
turn_on_spot(180)
turn_on_spot(-90)
back_buttons(40)
*/
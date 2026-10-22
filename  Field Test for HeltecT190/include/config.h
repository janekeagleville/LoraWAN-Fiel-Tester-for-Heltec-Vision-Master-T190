#pragma once
#include <stdint.h>
// Heltec Vision Master T190 (ESP32-S3R8, SX1262, TFT ST7789 170x320).
// Piny wg definicji wariantu Meshtastic "heltec_vision_master_t190" (variant.h). Piny TFT sa w platformio.ini (build_flags).
constexpr int PIN_BTN_A=0;      // PRG / BOOT (przytrzymany przy resecie wchodzi w tryb wgrywania)
constexpr int PIN_BTN_B=21;     // drugi przycisk na plytce (KEY)
constexpr int PIN_TFT_PWR=7;    // zasilanie TFT (VTFT_CTRL); aktywny poziom ustawiany w NVS, domyslnie LOW
constexpr int PIN_TFT_BL=17;    // podswietlenie (VTFT_LEDA), HIGH = wlaczone
constexpr int PIN_VEXT=5;       // zasilanie zlacza czujnikow (VEXT_ENABLE), HIGH = wlaczone
constexpr int PIN_BAT_ADC=6, PIN_BAT_CTRL=46;   // pomiar baterii, PIN_BAT_CTRL HIGH = dzielnik wlaczony
constexpr float BAT_MULT=4.9f*1.03f;             // mnoznik dzielnika wg Meshtastic (ADC 2,5 dB)
constexpr int PIN_LORA_CS=8, PIN_LORA_SCK=9, PIN_LORA_MOSI=10, PIN_LORA_MISO=11, PIN_LORA_RST=12, PIN_LORA_BUSY=13, PIN_LORA_DIO1=14;
constexpr float LORA_TCXO_V=1.8f;                // TCXO zasilany z DIO3
constexpr int SCR_W=320, SCR_H=170;

// ---- Test lacza (wlasny protokol, NIE Meshtastic) ----
constexpr uint8_t LINK_SYNC=0x12;                // prywatne slowo synchronizacji: Meshtastic (0x2B) nie dekoduje tych ramek
constexpr uint16_t LINK_PREAMBLE=8;
constexpr uint32_t REPLY_DELAY_MS=80;            // odpowiadajacy czeka, zeby inicjator zdazyl przejsc z TX na RX
constexpr unsigned LINK_HIST=60;                 // liczba ostatnich pomiarow na wykresie
// ---- Podsluch Meshtastic ----
constexpr uint8_t MESH_SYNC=0x2B;
constexpr uint16_t MESH_PREAMBLE=16;
constexpr unsigned MESH_MAX_NODES=40;
// ---- Pomiar szumu ----
constexpr uint32_t NOISE_PERIOD_MS=100;          // chwilowy RSSI 10 razy na sekunde
constexpr int NOISE_MIN=-130, NOISE_MAX=-40;     // zakres wykresow [dBm]
// ---- Skan pasma ----
constexpr float SCAN_START=863.0f, SCAN_STEP=0.1f;
constexpr unsigned SCAN_N=71;                    // 863.0 ... 870.0 MHz

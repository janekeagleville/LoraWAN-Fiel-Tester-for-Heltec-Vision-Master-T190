# LoRa Field Test — Heltec Vision Master T190

Samodzielny tester zasięgu LoRa na płytce **Heltec Vision Master T190** (ESP32-S3, SX1262, ekran TFT 1,9" 170×320, dwa przyciski, bez dotyku i bez GPS).
To **własny firmware, nie Meshtastic**: płytka sama obsługuje radio. Nie łączy się z CYD ani z Trackerem i nie ma mapy, czatu ani listy węzłów Meshtastic.

> Uwaga: firmware został napisany i przetestowany na hoście (symulacja radia i ekranu, ASan/UBSan). **Nie był kompilowany dla ESP32 ani uruchamiany na płytce**, więc pierwsze wgranie może wymagać drobnych poprawek.

## Co potrafi

| Ekran | Opis |
|---|---|
| HOME | kafelki: test łącza, sygnał, szum, zajętość kanału, parametry radia, bateria |
| LINK | ping-pong dwóch T190: RSSI i SNR w **obu kierunkach**, ocena GOOD / MARGINAL / WEAK względem granicy demodulacji dla SF, wykres, straty pakietów, RTT |
| NOISE | **prawdziwy pomiar szumu** (chwilowy RSSI radia 10×/s): teraz, podłoga, szczyt, zajętość, wykres z oknami 1 min / 10 min / 1 h |
| SCAN | przemiatanie pasma 863–870 MHz (krok 100 kHz) z zapamiętaniem szczytów; zaznaczone częstotliwości testu i Meshtastic |
| MESH | podsłuch pakietów Meshtastic (nagłówki jawne): nadawca, RSSI, SNR, hopy, wiek; nazwy węzłów tylko z domyślnym kluczem kanału; **ping wybranego węzła** z pomiarem ACK |
| SETUP | rola, preset, częstotliwość, moc, interwał, preset Meshtastic, odwrócenie kolorów, obrót ekranu, polaryzacja zasilania TFT; zapis w NVS |

### Test łącza (dwie płytki)
1. Obie płytki muszą mieć te same ustawienia presetu i częstotliwości.
2. Na jednej ustaw rolę **INIT**, na drugiej **ECHO** (SETUP → Role).
3. INIT wysyła ramkę PROBE co 3/5/10/30 s; ECHO odpowiada ramką REPLY z RSSI/SNR, z jakimi usłyszał PROBE. INIT zna więc jakość w obu kierunkach.
4. Domyślnie: MediumFast (SF9, BW 250 kHz, CR 4/5), 869,525 MHz, 14 dBm, co 5 s. Ramka testowa ma 12 bajtów i słowo synchronizacji 0x12, więc Meshtastic jej nie dekoduje i sieć nie jest zakłócana treścią.
5. Ocena: margines SNR względem granicy (SF7 −7,5 dB, każdy kolejny SF o 2,5 dB mniej): ≥10 dB GOOD, ≥3 dB MARGINAL, niżej WEAK.

### Ping węzła Meshtastic (ekran MESH)
Test działa z dowolnym węzłem Meshtastic widocznym na liście, bez drugiego T190.
1. KEY (krótko) wybiera węzeł z listy, KEY (długo) wysyła ping.
2. T190 wysyła do węzła pakiet z żądaniem potwierdzenia (want_ack), z własnym losowym identyfikatorem (zapisanym na stałe w pamięci, widocznym w logu szeregowym), hop_limit 0 (test bezpośredni, bez przekazywania przez inne węzły) i portem PRIVATE_APP. Węzeł docelowy nie wyświetla takiej wiadomości, ale odsyła ACK.
3. Panel na dole pokazuje: ACK z RSSI, SNR, czasem odpowiedzi i liczbą hopów odpowiedzi, albo NAK, albo brak ACK po 12 s. Liczniki wysłanych i potwierdzonych pingów są osobne dla każdego węzła.
4. Ograniczenia: działa tylko dla kanału z **domyślnym kluczem** (AQ==) i nazwą presetu zgodną z ustawieniem „Mesh preset”. Dla innego klucza pakiet jest dla węzła nieczytelny i ACK nie przyjdzie. Ping to nadawanie w sieci (moc i limit wypełnienia jak w ustawieniach), więc używaj go oszczędnie. Pomiar dotyczy drogi tam (pakiet dotarł) i z powrotem (ACK dotarł): RSSI i SNR dotyczą ACK słyszanego u Ciebie.

### Limity EU868 (pilnowane przez firmware)
- 869,4–869,65 MHz: wypełnienie 10 %; 868,0–868,6 MHz: 1 % i maks. 14 dBm; pozostałe podpasma 0,1 %.
- Firmware liczy czas nadawania z ostatniej godziny i wstrzymuje wysyłanie po przekroczeniu limitu (na HOME pojawia się DUTY LIMIT). Moc jest przycinana do limitu pasma.

### Szum
Szum to chwilowy RSSI radia w trybie odbioru, agregowany do sekundowych kubełków (średnia, min, max, zajętość) na godzinę. „Podłoga” to 10. percentyl minimów z okna; „zajętość” to odsetek odczytów powyżej podłogi + 8 dB. Pomiar nie jest wykonywany podczas własnego nadawania i na ekranie SCAN.

## Obsługa (dwa przyciski)
| | PRG (GPIO0) | KEY (GPIO21) |
|---|---|---|
| zwykłe ekrany | krótko: następny ekran, długo: poprzedni | krótko: akcja ekranu (pauza / okno czasu / pauza skanu / następny węzeł), długo: reset, wyczyszczenie lub ping (MESH) |
| SETUP | krótko: następna pozycja, długo: wyjście | krótko: zmiana wartości, długo: poprzednia pozycja |

Przytrzymanie **KEY podczas włączania** odwraca polaryzację zasilania TFT (GPIO7). Użyj tego, jeśli ekran pozostaje czarny.

## Budowanie
```
pio run -e t190 -t upload       # kolory RGB
pio run -e t190_bgr -t upload   # jeśli czerwony i niebieski są zamienione
pio device monitor             # log CSV co 5 s
```
Zależności (RadioLib 7.1.2, TFT_eSPI 2.5.43, platforma espressif32 6.9.0) są przypięte w `platformio.ini`. Płytka zgłasza się jako `esp32-s3-devkitc-1` z 8 MB flash i PSRAM OPI. Do wgrania może być potrzebne wejście w tryb bootloadera (przytrzymaj PRG, naciśnij RST).

## Piny (z definicji wariantu Meshtastic)
TFT ST7789: CS 39, DC 47, MOSI 48, SCK 38, RST 40, podświetlenie 17, zasilanie TFT 7, VEXT 5. LoRa SX1262: CS 8, SCK 9, MOSI 10, MISO 11, RST 12, BUSY 13, DIO1 14, TCXO 1,8 V, DIO2 jako przełącznik RF. Bateria: ADC 6, sterowanie dzielnikiem 46. Przyciski: 0 i 21.

## Do sprawdzenia na sprzęcie
- Polaryzacja zasilania TFT (GPIO7) i ewentualny wariant BGR / odwrócenie kolorów (SETUP).
- Mnożnik napięcia baterii (4,9 × 1,03) jest wzięty z Meshtastic; wskazanie procentowe jest orientacyjne.
- Przesunięcie 170-pikselowego ST7789 (`CGRAM_OFFSET`) — gdyby obraz był przesunięty, zmień rotację w SETUP (Flip screen).
- Nazwy węzłów w MESH pojawiają się tylko dla kanału z domyślnym kluczem i po odebraniu pakietu NODEINFO.

## Testy na hoście
```
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/units.cpp -o units && ./units
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/sim.cpp -o sim && mkdir -p shots && ./sim && python3 tools/render.py shots
```
`units` sprawdza AES (FIPS-197, tryb CTR zgodny z biblioteką Pythona), nagłówki Meshtastic, protokół łącza, szum i limity wypełnienia. `sim` uruchamia cały firmware z wirtualnym SX1262, przyciskami i zegarem; `render.py` składa podglądy ekranów (czcionki zastępcze).

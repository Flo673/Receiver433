#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ==== LoRa Setup (Adafruit Feather M0 LoRa / RFM95, antenna on U.FL) ====
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define LORA_FREQUENCY 433E6  // must match the transmitter

// Board 4 (Transmitter433): bare RSSI-test transmitter, no sensors -- just a
// packet counter + battery voltage so packets are easy to count for loss/RSSI checks.
#define BOARD4_FIELD_COUNT 2
// Board 5 (Transmitter433wBno): the same two fields plus an SHT4x sample --
// temperature and relative humidity.
#define BOARD5_FIELD_COUNT 4

// Field offsets within a board 5 packet, after the board id has been split off.
#define B5_PACKET_COUNT 0
#define B5_BATTERY 1
#define B5_TEMPERATURE 2
#define B5_HUMIDITY 3

// ==== Commanding the transmitters ====
// The transmitters boot into LISTEN mode (START_IN_LISTEN_MODE) and stay asleep until
// they hear a START, so with only this board plugged into the PC it is this board that
// has to wake them. Type START or STOP into the serial monitor; "0" targets every board.
#define CMD_TARGET_BOARD "0"

// A sleeping transmitter only listens for LISTEN_MS out of every SLEEP_MS, so a single
// START would almost always land while it is asleep. Repeating back-to-back for longer
// than one full sleep+listen cycle (5000 + 250 ms over there) guarantees one copy lands
// inside a listen window. Raise this if you raise SLEEP_MS on the transmitter.
#define START_BURST_MS 6000
// A transmitting board listens for the whole gap between packets, so STOP needs far less.
#define STOP_BURST_MS 2000

// Below 525 MHz the SX1276 receives on the LF port, so the LoRa library subtracts
// 164 from PktRssi instead of the 157 it uses at 915 MHz -- add back what it took.
#define RSSI_OFFSET_LF_PORT 164

// The LoRa library's packetRssi() is a simplified "PktRssi register - offset", which drops
// both terms the SX1276 datasheet (section 5.5.5) asks for: the 16/15 scaling when
// SNR >= 0, and the SNR term when SNR < 0. That is only worth a few dB, but when the
// whole point of a packet is comparing antennas, report the datasheet number too.
// rawPktRssi recovers the register value the library already subtracted the offset from.
float datasheetRssi(int libraryRssi, float snr) {
  int rawPktRssi = libraryRssi + RSSI_OFFSET_LF_PORT;
  if (snr < 0) {
    return -RSSI_OFFSET_LF_PORT + rawPktRssi + snr;
  }
  return -RSSI_OFFSET_LF_PORT + (16.0f / 15.0f) * rawPktRssi;
}

// Splits s on commas into up to maxFields entries of fields[], and returns the
// total number of fields found (which may exceed maxFields if s has more
// commas than expected -- callers should check the count before using fields).
int splitCsv(const String &s, String *fields, int maxFields) {
  int count = 0;
  int start = 0;
  while (true) {
    int comma = s.indexOf(',', start);
    if (comma == -1) {
      if (count < maxFields) fields[count] = s.substring(start);
      count++;
      break;
    }
    if (count < maxFields) fields[count] = s.substring(start, comma);
    count++;
    start = comma + 1;
  }
  return count;
}

// Repeats "CMD,<target>,<verb>" back-to-back for burstMs so one copy lands inside a
// sleeping transmitter's listen window. Nothing is received while this runs.
void sendCommandBurst(const char *verb, uint32_t burstMs) {
  String packet = String("CMD," CMD_TARGET_BOARD ",") + verb;

  Serial.print(F("# sending "));
  Serial.print(packet);
  Serial.print(F(" for "));
  Serial.print(burstMs);
  Serial.println(F(" ms"));

  uint32_t start = millis();
  while (millis() - start < burstMs) {
    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();  // blocks until the packet is on the air
  }

  Serial.println(F("# burst done, listening again"));
}

// Non-blocking read of one newline-terminated command typed into the serial monitor.
void pollSerialCommand() {
  static String line;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      line += c;
      continue;
    }

    line.trim();
    line.toUpperCase();
    if (line == "START") {
      sendCommandBurst("START", START_BURST_MS);
    } else if (line == "STOP") {
      sendCommandBurst("STOP", STOP_BURST_MS);
    } else if (line.length() > 0) {
      Serial.print(F("# unknown command: "));
      Serial.println(line);
    }
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  while (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed, retrying...");
    delay(1000);
  }

  Serial.println("# 433 MHz RSSI-test receiver");
  Serial.println("# board 4: board_id,packet_count,battery_voltage,rssi,snr,rssi_datasheet,noise_floor");
  Serial.println("# board 5: board 4 fields + temperature_c,humidity_pct");
  Serial.println("# type START to wake the transmitters, STOP to put them back to sleep");
}

void loop() {
  pollSerialCommand();

  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return;
  }

  String payload;
  while (LoRa.available()) {
    payload += (char)LoRa.read();
  }

  // Payload format: "<board_id>,<board-specific fields...>" -- board_id selects which
  // fixed-width schema the rest of the payload is parsed as.
  int firstComma = payload.indexOf(',');
  if (firstComma == -1) {
    return;
  }
  String boardId = payload.substring(0, firstComma);
  String rest = payload.substring(firstComma + 1);

  long rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (boardId == "4") {
    String fields[BOARD4_FIELD_COUNT];
    if (splitCsv(rest, fields, BOARD4_FIELD_COUNT) != BOARD4_FIELD_COUNT) {
      return;
    }

    long packetCount = fields[0].toInt();
    float batteryVoltage = fields[1].toFloat();

    // Ambient channel level now that the packet is out of the FIFO. If this sits only
    // a few dB below the packet RSSI, the link is noise-limited; if the packet RSSI is
    // implausibly low while the noise floor is far below it, suspect the RF path
    // (U.FL seating, coax, antenna placement) rather than the radio config.
    int noiseFloor = LoRa.rssi();

    Serial.print(boardId);
    Serial.print(",");
    Serial.print(packetCount);
    Serial.print(",");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.print(snr, 2);
    Serial.print(",");
    Serial.print(datasheetRssi(rssi, snr), 1);
    Serial.print(",");
    Serial.println(noiseFloor);
  } else if (boardId == "5") {
    String fields[BOARD5_FIELD_COUNT];
    if (splitCsv(rest, fields, BOARD5_FIELD_COUNT) != BOARD5_FIELD_COUNT) {
      return;
    }

    long packetCount = fields[B5_PACKET_COUNT].toInt();
    float batteryVoltage = fields[B5_BATTERY].toFloat();
    int noiseFloor = LoRa.rssi();

    Serial.print(boardId);
    Serial.print(",");
    Serial.print(packetCount);
    Serial.print(",");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.print(snr, 2);
    Serial.print(",");
    Serial.print(datasheetRssi(rssi, snr), 1);
    Serial.print(",");
    Serial.print(noiseFloor);
    for (int i = B5_TEMPERATURE; i < BOARD5_FIELD_COUNT; i++) {
      Serial.print(",");
      Serial.print(fields[i]);
    }
    Serial.println();
  }
  // Unknown board_id: silently ignore.
}

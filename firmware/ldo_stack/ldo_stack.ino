// LDO stack controller firmware for the Seeed XIAO nRF52840 (Arduino, Seeeduino:nrf52 core).
// Step 1: answers one-line text commands over USB serial and over BLE (Nordic UART Service).
//   ping  -> pong
//   scan  -> scan <addr> <addr> ...   (I2C addresses in hex that acknowledged)
//   r <addr> <reg>        -> r <addr> <reg> <value>   (16-bit register read, all hex)
//   w <addr> <reg> <value> -> ok                       (16-bit register write, all hex)
//   set <dac> <mV>        -> set <dac> <mV> <code>    (LDO output target at J3; dac address hex, target after clamping)
//   read <ina>            -> read <ina> <uV> <uA>      (output voltage and load current from the INA226)
// Check "read" and "set" before "r"/"w": sscanf("read 41", "r %x %x") would parse "ead" as hex.
// Both chips on the LDO board (DAC60502, INA226) use an 8-bit register pointer and 16-bit data, MSB first.
// I2C runs on D4 (SDA) / D5 (SCL); the pull-ups are on the controller board.
#include <Wire.h>
#include <bluefruit.h>

BLEUart bleuart;

// DAC60502: at VDD = 3.3 V the 2.5 V reference must be halved (REF-DIV) or REF-ALARM holds the outputs at 0 V.
const uint8_t DAC_GAIN = 0x04, DAC_A = 0x08;
const uint16_t GAIN_REFDIV_X2 = 0x0103;  // REF-DIV = 1, both buffers x2 -> 0-2.5 V full scale
const uint8_t DAC_FIRST = 0x48, DAC_LAST = 0x4B;
// LT3042: OUT follows SET, and SET = DAC output + 100 uA x R2 (1 kOhm).
const int SET_OFFSET_MV = 100;
// INA226: bus voltage LSB 1.25 mV; shunt LSB 2.5 uV across 0.5 Ohm = 5 uA.
const uint8_t INA_SHUNT = 0x01, INA_BUS = 0x02;
const int INA_OFFSET = 8;  // each board's INA226 sits 8 below its DAC (0x48 -> 0x40)
const int MIN_MV = 100, MAX_MV = 2600;

// The shunt R4 (0.5 Ohm) sits after the LT3042's sense point, so J3 sags 0.5 mV per mA.
// For every board with a target, the DAC is raised by the measured shunt drop every 100 ms.
const uint32_t SAG_PERIOD_MS = 100;
int targetMv[4] = {-1, -1, -1, -1};  // per DAC 0x48-0x4B; -1 = no target set
int dacCode[4] = {-1, -1, -1, -1};   // last code written

// Collects characters from a stream until a line ending.
struct LineReader {
  char buf[64];
  uint8_t len = 0;

  // Returns true when a complete, non-empty line is in buf.
  bool feed(Stream &s) {
    while (s.available()) {
      char c = s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) continue;
        buf[len] = 0;
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) buf[len++] = c;
    }
    return false;
  }
};

LineReader usbLine, bleLine;

void cmdScan(Stream &out) {
  out.print("scan");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      char hex[4];
      snprintf(hex, sizeof(hex), " %02X", addr);
      out.print(hex);
    }
  }
  out.println();
}

bool regWrite(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val >> 8);
  Wire.write(val & 0xFF);
  return Wire.endTransmission() == 0;
}

bool regRead(uint8_t addr, uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
  val = Wire.read() << 8;
  val |= Wire.read();
  return true;
}

// Writes the DAC code for slot i's target plus the shunt drop at the present load current.
// Returns false if the DAC did not acknowledge.
bool applyTarget(int i) {
  uint8_t dac = DAC_FIRST + i;
  uint16_t shunt;
  long ua = regRead(dac - INA_OFFSET, INA_SHUNT, shunt) ? (long)(int16_t)shunt * 5 : 0;
  // DAC output in uV = target - SET offset + I x 0.5 Ohm
  int64_t uv = (int64_t)(targetMv[i] - SET_OFFSET_MV) * 1000 + ua / 2;
  int code = constrain((uv * 4096 + 1250000) / 2500000, 0, 4095);
  if (code == dacCode[i]) return true;
  if (!regWrite(dac, DAC_A, code << 4)) return false;
  dacCode[i] = code;
  return true;
}

void cmdSet(Stream &out, uint8_t dac, int mv) {
  if (dac < DAC_FIRST || dac > DAC_LAST) {
    out.println("err dac address");
    return;
  }
  int i = dac - DAC_FIRST;
  targetMv[i] = constrain(mv, MIN_MV, MAX_MV);
  dacCode[i] = -1;  // force a write
  if (!regWrite(dac, DAC_GAIN, GAIN_REFDIV_X2) || !applyTarget(i)) {
    targetMv[i] = -1;
    out.println("err i2c");
    return;
  }
  char line[40];
  snprintf(line, sizeof(line), "set %02X %d %d", dac, targetMv[i], dacCode[i]);
  out.println(line);
}

void cmdRead(Stream &out, uint8_t ina) {
  uint16_t bus, shunt;
  if (!regRead(ina, INA_BUS, bus) || !regRead(ina, INA_SHUNT, shunt)) {
    out.println("err i2c");
    return;
  }
  char line[40];
  snprintf(line, sizeof(line), "read %02X %ld %ld", ina, (long)bus * 1250, (long)(int16_t)shunt * 5);
  out.println(line);
}

void handle(Stream &out, const char *cmd) {
  unsigned addr, reg, val;
  int mv;
  char line[32];
  if (strcmp(cmd, "ping") == 0) {
    out.println("pong");
  } else if (strcmp(cmd, "scan") == 0) {
    cmdScan(out);
  } else if (sscanf(cmd, "read %x", &addr) == 1) {
    cmdRead(out, addr);
  } else if (sscanf(cmd, "set %x %d", &addr, &mv) == 2) {
    cmdSet(out, addr, mv);
  } else if (sscanf(cmd, "r %x %x", &addr, &reg) == 2) {
    uint16_t v;
    if (regRead(addr, reg, v)) {
      snprintf(line, sizeof(line), "r %02X %02X %04X", addr, reg, v);
      out.println(line);
    } else {
      out.println("err i2c");
    }
  } else if (sscanf(cmd, "w %x %x %x", &addr, &reg, &val) == 3) {
    out.println(regWrite(addr, reg, val) ? "ok" : "err i2c");
  } else {
    out.print("err unknown command: ");
    out.println(cmd);
  }
}

void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // fast, then slow (units of 0.625 ms)
  Bluefruit.Advertising.setFastTimeout(30);    // seconds of fast advertising
  Bluefruit.Advertising.start(0);              // advertise forever
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  // Give every DAC its 0-2.5 V range at boot; absent addresses just don't acknowledge.
  for (uint8_t a = DAC_FIRST; a <= DAC_LAST; a++) regWrite(a, DAC_GAIN, GAIN_REFDIV_X2);

  Bluefruit.autoConnLed(true);  // blue LED shows the BLE connection
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("LDO-Stack");
  bleuart.begin();
  startAdvertising();
}

void loop() {
  if (usbLine.feed(Serial)) handle(Serial, usbLine.buf);
  if (bleLine.feed(bleuart)) handle(bleuart, bleLine.buf);

  static uint32_t lastSag = 0;
  if (millis() - lastSag >= SAG_PERIOD_MS) {
    lastSag = millis();
    for (int i = 0; i < 4; i++)
      if (targetMv[i] >= 0) applyTarget(i);
  }
}

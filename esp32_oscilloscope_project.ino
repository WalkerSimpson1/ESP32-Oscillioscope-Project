#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pins (from schematic) ----------------
#define I2C_SDA   21
#define I2C_SCL   22
#define CH1_PIN   34   // ADC1_CH6, input-only pin
#define CH2_PIN   35   // ADC1_CH7, input-only pin

// ---------------- Display settings ----------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED1_ADDR    0x3C   // U2 -> CH1
#define OLED2_ADDR    0x3D   // U3 -> CH2
#define TEXT_H        10     // top rows reserved for the text line

// ---------------- Scope settings ----------------
#define CAPTURE_LEN    256   // samples per capture (2x screen width, room to trigger)
#define TARGET_CYCLES  3     // aim to show about this many cycles on screen
#define MAX_DELAY_US   2000  // slowest timebase (extra wait per sample)
#define MIN_SIGNAL     60    // ADC counts (~50 mV); smaller = treated as no signal
#define MIN_SPAN       200   // smallest vertical range when auto-scaling
#define AUTO_SCALE     true  // false = fixed 0..3.3 V vertical scale

Adafruit_SSD1306 oled1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 oled2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct Channel {
  const char       *name;
  uint8_t           pin;
  uint8_t           addr;
  Adafruit_SSD1306 *disp;
  bool              present;
  int               delayUs;          // extra delay between samples
  float             samplePeriodUs;   // measured time per sample
  float             freq;             // Hz, 0 = unknown
  int               minV, maxV;       // raw ADC min / max of capture
  int               trigIndex;        // where the screen starts in buf[]
  uint16_t          buf[CAPTURE_LEN];
};

Channel ch[2] = {
  { "CH1", CH1_PIN, OLED1_ADDR, &oled1, false, 20 },
  { "CH2", CH2_PIN, OLED2_ADDR, &oled2, false, 20 },
};

// ---------------------------------------------------------
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);          // fast I2C = faster screen refresh

  analogReadResolution(12);
  for (auto &c : ch) {
    pinMode(c.pin, INPUT);
    analogSetPinAttenuation(c.pin, ADC_11db);   // ~0..3.3 V range

     analogWrite(25, 128);   // 1 kHz square wave on D25
  }

  // Report everything on the I2C bus (handy for checking addresses)
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cDevicePresent(a)) Serial.printf("  found device at 0x%02X\n", a);
  }

  for (auto &c : ch) {
    c.present = i2cDevicePresent(c.addr) &&
                c.disp->begin(SSD1306_SWITCHCAPVCC, c.addr);
    if (!c.present) {
      Serial.printf("%s: no OLED at 0x%02X - channel disabled\n", c.name, c.addr);
      continue;
    }
    c.disp->clearDisplay();
    c.disp->setTextColor(SSD1306_WHITE);
    c.disp->setTextSize(1);
    c.disp->setCursor(0, 0);
    c.disp->println("ESP32 MiniScope");
    c.disp->println(c.name);
    c.disp->display();
  }

  if (!ch[0].present && !ch[1].present) {
    Serial.println("No displays found. Check wiring and addresses.");
  }
  delay(1000);
}

void loop() {
  for (auto &c : ch) {
    if (!c.present) continue;
    capture(c);
    analyse(c);
    draw(c);
    adjustTimebase(c);
  }
}

// ---------------------------------------------------------
//  CAPTURE: fill the buffer and measure the real sample rate
// ---------------------------------------------------------
void capture(Channel &c) {
  unsigned long t0 = micros();
  for (int i = 0; i < CAPTURE_LEN; i++) {
    c.buf[i] = analogRead(c.pin);
    if (c.delayUs > 0) delayMicroseconds(c.delayUs);
  }
  unsigned long t1 = micros();
  // analogRead itself takes ~10 us, so measure instead of assuming
  c.samplePeriodUs = (float)(t1 - t0) / CAPTURE_LEN;

  c.minV = 4095;
  c.maxV = 0;
  for (int i = 0; i < CAPTURE_LEN; i++) {
    if (c.buf[i] < c.minV) c.minV = c.buf[i];
    if (c.buf[i] > c.maxV) c.maxV = c.buf[i];
  }
}

// ---------------------------------------------------------
//  ANALYSE: find rising edges around the signal's own midpoint
//  (works whether the signal is centred at 0 V or at 1.65 V)
// ---------------------------------------------------------
void analyse(Channel &c) {
  c.freq = 0;
  c.trigIndex = 0;

  int span = c.maxV - c.minV;
  if (span < MIN_SIGNAL) return;          // flat line / noise only

  int mid  = (c.maxV + c.minV) / 2;
  int hyst = max(10, span / 10);

  bool armed = false;
  int firstEdge = -1, lastEdge = -1, edges = 0;

  for (int i = 0; i < CAPTURE_LEN; i++) {
    if (c.buf[i] < mid - hyst) armed = true;
    else if (armed && c.buf[i] >= mid + hyst) {
      armed = false;
      if (firstEdge < 0) firstEdge = i;
      lastEdge = i;
      edges++;
    }
  }

  // Trigger: start the screen on the first rising edge if it fits
  if (firstEdge >= 0 && firstEdge <= CAPTURE_LEN - SCREEN_WIDTH) {
    c.trigIndex = firstEdge;
  }

  if (edges >= 2 && lastEdge > firstEdge) {
    float cycles  = edges - 1;
    float time_us = (lastEdge - firstEdge) * c.samplePeriodUs;
    c.freq = cycles * 1e6f / time_us;
  }
}

// ---------------------------------------------------------
//  AUTO TIMEBASE: show about TARGET_CYCLES cycles on screen
// ---------------------------------------------------------
void adjustTimebase(Channel &c) {
  float overhead = c.samplePeriodUs - c.delayUs;   // time analogRead takes

  if (c.freq > 0) {
    float wanted = (1e6f / c.freq) * TARGET_CYCLES / SCREEN_WIDTH;
    int newDelay = (int)(wanted - overhead);
    c.delayUs = constrain(newDelay, 0, MAX_DELAY_US);
  } else if (c.maxV - c.minV >= MIN_SIGNAL) {
    // Signal present but fewer than 2 edges seen: slow down to find them
    c.delayUs = min(c.delayUs * 2 + 1, MAX_DELAY_US);
  }
}

// ---------------------------------------------------------
//  DRAW
// ---------------------------------------------------------
void draw(Channel &c) {
  Adafruit_SSD1306 &d = *c.disp;
  d.clearDisplay();

  const int top = TEXT_H, bottom = SCREEN_HEIGHT - 1;

  // Dotted graticule: centre line + 4 vertical divisions
  int yMid = (top + bottom) / 2;
  for (int x = 0; x < SCREEN_WIDTH; x += 4) d.drawPixel(x, yMid, SSD1306_WHITE);
  for (int gx = 32; gx < SCREEN_WIDTH; gx += 32)
    for (int y = top; y <= bottom; y += 4) d.drawPixel(gx, y, SSD1306_WHITE);

  // Vertical scale
  int lo = 0, hi = 4095;
  if (AUTO_SCALE) {
    int span = max(c.maxV - c.minV, MIN_SPAN);
    int centre = (c.maxV + c.minV) / 2;
    lo = centre - span * 6 / 10;          // 10% margin top and bottom
    hi = centre + span * 6 / 10;
  }

  // Waveform
  for (int x = 0; x < SCREEN_WIDTH - 1; x++) {
    int i = c.trigIndex + x;
    int y1 = map(c.buf[i],     lo, hi, bottom, top);
    int y2 = map(c.buf[i + 1], lo, hi, bottom, top);
    y1 = constrain(y1, top, bottom);
    y2 = constrain(y2, top, bottom);
    d.drawLine(x, y1, x + 1, y2, SSD1306_WHITE);
  }

  // Text line: "CH1 440.0Hz 1.23Vpp"
  d.setTextSize(1);
  d.setCursor(0, 0);
  d.print(c.name);
  d.print(' ');
  if (c.freq <= 0)            d.print("--Hz");
  else if (c.freq >= 1000.0f) { d.print(c.freq / 1000.0f, 2); d.print("kHz"); }
  else                        { d.print(c.freq, 1); d.print("Hz"); }

  float vpp = (c.maxV - c.minV) * 3.3f / 4095.0f;   // approximate
  d.print(' ');
  d.print(vpp, 2);
  d.print("Vpp");

  d.display();
}

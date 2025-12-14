#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <util/delay.h>
#include <DHT.h>
#include <Stepper.h>

// States
enum SystemState {
 DISABLED,
 IDLE,
 ERROR,
 RUNNING
};

// Libraries
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;
bool rtcAvailable = false;
LiquidCrystal lcd(22, 23, 3, 4, 5, 6);

// Settings
static const uint8_t WATER_ADC = 0;
static const uint8_t SAMPLES = 10;
static const uint16_t WATER_LOW_THRES_THRESHOLD = 200;

// Temperature Range
static const float COLD_THRESH = 18.0;
static const float HOT_THRESH = 22.0;

// DHT and LCD Refresh Rates
static const uint16_t TEMP_INTERVAL_MS = 60000;
static const uint32_t LCD_INTERVAL_MS = 60000;  // Changed to 2000ms in demo to save time

// Reset
#define RESET_DDR DDRC
#define RESET_PORT PORTC
#define RESET_PINR PINC
#define RESET_BIT PC2

// Mapping
#define START_DDR DDRE
#define START_PORT PORTE
#define START_PINR PINE
#define START_BIT PE4

// Vent
#define VENT_DDR DDRH
#define VENT_PORT PORTH
#define VENT_PINR PINH
#define FWD_BIT PH5
#define REV_BIT PH6

// Fan
#define FAN_DDR DDRC
#define FAN_PORT PORTC
#define FAN_BIT PC3

// LEDs
#define LED_DDR DDRC
#define LED_PORT PORTC
#define LED_YELLOW_THRES PC7 // D30
#define LED_GREEN PC6 // D31
#define LED_RED PC5 // D32
#define LED_BLUE PC1 // D36, these are all over the place because of the order i built it

// GPIO
#define SET_OUTPUT(ddr, bit) ((ddr) |=  (1U << (bit)))
#define SET_INPUT(ddr, bit) ((ddr) &= ~(1U << (bit)))
#define PULLUP_ON(port, bit) ((port) |=  (1U << (bit)))
#define WRITE_HIGH(port, bit) ((port) |=  (1U << (bit)))
#define WRITE_LOW_THRES(port, bit) ((port) &= ~(1U << (bit)))
#define READ_BIT(pinreg, bit) (((pinreg) & (1U << (bit))) ? 1 : 0)

static inline uint8_t buttonPressed(volatile uint8_t &pinreg, uint8_t bit) {
 return (READ_BIT(pinreg, bit) == 0);
}

// Timebase
volatile uint32_t gMillis = 0;
ISR(TIMER1_COMPA_vect) { gMillis++; }

static void timer() {
 TCCR1A = 0;
 TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
 OCR1A  = 249;
 TIMSK1 = (1 << OCIE1A);
}

static uint32_t nms() { //stands for now (ms)
 uint32_t ms;
 cli(); ms = gMillis; sei();
 return ms;
}

static void delayMs(uint16_t ms) {
 uint32_t start = nms();
 while ((uint32_t)(nms() - start) < ms) { }
}

// UART
static void uart_init_9600() {
 const uint16_t ubrr = 103; // 16MHz @ 9600
 UBRR0H = (uint8_t)(ubrr >> 8);
 UBRR0L = (uint8_t)(ubrr & 0xFF);
 UCSR0A = 0;
 UCSR0B = (1 << TXEN0);
 UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}
// Write single char to UART0
static void uart_write_char(char c) {
 while (!(UCSR0A & (1 << UDRE0))) { }  // Wait for TX buffer empty
 UDR0 = c;  // Send character
}

// Print string
static void uart_print(const char *s) {
 while (*s) uart_write_char(*s++);
}

// Print string with newline
static void uart_println(const char *s) {
 uart_print(s);
 uart_write_char('\r');
 uart_write_char('\n');
}

// Print unsigned 32-bit integer
static void uart_print_u32(uint32_t v) {
 char buf[11];
 uint8_t i = 0;
 if (v == 0) { 
   uart_write_char('0'); 
   return; 
 }
 // Build digits in reverse
 while (v && i < sizeof(buf) - 1) { 
   buf[i++] = (char)('0' + (v % 10)); 
   v /= 10; 
 }
 // Print digits in correct order
 while (i--) uart_write_char(buf[i]);
}

// Print signed 32-bit integer
static void uart_print_i32(int32_t v) {
 if (v < 0) { 
   uart_write_char('-'); 
   v = -v; 
 }
 uart_print_u32((uint32_t)v);
}

// Print timestamp (RTC or millis)
static void uart_printTimestamp() {
 if (rtcAvailable) {
   DateTime now = rtc.now();
   uart_print("Time: ");
   // YYYY-MM-DD HH:MM:SS
   uart_print_u32((uint32_t)now.year()); uart_write_char('-');
   if (now.month() < 10) uart_write_char('0');
   uart_print_u32((uint32_t)now.month()); uart_write_char('-');
   if (now.day() < 10) uart_write_char('0');
   uart_print_u32((uint32_t)now.day()); uart_write_char(' ');
   if (now.hour() < 10) uart_write_char('0');
   uart_print_u32((uint32_t)now.hour()); uart_write_char(':');
   if (now.minute() < 10) uart_write_char('0');
   uart_print_u32((uint32_t)now.minute()); uart_write_char(':');
   if (now.second() < 10) uart_write_char('0');
   uart_print_u32((uint32_t)now.second());
 } else {
   // Fallback
   uart_print("Time: RTC_MISSING ms=");
   uart_print_u32(nms());
 }
}

//  ADC
static void adc_init() {
 ADMUX  = (1 << REFS0); // AVcc ref
 ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // /128
 DIDR0 |= (1 << ADC0D);
}

static uint16_t adc_read(uint8_t channel) {
 ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
 ADCSRA |= (1 << ADSC);
 while (ADCSRA & (1 << ADSC)) { }
 return ADC;
}

// Stepper
static const int STEPS_PER_REV = 2048;
Stepper stepper(STEPS_PER_REV, 13, 11, 12, 10);
static int ventPosition = 0;

// ISR for Disabled
volatile uint8_t startIsrFlag = 0;
ISR(INT4_vect) { startIsrFlag = 1; }

static inline void startIntEnable()  { EIMSK |=  (1 << INT4); }
static inline void startIntDisable() { EIMSK &= ~(1 << INT4); }

// Global Variables
static SystemState currentState = ERROR; 
static bool fanIsOn = false;

static float tempC = 0.0f;
static float hum = 0.0f;
static bool haveTemp = false;
static bool haveHum = false;

static uint32_t lastTempUpdate = 0;
static uint32_t lastLCDUpdate = 0;

static uint8_t prevStopPressed = 0;
static uint32_t lastStopEventMs = 0;
static const uint16_t STOP_DEBOUNCE_MS = 250;

static uint32_t lastStartEventMs = 0;
static const uint16_t DEBOUNCE_MS = 300;

// Helpers
static const char* getStateName(SystemState s) {
 switch (s) {
   case DISABLED: return "DISABLED";
   case IDLE: return "IDLE";
   case ERROR: return "ERROR";
   case RUNNING: return "RUNNING";
   default: return "UNKNOWN";
 }
}
static void setStateLEDs(SystemState s) {
 WRITE_LOW_THRES(LED_PORT, LED_YELLOW_THRES);
 WRITE_LOW_THRES(LED_PORT, LED_GREEN);
 WRITE_LOW_THRES(LED_PORT, LED_RED);
 WRITE_LOW_THRES(LED_PORT, LED_BLUE);

 if (s == DISABLED) WRITE_HIGH(LED_PORT, LED_YELLOW_THRES);
 else if (s == IDLE) WRITE_HIGH(LED_PORT, LED_GREEN);
 else if (s == ERROR) WRITE_HIGH(LED_PORT, LED_RED);
 else if (s == RUNNING) WRITE_HIGH(LED_PORT, LED_BLUE);
}

static void logStateTransition(SystemState s) {
 uart_printTimestamp();
 uart_print(" - State transition to: ");
 uart_println(getStateName(s));
}

static void fan_set(bool on) {
 if (on == fanIsOn) return;
 fanIsOn = on;
 if (on) WRITE_HIGH(FAN_PORT, FAN_BIT);
 else WRITE_LOW_THRES(FAN_PORT, FAN_BIT);
 uart_printTimestamp();
 uart_print(" - FAN ");
 uart_println(on ? "ON" : "OFF");
}

static void lcdShow(float t, float h, bool tOk, bool hOk) {
 lcd.setCursor(0, 0);
 lcd.print("Temp:           ");
 lcd.setCursor(6, 0);

 if (tOk) { 
  lcd.print(t, 1); 
  lcd.print("C"); 
 }

 else lcd.print("ERR");
 lcd.setCursor(0, 1);
 lcd.print("Humidity:       ");
 lcd.setCursor(10, 1);

 if (hOk) { 
  lcd.print(h, 0); 
  lcd.print("%");
 }
 else lcd.print("ERR");
}

static void updateLCDForState() {
 switch (currentState) {
   case DISABLED:
     lcd.clear();
     lcd.print("System OFF");
     break;

   case ERROR:
     lcd.clear();
     lcd.setCursor(0, 0);
     lcd.print("Water level is");
     lcd.setCursor(0, 1);
     lcd.print("too low");
     break;

   case IDLE:
   case RUNNING:
     lcdShow(tempC, hum, haveTemp, haveHum);
     break;
 }
}

static void changeState(SystemState newState) {
 if (newState == currentState) return;
 currentState = newState;
 setStateLEDs(newState);
 logStateTransition(newState);
 updateLCDForState();
}

static uint8_t stopPressedEdge() {
 uint8_t pressed = buttonPressed(START_PINR, START_BIT);
 if (pressed && !prevStopPressed) {
   uint32_t ms = nms();
   if ((uint32_t)(ms - lastStopEventMs) > STOP_DEBOUNCE_MS) {
     lastStopEventMs = ms;
     prevStopPressed = pressed;
     return 1;
   }
 }

 prevStopPressed = pressed;
 return 0;
}

// Stop handling
static uint8_t handleStopNonDisabled() {
 startIntDisable();
 if (stopPressedEdge()) {
   fan_set(false);
   changeState(DISABLED);

   // Dont start immediately
   EIFR |= (1 << INTF4);
   startIsrFlag = 0;
   return 1;
 }
 return 0;
}

void setup() {
 cli();

 timer();
 uart_init_9600();
 adc_init();

 // Start/Stop
 SET_INPUT(START_DDR, START_BIT);
 PULLUP_ON(START_PORT, START_BIT);

 // Reset
 SET_INPUT(RESET_DDR, RESET_BIT);
 PULLUP_ON(RESET_PORT, RESET_BIT);

 // Fan 
 SET_OUTPUT(FAN_DDR, FAN_BIT);
 WRITE_LOW_THRES(FAN_PORT, FAN_BIT);

  // Vent
 SET_INPUT(VENT_DDR, FWD_BIT);
 PULLUP_ON(VENT_PORT, FWD_BIT);
 SET_INPUT(VENT_DDR, REV_BIT);
 PULLUP_ON(VENT_PORT, REV_BIT);

 // LED
 SET_OUTPUT(LED_DDR, LED_YELLOW_THRES);
 SET_OUTPUT(LED_DDR, LED_GREEN);
 SET_OUTPUT(LED_DDR, LED_RED);
 SET_OUTPUT(LED_DDR, LED_BLUE);

 EICRB = (EICRB & ~((1 << ISC40) | (1 << ISC41))) | (1 << ISC41);
 EIFR  |= (1 << INTF4);
 startIntEnable();

 sei();

 uart_println("System starting...");

 dht.begin();
 rtcAvailable = rtc.begin();
 uart_println(rtcAvailable ? "RTC works." : "RTC does not work.");

 lcd.begin(16, 2);
 lcd.clear();
 lcd.print("System Starting");
 delayMs(600);

 stepper.setSpeed(10);

 fan_set(false);
 changeState(DISABLED);
}

void loop() {
 if (currentState == DISABLED) { // start with isr
   fan_set(false);

   startIntEnable();

   if (startIsrFlag) {
     cli(); startIsrFlag = 0; sei();

     uint32_t ms = nms();
     if ((uint32_t)(ms - lastStartEventMs) > DEBOUNCE_MS) {
       lastStartEventMs = ms;

       // Prevent bounce
       startIntDisable();
       EIFR |= (1 << INTF4);

       changeState(IDLE);

       prevStopPressed = 1; // must release first
     }
   }
   return;
 }

 if (handleStopNonDisabled()) return;

 uint8_t fwd = buttonPressed(VENT_PINR, FWD_BIT);
 uint8_t rev = buttonPressed(VENT_PINR, REV_BIT);

 if (fwd && !rev) {
   stepper.step(1);
   ventPosition++;
   uart_printTimestamp();
   uart_print(" - Vent position: ");
   uart_print_i32(ventPosition);
   uart_println("");
 } else if (rev && !fwd) {
   stepper.step(-1);
   ventPosition--;
   uart_printTimestamp();
   uart_print(" - Vent position: ");
   uart_print_i32(ventPosition);
   uart_println("");
 }

// Water
 uint16_t waterLevel = adc_read(WATER_ADC);
 if (waterLevel < WATER_LOW_THRES_THRESHOLD) {
   fan_set(false);
   changeState(ERROR);
   return;
 }

 // Temp/Humidity
 uint32_t msNow = nms();
 if (lastTempUpdate == 0 || (uint32_t)(msNow - lastTempUpdate) >= TEMP_INTERVAL_MS) {
   lastTempUpdate = msNow;

   float t = dht.readTemperature();
   float h = dht.readHumidity();

   if (t == t) { 
    tempC = t; 
    haveTemp = true; 
   }
   if (h == h) { 
    hum = h; 
    haveHum = true; 
   }
 }

 // Error
 if (currentState == ERROR) {
   fan_set(false);

   if (handleStopNonDisabled()) return;

   // Return to idle
   if (buttonPressed(RESET_PINR, RESET_BIT)) {
     delayMs(50);
     if (buttonPressed(RESET_PINR, RESET_BIT)) {
       uint16_t waterLevel = adc_read(WATER_ADC);
       if (waterLevel >= WATER_LOW_THRES_THRESHOLD) {
         changeState(IDLE);
       } else {
         lcd.clear();
         lcd.setCursor(0, 0);
         lcd.print("STILL LOW WATER");
         delayMs(1500);
         updateLCDForState();
       }

       // Wait for reset release
       while (buttonPressed(RESET_PINR, RESET_BIT)) {
         if (handleStopNonDisabled()) return;
       }
     }
   }

   return;
 }

 
 // LCD
 if (lastLCDUpdate == 0 || (uint32_t)(msNow - lastLCDUpdate) >= LCD_INTERVAL_MS) {
   lastLCDUpdate = msNow;
   
   if (currentState == ERROR) {
      updateLCDForState();
    } else {
      lcdShow(tempC, hum, haveTemp, haveHum);
    }
 }

 // Fan
 if (haveTemp) {
   bool outOfRange = (tempC <COLD_THRESH) || (tempC > HOT_THRESH);
   if (outOfRange) {
     if (currentState != RUNNING) changeState(RUNNING);
     fan_set(true);
   } else {
     fan_set(false);
     if (currentState != IDLE) changeState(IDLE);
   }
 } else {
   fan_set(false);
   if (currentState != IDLE) changeState(IDLE);
 }
}
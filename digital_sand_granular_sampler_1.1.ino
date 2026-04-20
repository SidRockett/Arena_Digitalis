/* 
 * ARDUINO NANO R4 - MASTER GRANULAR WORKSTATION
 * 
 * MIN SIZE: 20 samples (~1.6ms) - Classic Microsound
 * OUTPUT: Pin A0 (12-bit DAC)
 */

#include "FspTimer.h"

#define SAMPLE_RATE 12000 
#define BUFFER_SIZE 3500 

#define DAC_PIN    A0  
#define ADC_PIN    A1  
#define SIZE_KNOB  A2  
#define SPEED_KNOB A3  
#define MIX_KNOB   A4  
#define TONE_KNOB  A5  
#define FREEZE_BUT 2   
#define RANDOM_BUT 4    
#define RANDOM_LED 6    
#define CLIP_LED   9     
#define FREEZE_LED 13  

uint16_t audioBuffer[BUFFER_SIZE];
volatile int writeHead = 0;
volatile float readHead = 0;
volatile float grainCounter = 0;

volatile float currentGrainSize = 500;
volatile float currentSpeed = 1.0;
volatile float currentMix = 0.5;
volatile float filterAlpha = 0.8;
volatile float envSoftness = 0.15;
volatile bool isReverse = false;
volatile float digitalGain = 1.5;

volatile float lfoPhase = 0;
volatile float lfoRate = 0; 
volatile int bitReduction = 0;
volatile int sampleHoldCount = 1;
volatile int downsampleCounter = 0;
volatile float lastOutput = 2048.0;
volatile uint16_t heldSample = 2048;

volatile bool isFrozen = false;
volatile bool isRandom = false; 
volatile int clipFlashTimer = 0; 

// FIX 2: ADC values read safely in loop(), consumed in ISR — analogRead() is not ISR-safe on R4
volatile uint16_t rawAudioIn   = 512;
volatile int      rawSizeKnob  = 512;
volatile int      rawSpeedKnob = 512;
volatile int      rawMixKnob   = 512;
volatile int      rawToneKnob  = 512;

// FIX 3: spray pre-computed in loop() — random() is not interrupt-safe
volatile int rawSpray = 0;

// FIX 5: clip LED driven from loop() via flag — no digitalWrite() inside ISR
volatile bool clipLedOn = false;

FspTimer audioTimer;
int scanner = 0;
float intervals[] = {0.5, 0.75, 1.0, 1.25, 1.5, 2.0};
unsigned long comboStartTime = 0;

void audioCallback(timer_callback_args_t __attribute((unused)) *p_args) {
  // FIX 2: read from volatile updated by loop() — no analogRead() in ISR
  // FIX 4: correct 10-bit→12-bit scaling (4095/1023 ≈ 4.004, not 4.0)
  float liveInput = (float)rawAudioIn * (4095.0f / 1023.0f);
  
  // 1. SCANNER (Knobs & Buttons)
  scanner++;
  if (scanner == 40) {
    int sVal = rawSizeKnob;  // FIX 2
    if (digitalRead(RANDOM_BUT) == LOW && digitalRead(FREEZE_BUT) == HIGH) 
      envSoftness = map(sVal, 0, 1023, 1, 500) / 1000.0;
    else 
      currentGrainSize = map(sVal, 0, 1023, 20, 1500); // REVERTED TO 20
  } else if (scanner == 80) {
    int spVal = rawSpeedKnob;  // FIX 2
    if (digitalRead(RANDOM_BUT) == LOW && digitalRead(FREEZE_BUT) == HIGH)
      currentSpeed = intervals[map(spVal, 0, 1023, 0, 5)];
    else currentSpeed = spVal / 512.0; 
  } else if (scanner == 120) currentMix = rawMixKnob / 1023.0;  // FIX 2
  else if (scanner == 160) {
    int tVal = rawToneKnob;  // FIX 2
    if (digitalRead(FREEZE_BUT) == LOW && digitalRead(RANDOM_BUT) == HIGH) 
      lfoRate = map(tVal, 0, 1023, 0, 100) / 1000.0;
    else {
      if (tVal > 600) { 
        filterAlpha = map(tVal, 600, 1023, 10, 1000) / 1000.0;
        bitReduction = 0; sampleHoldCount = 1; digitalGain = 1.5;
      } else {
        filterAlpha = 0.1; bitReduction = map(tVal, 0, 600, 8, 0);
        sampleHoldCount = map(tVal, 0, 600, 12, 1); digitalGain = 2.5; 
      }
    }
    scanner = 0;
  }

  // 2. ENGINE
  if (!isFrozen) {
    audioBuffer[writeHead] = (uint16_t)liveInput;
    writeHead = (writeHead + 1) % BUFFER_SIZE;
  }

  if (isReverse) readHead -= currentSpeed; else readHead += currentSpeed;
  grainCounter += currentSpeed;
  
  if (grainCounter >= currentGrainSize) {
    grainCounter = 0;
    int spray = rawSpray;  // FIX 3: random() is not ISR-safe; value computed in loop()
    readHead = isReverse ? (float)writeHead - 20 + spray : (float)writeHead - currentGrainSize - 30 + spray;
  }
  while (readHead < 0) readHead += BUFFER_SIZE; while (readHead >= BUFFER_SIZE) readHead -= BUFFER_SIZE;

  // 3. ENVELOPE (Windowing)
  float env = 1.0;
  float windowSize = currentGrainSize * envSoftness; 
  if (windowSize < 1.0f) windowSize = 1.0f; 
  if (grainCounter < windowSize) env = grainCounter / windowSize;
  else if (grainCounter > currentGrainSize - windowSize) env = (currentGrainSize - grainCounter) / windowSize;

  float grainSample = (float)audioBuffer[(int)readHead];
  float processed = (grainSample - 2048.0f) * env + 2048.0f;
  float mixed = (liveInput * (1.0 - currentMix)) + (processed * currentMix);
  
  // 4. LFO & AMP
  float finalFilterAlpha = filterAlpha;
  if (lfoRate > 0) {
    lfoPhase += lfoRate; if (lfoPhase > 6.283) lfoPhase = 0;
    // FIX 1: was +1.1 — factor could exceed 1.0, making (1-α) negative → positive feedback → blowup
    // +1.0 keeps the factor in [0.0, 1.0]; safety clamp guards against float edge cases
    finalFilterAlpha *= (sin(lfoPhase) + 1.0) * 0.5;
    if (finalFilterAlpha > 1.0f) finalFilterAlpha = 1.0f;
    if (finalFilterAlpha < 0.0f) finalFilterAlpha = 0.0f;
  }

  float amplified = (mixed - 2048.0f) * digitalGain + 2048.0f;
  bool clipped = false;
  if (amplified >= 4095) { amplified = 4095; clipped = true; } 
  if (amplified <= 0) { amplified = 0; clipped = true; }

  // FIX 5: set flag; loop() drives the LED — digitalWrite() in an ISR adds jitter
  if (clipped) clipFlashTimer = 500;
  clipLedOn = (clipFlashTimer > 0);
  if (clipFlashTimer > 0) clipFlashTimer--;

  lastOutput = (finalFilterAlpha * amplified) + ((1.0 - finalFilterAlpha) * lastOutput);

  // 5. DEGRADE & OUTPUT
  downsampleCounter++;
  if (downsampleCounter >= sampleHoldCount) {
    downsampleCounter = 0;
    uint16_t outVal = (uint16_t)lastOutput;
    if (bitReduction > 0) outVal = (outVal >> bitReduction) << bitReduction;
    heldSample = outVal;
  }
  analogWrite(DAC_PIN, heldSample);
}

void setup() {
  analogReadResolution(10); analogWriteResolution(12);
  pinMode(FREEZE_BUT, INPUT_PULLUP); pinMode(RANDOM_BUT, INPUT_PULLUP);
  pinMode(FREEZE_LED, OUTPUT); pinMode(RANDOM_LED, OUTPUT); pinMode(CLIP_LED, OUTPUT);

  for(int i=0; i<3; i++) {
    digitalWrite(FREEZE_LED, HIGH); digitalWrite(RANDOM_LED, HIGH); digitalWrite(CLIP_LED, HIGH); delay(100);
    digitalWrite(FREEZE_LED, LOW); digitalWrite(RANDOM_LED, LOW); digitalWrite(CLIP_LED, LOW); delay(100);
  }

  for(int i=0; i<BUFFER_SIZE; i++) audioBuffer[i] = 2048;
  uint8_t timer_type = GPT_TIMER; int8_t tindex = FspTimer::get_available_timer(timer_type);
  audioTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, SAMPLE_RATE, 0.0f, audioCallback);
  audioTimer.setup_overflow_irq(); audioTimer.open(); audioTimer.start();
}

void loop() {
  // FIX 2: all analogRead() calls live here — safe from interrupt context
  // loop() runs well above 12 kHz when idle, so the ISR always has a fresh value
  rawAudioIn   = analogRead(ADC_PIN);
  rawSizeKnob  = analogRead(SIZE_KNOB);
  rawSpeedKnob = analogRead(SPEED_KNOB);
  rawMixKnob   = analogRead(MIX_KNOB);
  rawToneKnob  = analogRead(TONE_KNOB);

  // FIX 3: random() computed here — one fresh value per grain reset is sufficient
  rawSpray = isRandom ? random(-50, 50) : 0;

  // FIX 5: drive clip LED from loop() using ISR-set flag
  digitalWrite(CLIP_LED, clipLedOn);

  bool fCur = digitalRead(FREEZE_BUT); bool rCur = digitalRead(RANDOM_BUT);

  // GLOBAL RESET (Hold both)
  if (fCur == LOW && rCur == LOW) {
    if (comboStartTime == 0) comboStartTime = millis();
    if (millis() - comboStartTime > 1500) {
      isFrozen = false; isRandom = false; isReverse = false; lfoRate = 0; envSoftness = 0.15; bitReduction = 0; sampleHoldCount = 1; digitalGain = 1.5;
      for(int i=0; i<10; i++) { digitalWrite(FREEZE_LED, i%2); digitalWrite(RANDOM_LED, i%2); digitalWrite(CLIP_LED, i%2); delay(40); }
      digitalWrite(FREEZE_LED, LOW); digitalWrite(RANDOM_LED, LOW); digitalWrite(CLIP_LED, LOW); comboStartTime = 0;
    }
  } else comboStartTime = 0;

  static bool fLast = HIGH;
  if (fCur == LOW && fLast == HIGH && rCur == HIGH) { isFrozen = !isFrozen; digitalWrite(FREEZE_LED, isFrozen); delay(100); }
  fLast = fCur;

  static bool rLast = HIGH;
  if (rCur == LOW && rLast == HIGH) {
    if (fCur == LOW) { isReverse = !isReverse; digitalWrite(RANDOM_LED, HIGH); delay(50); digitalWrite(RANDOM_LED, LOW); delay(50); digitalWrite(RANDOM_LED, isRandom); }
    else { isRandom = !isRandom; digitalWrite(RANDOM_LED, isRandom); }
    delay(100);
  }
  rLast = rCur;
}
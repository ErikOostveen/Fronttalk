#include <ClickEncoder.h>
#include <Arduino.h>
#include <MIDI.h>
#include "Talkie.h"
#include "SpeakBanks.h"
#include <math.h>

bool pendingGateHigh = false;
unsigned long gateHighTime = 0;

unsigned long lastActivityTime = 0;
bool idleEffectActive = false;
unsigned long nextIdleFlashTime = 0;

// ——— MIDI on Serial1 (DIN-jack to RX1 pin 0) ———
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// talkie
volatile bool previewActive = false;
Talkie Voice(true, false);

int TalkiePitch = 8000;
#define BASE_TALKIE_PITCH 8000
#define PITCH_SENSITIVITY 8.0

#define NUM_BANKS 7
#define NUM_FX    4
#define DEFAULT_LATCH_BANK  0
#define DEFAULT_LATCH_INDEX 0

int latchedIndex = -1;
const uint8_t** latchedBank = nullptr;

// Encoder A (effectLevel)
#define ENCODER_A_PIN_A  2
#define ENCODER_A_PIN_B  7
#define ENCODER_A_SW     4

// Encoder B (phoneme & bank)
#define ENCODER_B_PIN_A A2
#define ENCODER_B_PIN_B 12
#define ENCODER_B_SW    8

#define GATE_INPUT_PIN   A3
#define PITCH_CV_PIN     A4

int lastGateState = LOW;

ClickEncoder encoderA(ENCODER_A_PIN_A, ENCODER_A_PIN_B, ENCODER_A_SW, 4);
ClickEncoder encoderB(ENCODER_B_PIN_A, ENCODER_B_PIN_B, ENCODER_B_SW, 4);

int effectMode  = 1;
int effectLevel = 0;

const uint8_t** vocabBanks[NUM_BANKS] = {
    Vocab_AstroBlaster,
    Vocab_Erik,
    Vocab_US_Large,
    Vocab_US_Acorn_Alphabet,
    Vocab_US_Acorn,
    Vocab_US_Acorn_Numbers,
    Vocab_US_Clock
};
const size_t vocabBankSizes[NUM_BANKS] = {
    num_Vocab_AstroBlaster,
    num_Vocab_Erik,
    num_Vocab_US_Large,
    num_Vocab_US_Acorn_Alphabet,
    num_Vocab_US_Acorn,
    num_Vocab_US_Acorn_Numbers,
    num_Vocab_US_Clock
};

int currentBankIndex = DEFAULT_LATCH_BANK;

#define LED_A_PIN 5
#define LED_B_PIN 6
struct LedFlashState {
    bool active = false;
    unsigned long startTime = 0, duration = 0;
};
LedFlashState ledAState, ledBState;

#define RESPONSE_MODE_PIN 10
#define PLAY_MODE_PIN     11
#define AUDIO_GATE_PIN    9 // Pin 9 of Teensy V4.0
#define TRIGGER_OUT_PIN   A1
#define TURN_TO_TALK_PIN  A0

// ——— MIDI channel selector pins ———
#define MIDI_CH_PIN0  A5  // bit 0 (LSB)
#define MIDI_CH_PIN1  A6  // bit 1
#define MIDI_CH_PIN2  A7  // bit 2
#define MIDI_CH_PIN3  A8  // bit 3 (MSB)

bool TurnToTalk;
String responseMode, playMode;

volatile bool triggerActive = false;
volatile unsigned long triggerEndTime = 0;
int encoderBIndex = 0;

// debounce for pushbuttons
static bool lastButtonAStateInitialized = false, lastButtonAState = HIGH;
static unsigned long lastDebounceTimeA = 0;
const unsigned long debounceDelayA = 250;
static bool lastButtonBStateInitialized = false, lastButtonBState = HIGH;
static unsigned long lastDebounceTimeB = 0;
const unsigned long debounceDelayB = 250;





void resetIdleTimer() {
    lastActivityTime = millis();
    if (idleEffectActive) {
        idleEffectActive = false;
        analogWrite(LED_A_PIN, 0);
        analogWrite(LED_B_PIN, 0);
        ledAState.active = false;
        ledBState.active = false;
    }
}





void speakPhoneme(const uint8_t* phoneme, int pitch) {
    lastActivityTime = millis();
    idleEffectActive = false;
    Voice.sayQ(phoneme, pitch);
}






void updateSwitchModes() {
    static int lastTurnToTalk = -1;
    static int lastResponsePin = -1;
    static int lastPlayPin = -1;

    int ttt = digitalRead(TURN_TO_TALK_PIN);
    int rsp = digitalRead(RESPONSE_MODE_PIN);
    int ply = digitalRead(PLAY_MODE_PIN);

    if (ttt != lastTurnToTalk || rsp != lastResponsePin || ply != lastPlayPin) {
        resetIdleTimer();
        lastTurnToTalk = ttt;
        lastResponsePin = rsp;
        lastPlayPin = ply;
    }

    static String lastResponseMode = "";
    responseMode = (rsp == HIGH ? "Gated" : "Triggered");
    playMode     = (ply == HIGH ? "Hold"  : "Select");

    if (responseMode != lastResponseMode) {
        digitalWrite(TRIGGER_OUT_PIN, LOW);
        lastResponseMode = responseMode;
    }
}


void handleNoteOn(byte channel, byte note, byte velocity) {
    updateSwitchModes();
    previewActive = false;
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
    if (responseMode=="Triggered") {
        triggerActive  = true;
        triggerEndTime = millis() + 25;
    }
    const uint8_t** bank = vocabBanks[currentBankIndex];
    size_t bankSize = vocabBankSizes[currentBankIndex];
    if (playMode=="Select") {
        int idx = note - 29;
        if (idx<0 || idx>=(int)bankSize) return;
        Voice.terminate();
        TalkiePitch = BASE_TALKIE_PITCH;
        speakPhoneme(bank[idx], TalkiePitch);
        latchedIndex  = idx;
        latchedBank   = bank;
        encoderBIndex = idx;
    } else {
        if (!latchedBank || latchedIndex<0) return;
        float mult = pow(2.0, (note-60)/PITCH_SENSITIVITY);
        TalkiePitch = (int)(BASE_TALKIE_PITCH * mult);
        Voice.terminate();
        speakPhoneme(latchedBank[latchedIndex], TalkiePitch);
    }
}

void handleNoteOff(byte, byte note, byte) {
    updateSwitchModes();
    if (responseMode=="Gated") {
        digitalWrite(TRIGGER_OUT_PIN, LOW);
        digitalWrite(AUDIO_GATE_PIN,  LOW);
        Voice.terminate();
    }
}

void flashLED(char led, unsigned long duration) {
    duration = constrain(duration, 10UL, 5000UL);
    auto &s = (led=='A' ? ledAState : ledBState);
    s.active = true;
    s.startTime = millis();
    s.duration  = duration;
    analogWrite(led=='A'?LED_A_PIN:LED_B_PIN, 255);
}

void myGateCallback(bool gateOn) {
    if (gateOn) {
        pendingGateHigh = true;
        gateHighTime = millis() + 12;  // Delay by 12ms
    } else {
        digitalWrite(AUDIO_GATE_PIN, LOW);
        previewActive = false;
        pendingGateHigh = false;  // Cancel any pending HIGH
    }
}

void softResetViaAIRCR() {
  // VECTKEY (0x5FA) in bits[31:16], SYSRESETREQ in bit 2
  *(volatile uint32_t *)0xE000ED0C = (0x5FA << 16) | (1 << 2);
  while (1) { }  // wait for the reset
}

void setup() {

  // ——— MIDI-channel DIP switches (pull-**up**) ———
  // Switch CLOSED → pin to GND → reads LOW → bit = 1
  // Switch OPEN   → pulled HIGH → reads HIGH → bit = 0
  pinMode(MIDI_CH_PIN0, INPUT_PULLUP);
  pinMode(MIDI_CH_PIN1, INPUT_PULLUP);
  pinMode(MIDI_CH_PIN2, INPUT_PULLUP);
  pinMode(MIDI_CH_PIN3, INPUT_PULLUP);

  // wait for pull-ups to charge and the lines to settle
  delay(20);

  // Build 0–15 from A5…A8 (A5=LSB … A8=MSB)
  uint8_t ch_bits = 0;
  ch_bits |= (digitalRead(MIDI_CH_PIN0) == LOW) << 0;
  ch_bits |= (digitalRead(MIDI_CH_PIN1) == LOW) << 1;
  ch_bits |= (digitalRead(MIDI_CH_PIN2) == LOW) << 2;
  ch_bits |= (digitalRead(MIDI_CH_PIN3) == LOW) << 3;

  Serial.begin(115200);
  analogReadResolution(12);

  // Tell Talkie to use pin 13 for its PWM audio:
  Voice.beginPWM(13);
  analogWriteResolution(13);

  // ——— MIDI over DIN on selector channel ———
  Serial1.begin(31250);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  // only 0000 → Omni; 0001–1111 → channels 1–15
  if (ch_bits == 0) {
    Serial.println("Using MIDI channel: OMNI");
    MIDI.begin(MIDI_CHANNEL_OMNI);
  } else {
    Serial.print("Using MIDI channel: ");
    Serial.println(ch_bits);
    MIDI.begin(ch_bits);
  }

  // I/O pins
  pinMode(GATE_INPUT_PIN, INPUT);
  pinMode(PITCH_CV_PIN, INPUT);
  pinMode(LED_A_PIN, OUTPUT); analogWrite(LED_A_PIN,0);
  pinMode(LED_B_PIN, OUTPUT); analogWrite(LED_B_PIN,0);
  pinMode(ENCODER_A_SW, INPUT_PULLUP);
  pinMode(ENCODER_B_SW, INPUT_PULLUP);
  pinMode(TURN_TO_TALK_PIN, INPUT_PULLUP);
  pinMode(RESPONSE_MODE_PIN, INPUT_PULLUP);
  pinMode(PLAY_MODE_PIN,     INPUT_PULLUP);
  pinMode(AUDIO_GATE_PIN,    OUTPUT);
  digitalWrite(AUDIO_GATE_PIN, LOW);
  pinMode(TRIGGER_OUT_PIN,   OUTPUT);
  digitalWrite(TRIGGER_OUT_PIN, LOW);

  Talkie::setGateCallback(myGateCallback);

  effectLevel = 0;
  encoderBIndex = 0;
  currentBankIndex = DEFAULT_LATCH_BANK;
  latchedBank = vocabBanks[currentBankIndex];
  latchedIndex = encoderBIndex;
}

static int lastCVNote = 0;

void loop() {

    TurnToTalk = (digitalRead(TURN_TO_TALK_PIN) == HIGH);

    updateSwitchModes();
    MIDI.read();
        
    // --- Idle LED effect ---
    if (!idleEffectActive && millis() - lastActivityTime > 60000) { // LED Patterns kick in after 60 seconds
        idleEffectActive = true;
        nextIdleFlashTime = millis();
    }

    // If idle mode is active, animate randomly
    if (idleEffectActive && millis() >= nextIdleFlashTime) {
        // Randomly choose LED: 0 = A, 1 = B
        int led = random(0, 2);
        // Random duration: 100–600 ms
        unsigned long dur = random(100, 600);
        // Flash the LED
        flashLED(led == 0 ? 'A' : 'B', dur);
        // Set next flash time (add a short random delay between animations)
        nextIdleFlashTime = millis() + dur + random(200, 800);
    }

    if (pendingGateHigh && millis() >= gateHighTime) {
        digitalWrite(AUDIO_GATE_PIN, HIGH);
        pendingGateHigh = false;
    }

    int gateState = digitalRead(GATE_INPUT_PIN);
    if (gateState == HIGH && lastGateState == LOW) {

      // Rising edge: read A4, map 0–3.3 V (0–4095) → MIDI 48–72
      int raw = analogRead(PITCH_CV_PIN);
      // clamp just in case someone feeds you a little over 3.3 V
      if (raw > 4095) raw = 4095;
      int cvNote = map(raw, 0, 4095, 48, 72);
      lastCVNote = cvNote;

    handleNoteOn(1, cvNote, 127);
    }
    else if (gateState == LOW && lastGateState == HIGH) {
      Serial.println("Low");
        // falling edge: send matching “Note Off”
        handleNoteOff(1, lastCVNote, 0);
    }
    lastGateState = gateState;

    encoderA.service();
    encoderB.service();

    // --- Encoder B pushbutton (Bank select, debounced) ---
    bool buttonBState = digitalRead(ENCODER_B_SW);

    if (!lastButtonBStateInitialized) {
        lastButtonBState = buttonBState;
        lastButtonBStateInitialized = true;
    }

    if (buttonBState == LOW && lastButtonBState == HIGH &&
        (millis() - lastDebounceTimeB) > debounceDelayB) {

        resetIdleTimer();;  
        lastDebounceTimeB = millis();
        
        currentBankIndex++;
        if (currentBankIndex >= NUM_BANKS) currentBankIndex = 0;
        if (currentBankIndex == 0) { flashLED('A', 300); }
        if (currentBankIndex == NUM_BANKS-1) { flashLED('B', 300); }

        encoderBIndex = 0;
        latchedIndex = 0;
        latchedBank = vocabBanks[currentBankIndex];
    }
    lastButtonBState = buttonBState;

    // --- Encoder A pushbutton (effectMode, debounced) ---
    bool buttonAState = digitalRead(ENCODER_A_SW);

    if (!lastButtonAStateInitialized) {
        lastButtonAState = buttonAState;
        lastButtonAStateInitialized = true;
    }

    if (buttonAState == LOW && lastButtonAState == HIGH &&
        (millis() - lastDebounceTimeA) > debounceDelayA) {
        lastDebounceTimeA = millis();

        effectMode++;
        if (effectMode > NUM_FX) effectMode = 1;
        effectLevel = 0;
        if (effectMode == 1) { flashLED('A', 300); }
        if (effectMode == NUM_FX) { flashLED('B', 300); }
    }
    lastButtonAState = buttonAState;

    // Non-blocking trigger pulse management
    if (triggerActive && (millis() >= triggerEndTime)) {
        digitalWrite(TRIGGER_OUT_PIN, LOW);
        triggerActive = false;
    }

    // --- Encoder A for effectLevel ---
    int encA = encoderA.getValue();
    if (encA != 0) {
        int prevLevel = effectLevel;
        int newLevel = effectLevel + encA;

        if (newLevel < 0) {
            // Already at bottom and still turning left
            if (effectLevel == 0 && encA < 0) {
                flashLED('A', 300);
            }
            effectLevel = 0;
        }
        else if (newLevel > 25) {
            // Already at top and still turning right
            if (effectLevel == 25 && encA > 0) {
                flashLED('B', 300);
            }
            effectLevel = 25;
        }
        else {
            effectLevel = newLevel;

            // Just arrived at 0 or 25 from inside the valid range
            if (effectLevel == 0 && prevLevel > 0 && encA < 0) flashLED('A', 300);
            if (effectLevel == 25 && prevLevel < 25 && encA > 0) flashLED('B', 300);
        }

        resetIdleTimer();
    }


    // --- Encoder B: select phoneme in selected bank ---
    long maxPhoneme = vocabBankSizes[currentBankIndex];
    int encB = encoderB.getValue();
    if (encB != 0) {
        resetIdleTimer();
        int prevIndex = encoderBIndex;

        encoderBIndex += encB;
        if (encoderBIndex < 0) encoderBIndex = 0;
        if (encoderBIndex >= (int)maxPhoneme) encoderBIndex = maxPhoneme - 1;

        // ——— ONLY FLASH IF STAYING AT 0 AND MOVING CCW ———
        if (encoderBIndex == 0 && prevIndex == 0 && encB < 0) {
            flashLED('A', 300);
        }

        // ——— ONLY FLASH WHEN ARRIVING AT MAX FROM BELOW ———
        if (encoderBIndex == (int)maxPhoneme - 1 && prevIndex < encoderBIndex) {
            flashLED('B', 300);
        }

        latchedIndex = encoderBIndex;
        latchedBank = vocabBanks[currentBankIndex];

        if (TurnToTalk) {
            const uint8_t** bank = vocabBanks[currentBankIndex];
            if (bank && maxPhoneme > 0) {
                if (previewActive) {
                    Voice.terminate();
                    previewActive = false;
                    digitalWrite(AUDIO_GATE_PIN, LOW);
                }
                previewActive = true;
                Voice.terminate();
                digitalWrite(TRIGGER_OUT_PIN, HIGH);
                TalkiePitch = BASE_TALKIE_PITCH;
                speakPhoneme(bank[encoderBIndex], TalkiePitch);  
                triggerActive = true;
                triggerEndTime = millis() + 25;
            }
        }

        Serial.print(encoderBIndex);
    }


    // --- LED Fading Update ---
    if (ledAState.active) {
        unsigned long elapsed = millis() - ledAState.startTime;
        if (elapsed < ledAState.duration) {
            float brightness = 1.0 - ((float)elapsed / ledAState.duration);
            if (brightness < 0.0) brightness = 0.0;
            brightness = pow(brightness, 2.2);
            analogWrite(LED_A_PIN, (int)(brightness * 255));
        } else {
            analogWrite(LED_A_PIN, 0);
            ledAState.active = false;
        }
    }
    if (ledBState.active) {
        unsigned long elapsed = millis() - ledBState.startTime;
        if (elapsed < ledBState.duration) {
            float brightness = 1.0 - ((float)elapsed / ledBState.duration);
            if (brightness < 0.0) brightness = 0.0;
            brightness = pow(brightness, 2.2);
            analogWrite(LED_B_PIN, (int)(brightness * 255));
        } else {
            analogWrite(LED_B_PIN, 0);
            ledBState.active = false;
        }
    }

  if (buttonAState == LOW && buttonBState == LOW) {
    delay(250);
    softResetViaAIRCR();
  } 

}

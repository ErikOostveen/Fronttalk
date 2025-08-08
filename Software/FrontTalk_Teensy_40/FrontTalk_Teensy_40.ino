#include <ClickEncoder.h>
#include <Arduino.h>
#include <MIDI.h>
#include "Talkie.h"
#include "SpeakBanks.h"
#include <math.h>

// ==== Constants ====
#define BASE_TALKIE_PITCH     8000
#define PITCH_SENSITIVITY     8.0
#define NUM_BANKS             7
#define NUM_FX                4
#define DEFAULT_LATCH_BANK    0
#define DEFAULT_LATCH_INDEX   0
#define LED_A_PIN             5
#define LED_B_PIN             6
#define RESPONSE_MODE_PIN     10
#define PLAY_MODE_PIN         11
#define AUDIO_GATE_PIN        9
#define TRIGGER_OUT_PIN       A1
#define TURN_TO_TALK_PIN      A0

// Encoder A (effectLevel)
#define ENCODER_A_PIN_A       2
#define ENCODER_A_PIN_B       7
#define ENCODER_A_SW          4

// Encoder B (phoneme & bank)
#define ENCODER_B_PIN_A       A2
#define ENCODER_B_PIN_B       12
#define ENCODER_B_SW          8

#define GATE_INPUT_PIN        A3
#define PITCH_CV_PIN          A4

// MIDI channel selector DIP pins
#define MIDI_CH_PIN0          A5
#define MIDI_CH_PIN1          A6
#define MIDI_CH_PIN2          A7
#define MIDI_CH_PIN3          A8

// ==== Globals ====
bool pendingGateHigh = false;
unsigned long gateHighTime = 0;

unsigned long lastActivityTime = 0;
bool idleEffectActive = false;
unsigned long nextIdleFlashTime = 0;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

volatile bool previewActive = false;
Talkie Voice(true, false);

int TalkiePitch = BASE_TALKIE_PITCH;

int latchedIndex = -1;
const uint8_t** latchedBank = nullptr;

int lastGateState = LOW;

// Encoder objects
ClickEncoder encoderA(ENCODER_A_PIN_A, ENCODER_A_PIN_B, ENCODER_A_SW, 4);
ClickEncoder encoderB(ENCODER_B_PIN_A, ENCODER_B_PIN_B, ENCODER_B_SW, 4);

// Effect and bank variables
int effectMode  = 1;
int effectLevel = 0;
int currentBankIndex = DEFAULT_LATCH_BANK;
int encoderBIndex = 0;

// Bank and vocab definitions
const uint8_t** vocabBanks[NUM_BANKS] = {
    Vocab_AstroBlaster, Vocab_Erik, Vocab_US_Large,
    Vocab_US_Acorn_Alphabet, Vocab_US_Acorn,
    Vocab_US_Acorn_Numbers, Vocab_US_Clock
};
const size_t vocabBankSizes[NUM_BANKS] = {
    num_Vocab_AstroBlaster, num_Vocab_Erik, num_Vocab_US_Large,
    num_Vocab_US_Acorn_Alphabet, num_Vocab_US_Acorn,
    num_Vocab_US_Acorn_Numbers, num_Vocab_US_Clock
};

// LED fading state
struct LedFlashState {
    bool active = false;
    unsigned long startTime = 0, duration = 0;
};
LedFlashState ledAState, ledBState;

// DIP/Mode state
bool TurnToTalk;
String responseMode, playMode;

// Trigger pulse state
volatile bool triggerActive = false;
volatile unsigned long triggerEndTime = 0;

// Debounce for pushbuttons
static bool lastButtonAStateInitialized = false, lastButtonAState = HIGH;
static unsigned long lastDebounceTimeA = 0;
const unsigned long debounceDelayA = 250;
static bool lastButtonBStateInitialized = false, lastButtonBState = HIGH;
static unsigned long lastDebounceTimeB = 0;
const unsigned long debounceDelayB = 250;

static int lastCVNote = 0;

// ==== Utility Functions ====

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
    static int lastTurnToTalk = -1, lastResponsePin = -1, lastPlayPin = -1;
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

void flashLED(char led, unsigned long duration) {
    duration = constrain(duration, 10UL, 5000UL);
    auto &s = (led == 'A' ? ledAState : ledBState);
    s.active = true;
    s.startTime = millis();
    s.duration  = duration;
    analogWrite((led == 'A' ? LED_A_PIN : LED_B_PIN), 255);
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
    *(volatile uint32_t *)0xE000ED0C = (0x5FA << 16) | (1 << 2);
    while (1) {}
}

// ==== MIDI Handlers ====

void handleNoteOn(byte channel, byte note, byte velocity) {
    handleNoteOn(channel, note, velocity, 0, LOW);
}

void handleNoteOn(byte channel, byte note, byte velocity, int raw, int gateState) {
    updateSwitchModes();
    previewActive = false;
    digitalWrite(TRIGGER_OUT_PIN, HIGH);

    if (responseMode == "Triggered") {
        triggerActive  = true;
        triggerEndTime = millis() + 25;
    }
    const uint8_t** bank = vocabBanks[currentBankIndex];
    size_t bankSize = vocabBankSizes[currentBankIndex];

    if (playMode == "Select") {
        int idx = (gateState == HIGH) ? map(raw, 0, 3400, 0, bankSize - 1) : (note - 29);
        idx = constrain(idx, 0, (int)bankSize - 1);

        Voice.terminate();
        TalkiePitch = BASE_TALKIE_PITCH;
        speakPhoneme(bank[idx], TalkiePitch);
        latchedIndex  = idx;
        latchedBank   = bank;
        encoderBIndex = idx;
    }
    else if (playMode == "Hold") {
        int mappedPitch = (gateState == HIGH)
            ? map(raw, 0, 3400, 2000, 16000)
            : map(note, 36, 84, 2000, 16000);
        mappedPitch = constrain(mappedPitch, 2000, 16000);

        Voice.terminate();
        TalkiePitch = mappedPitch;
        // Only play the currently latched phrase!
        speakPhoneme(latchedBank[latchedIndex], TalkiePitch);
        digitalWrite(AUDIO_GATE_PIN, HIGH);
    }
}

void handleNoteOff(byte, byte note, byte) {
    updateSwitchModes();
    if (responseMode == "Gated") {
        digitalWrite(TRIGGER_OUT_PIN, LOW);
        digitalWrite(AUDIO_GATE_PIN, LOW);
        Voice.terminate();
    }
}

// ==== SETUP ====
void setup() {
    // --- MIDI-channel DIP switches (pull-**up**) ---
    pinMode(MIDI_CH_PIN0, INPUT_PULLUP);
    pinMode(MIDI_CH_PIN1, INPUT_PULLUP);
    pinMode(MIDI_CH_PIN2, INPUT_PULLUP);
    pinMode(MIDI_CH_PIN3, INPUT_PULLUP);
    delay(20);

    // Build 0–15 from A5…A8 (A5=LSB … A8=MSB)
    uint8_t ch_bits = 0;
    ch_bits |= (digitalRead(MIDI_CH_PIN0) == LOW) << 0;
    ch_bits |= (digitalRead(MIDI_CH_PIN1) == LOW) << 1;
    ch_bits |= (digitalRead(MIDI_CH_PIN2) == LOW) << 2;
    ch_bits |= (digitalRead(MIDI_CH_PIN3) == LOW) << 3;

    Serial.begin(115200);
    analogReadResolution(12);

    Voice.beginPWM(13); // Tell Talkie to use pin 13 for PWM audio
    analogWriteResolution(13);

    Serial1.begin(31250);
    MIDI.setHandleNoteOn(handleNoteOn);
    MIDI.setHandleNoteOff(handleNoteOff);

    // only 0000 → Omni; 0001–1111 → channels 1–15
    MIDI.begin(ch_bits == 0 ? MIDI_CHANNEL_OMNI : ch_bits);

    // I/O pins
    pinMode(GATE_INPUT_PIN, INPUT);
    pinMode(PITCH_CV_PIN, INPUT);
    pinMode(LED_A_PIN, OUTPUT); analogWrite(LED_A_PIN, 0);
    pinMode(LED_B_PIN, OUTPUT); analogWrite(LED_B_PIN, 0);
    pinMode(ENCODER_A_SW, INPUT_PULLUP);
    pinMode(ENCODER_B_SW, INPUT_PULLUP);
    pinMode(TURN_TO_TALK_PIN, INPUT_PULLUP);
    pinMode(RESPONSE_MODE_PIN, INPUT_PULLUP);
    pinMode(PLAY_MODE_PIN, INPUT_PULLUP);
    pinMode(AUDIO_GATE_PIN, OUTPUT); digitalWrite(AUDIO_GATE_PIN, LOW);
    pinMode(TRIGGER_OUT_PIN, OUTPUT); digitalWrite(TRIGGER_OUT_PIN, LOW);

    Talkie::setGateCallback(myGateCallback);

    effectLevel = 0;
    encoderBIndex = 0;
    currentBankIndex = DEFAULT_LATCH_BANK;
    latchedBank = vocabBanks[currentBankIndex];
    latchedIndex = encoderBIndex;
}

// ==== LOOP ====
void loop() {
    TurnToTalk = (digitalRead(TURN_TO_TALK_PIN) == HIGH);
    updateSwitchModes();
    MIDI.read();

    // --- Idle LED effect ---
    if (!idleEffectActive && millis() - lastActivityTime > 60000) {
        idleEffectActive = true;
        nextIdleFlashTime = millis();
    }
    if (idleEffectActive && millis() >= nextIdleFlashTime) {
        int led = random(0, 2);
        unsigned long dur = random(100, 600);
        flashLED(led == 0 ? 'A' : 'B', dur);
        nextIdleFlashTime = millis() + dur + random(200, 800);
    }

    // Pending delayed gate-on
    if (pendingGateHigh && millis() >= gateHighTime) {
        digitalWrite(AUDIO_GATE_PIN, HIGH);
        pendingGateHigh = false;
    }

    // Gate input
    int gateState = digitalRead(GATE_INPUT_PIN);
    if (gateState == HIGH && lastGateState == LOW) {
        int raw = analogRead(PITCH_CV_PIN);
        raw = min(raw, 3400);

        if (playMode == "Select") {
            int idx = map(raw, 0, 3400, 0, vocabBankSizes[currentBankIndex] - 1);
            latchedIndex = idx;
            latchedBank = vocabBanks[currentBankIndex];
            Voice.terminate();
            TalkiePitch = BASE_TALKIE_PITCH;
            speakPhoneme(latchedBank[latchedIndex], TalkiePitch);
        } else if (playMode == "Hold") {
            int mappedPitch = map(raw, 0, 3400, 2000, 16000);
            Voice.terminate();
            TalkiePitch = mappedPitch;
            speakPhoneme(latchedBank[latchedIndex], TalkiePitch);
            digitalWrite(AUDIO_GATE_PIN, HIGH);
        }
    } else if (gateState == LOW && lastGateState == HIGH) {
        handleNoteOff(1, lastCVNote, 0);
    }
    lastGateState = gateState;

    // Encoder service
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
        resetIdleTimer();
        lastDebounceTimeB = millis();
        currentBankIndex = (currentBankIndex + 1) % NUM_BANKS;
        if (currentBankIndex == 0) flashLED('A', 300);
        if (currentBankIndex == NUM_BANKS - 1) flashLED('B', 300);
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
        effectMode = (effectMode % NUM_FX) + 1;
        effectLevel = 0;
        if (effectMode == 1) flashLED('A', 300);
        if (effectMode == NUM_FX) flashLED('B', 300);
    }
    lastButtonAState = buttonAState;

    // --- Non-blocking trigger pulse management ---
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
            if (effectLevel == 0 && encA < 0) flashLED('A', 300);
            effectLevel = 0;
        } else if (newLevel > 25) {
            if (effectLevel == 25 && encA > 0) flashLED('B', 300);
            effectLevel = 25;
        } else {
            effectLevel = newLevel;
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
        encoderBIndex = constrain(encoderBIndex + encB, 0, (int)maxPhoneme - 1);

        // Flash when at boundaries
        if (encoderBIndex == 0 && prevIndex == 0 && encB < 0) flashLED('A', 300);
        if (encoderBIndex == (int)maxPhoneme - 1 && prevIndex < encoderBIndex) flashLED('B', 300);

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
    }

    // --- LED Fading Update ---
    for (auto led : {'A', 'B'}) {
        LedFlashState& s = (led == 'A' ? ledAState : ledBState);
        int pin = (led == 'A' ? LED_A_PIN : LED_B_PIN);
        if (s.active) {
            unsigned long elapsed = millis() - s.startTime;
            if (elapsed < s.duration) {
                float brightness = 1.0f - ((float)elapsed / s.duration);
                brightness = max(0.0f, pow(brightness, 2.2f));
                analogWrite(pin, (int)(brightness * 255));
            } else {
                analogWrite(pin, 0);
                s.active = false;
            }
        }
    }

    // --- Simultaneous A/B button soft reset ---
    if (buttonAState == LOW && buttonBState == LOW) {
        delay(250);
        softResetViaAIRCR();
    }
}

const byte ledPin = PF_3;
const byte interruptPin = PF_4;
volatile byte state = LOW;


void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
}

void loop() {
  digitalWrite(ledPin, state);
  
}

void blink() {
  state = !state;
}

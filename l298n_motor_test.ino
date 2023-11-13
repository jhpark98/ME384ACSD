#define enA 9     // PWM capable pin
#define in1 6
#define in2 7

int pwmOutput = 200; // 

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set initial rotation direction ("forward")
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin

}

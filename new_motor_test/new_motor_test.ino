#define pwmA D5     // PWM capable pin
#define in1A D3
#define in2A D4

#define pwmB D6     // PWM capable pin
#define in1B D7
#define in2B D8

#define stby D2

int pwmOutput = 200; // 

void setup() {
  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  
  pinMode(stby, OUTPUT);
  

  // Set initial rotation direction ("forward")
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
 
  
  digitalWrite(stby, HIGH);
}


void loop() {
 
  analogWrite(pwmA, pwmOutput); // Send PWM signal to L298N Enable pin
  analogWrite(pwmB, pwmOutput); // Send PWM signal to L298N Enable pin

}

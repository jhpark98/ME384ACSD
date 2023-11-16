#define pwmA 23     // PWM capable pin
#define in1A 21
#define in2A 22

#define pwmB 26     // PWM capable pin
#define in1B 24
#define in2B 25

#define stby 20

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
  analogWrite(pwmA, pwmOutput); // Send PWM signal to L298N Enable pin

}

#include <PID_v1.h>

#define PWM 9
#define IN1 7
#define IN2 8

double setpoint = 100;
double input, output;
double Kp = 1.0, Ki = 0.5, Kd = 0.1;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  input = lerRPM();
  pid.Compute();
  analogWrite(PWM, output);

  Serial.print(\"Setpoint: \");
  Serial.print(setpoint);
  Serial.print(\" | RPM: \");
  Serial.println(input);
}

double lerRPM() {
  return analogRead(A0) / 10.0; // Exemplo: substitua conforme seu sensor
}


int brk = 3 ;
int white_dir = 4;
int grey_pwm = 9 ;

void setup() {
  // put your setup code here, to run once:
//Serial.begin(9600);

pinMode(white_dir,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
analogWrite(grey_pwm,50);









}

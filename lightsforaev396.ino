/*
 * lights for aev396
*/

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(6, OUTPUT);
  pinMode(5,OUTPUT);

    Serial.begin(9600);

}
// the loop function runs over and over again forever
void loop() {
  digitalWrite(5, LOW);   
  digitalWrite(6, HIGH);   
  delay(500);
   digitalWrite(6, LOW);    // turn the LED off by making the voltage LOW
   digitalWrite(5, HIGH);    // turn the LED off by making the voltage LOW
  delay(500);
    }


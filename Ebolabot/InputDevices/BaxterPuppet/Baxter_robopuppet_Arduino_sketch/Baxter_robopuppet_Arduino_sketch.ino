void setup()  {
  Serial.begin(9600);
}

void loop()  {
  Serial.print(analogRead(A0));
  Serial.print(" ");
  Serial.print(analogRead(A1));
  Serial.print(" ");
  Serial.print(analogRead(A2));
  Serial.print(" ");
  Serial.print(analogRead(A3));
  Serial.print(" ");
  Serial.print(analogRead(A4));
  Serial.print(" ");
  Serial.print(analogRead(A5));
  
  Serial.print(" ");
  Serial.print(analogRead(A6));
  Serial.print(" ");
  Serial.print(analogRead(A7));
  Serial.print(" ");
  Serial.print(analogRead(A8));
  Serial.print(" ");
  Serial.print(analogRead(A9));
  Serial.print(" ");
  Serial.print(analogRead(A10));
  Serial.print(" ");
  Serial.println(analogRead(A11));
  
  delay(50);
}

int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.println("Serial port begin at baud=9600...");
}

void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // show what you got:
    Serial.print("Received: ");
    Serial.println(incomingByte, HEX);
  }
}

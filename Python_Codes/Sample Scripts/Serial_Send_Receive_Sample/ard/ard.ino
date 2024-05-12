void setup() {
  Serial.begin(9600);  // Set the baud rate to match the Python script
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    Serial.println("Received from Python: " + message);

    // Send a response back to Python
    Serial.println("Hello from Arduino!");
  }
}

// Define the relay pin
const int relayPin = A0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Arduino Relay Control");

  // Set the relay pin as an output
  pinMode(relayPin, OUTPUT);

  // Ensure relay is off initially
  digitalWrite(relayPin, LOW); // Or HIGH depending on relay logic (active high/low)
}

void loop() {
  // Check if there is data available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming string
    String command = Serial.readStringUntil('\n'); // Read until newline

    // Remove any leading/trailing whitespace
    command.trim();

    // Check for "start" command
    if (command.equalsIgnoreCase("start")) {
      Serial.println("Starting relay...");
      digitalWrite(relayPin, HIGH); // Or LOW depending on relay logic
    }
    // Check for "stop" command
    else if (command.equalsIgnoreCase("stop")) {
      Serial.println("Stopping relay...");
      digitalWrite(relayPin, LOW); // Or HIGH depending on relay logic
    }
    else if (command.length() >0){ // prevent printing "unknown command" on an empty line.
        Serial.print("Unknown command: ");
        Serial.println(command);
    }
  }
}
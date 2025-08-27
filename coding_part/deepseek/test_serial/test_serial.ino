// --- Add these constants and variables at the top ---
const long ULTRASONIC_READ_INTERVAL = 50; // Read US sensors less frequently
unsigned long lastUltrasonicReadTime = 0;
bool ultrasonicToggle = false; // Alternate between front and rear sensors
int irStates[4]={0,0,0,0};
// --- Replace the readSensors function with this non-blocking version ---
void readSensors() {
    // Read IR Sensors (always fast)
    irStates[0] = digitalRead(IR_FRONT_LEFT_PIN);
    irStates[1] = digitalRead(IR_FRONT_RIGHT_PIN);
    irStates[2] = digitalRead(IR_RIGHT_45_PIN);
    irStates[3] = digitalRead(IR_LEFT_45_PIN);
    
    // Read MPU6050 (always fast)
    if (imuReady) {
        readMPU6050Data();
    }
    
    // Read Ultrasonic Sensors with non-blocking approach
    unsigned long currentMillis = millis();
    if (currentMillis - lastUltrasonicReadTime >= ULTRASONIC_READ_INTERVAL) {
        if (ultrasonicToggle) {
            usDistances[0] = readUltrasonic(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
        } else {
            usDistances[1] = readUltrasonic(US_REAR_TRIG_PIN, US_REAR_ECHO_PIN);
        }
        ultrasonicToggle = !ultrasonicToggle;
        lastUltrasonicReadTime = currentMillis;
    }
}

// --- Replace the sendSensorData function with this optimized version ---
void sendSensorData() {
    // Use a single print statement with minimal formatting
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
        "SENSORS,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d",
        irStates[0], irStates[1], irStates[2], irStates[3],
        usDistances[0], usDistances[1],
        imuAccelX, imuAccelY, imuAccelZ,
        imuGyroX, imuGyroY, imuGyroZ
    );
    Serial.println(buffer);
}

// --- Replace the parseAndExecuteCommand function with this efficient version ---
void parseAndExecuteCommand(const String &command) {
    // Check for PING first (most common command)
    if (command == "PING") {
        Serial.println("PONG");
        return;
    }
    
    // Use character-based parsing instead of String operations
    if (command.startsWith("MOTOR,")) {
        targetMotorSpeed = command.substring(6).toInt();
        targetMotorSpeed = constrain(targetMotorSpeed, -255, 255);
    } 
    else if (command.startsWith("SERVO,")) {
        targetServoAngle = command.substring(6).toInt();
        targetServoAngle = constrain(targetServoAngle, 0, 180);
    }
    else if (command == "ACK_START") {
        runStarted = true;
        Serial.println("NANO: Run started");
    }
    else if (command.length() > 0) {
        Serial.print("NANO: Unknown command: ");
        Serial.println(command);
    }
}

// --- Modify the loop function ---
void loop() {
    // Check for start button press
    checkStartButton();
    
    // Read sensors (non-blocking)
    readSensors();
    
    // Send sensor data at regular intervals
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
        sendSensorData();
        lastSensorSendTime = currentMillis;
    }
    
    // Check for commands from Raspberry Pi (non-blocking)
    while (Serial.available() > 0) {
        String receivedCommand = Serial.readStringUntil('\n');
        receivedCommand.trim();
        parseAndExecuteCommand(receivedCommand);
    }
    
    // Update actuators if run has started
    if (runStarted) {
        updateMotor();
        updateServo();
    }
    
    // No delay needed - we're using non-blocking timing
}

// HELPER FUNCTIONS FOR MODES
long getDistance(int maxDistanceCm = 400) {
    // Convert max distance to timeout (microseconds)
    // Sound travels ~343 m/s, round trip = distance * 2
    // Time = (distance_cm * 0.01 * 2) / 343 * 1000000 microseconds
    // Simplified: timeout = distance_cm * 58.3
    unsigned long timeoutUs = maxDistanceCm * 58;
    
    // Clear the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Trigger the sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the echo pin with calculated timeout
    long duration = pulseIn(echoPin, HIGH, timeoutUs);
    
    // Calculate distance (sound travels at ~343 m/s)
    // Duration is round-trip time, so divide by 2
    // Distance = (duration * 0.0343) / 2
    long distance = duration * 0.01715; // Simplified: 0.0343/2 = 0.01715
    
    return distance;
}

// MODE FUNCTIONS
void dance() {
    car.setSteering(0.5f);    // left
    delay(500);
    car.setSteering(-0.5f); // right
    delay(500);
}

void followLine() {
    // Constant settings
    const unsigned long LOST_TIMEOUT_MS = 500;    // timeout for permanent line loss
    const float BASE_THROTTLE = 0.45;
    const float BASE_STEERING = 0.15;
    const float ALIGN_THRESHOLD = 0.0;    // Below this, robot will reverse

    // Static variables for error-based line following state
    static unsigned long lineLastSeen = 0;                // timestamp when line was last detected
    static float lastDirection = 0.0;                 // remember last correction direction
    
    // Read sensor states (HIGH = black line, LOW = white background)
    int leftSensor = digitalRead(leftSensorPin);     // 0 or 1
    int rightSensor = digitalRead(rightSensorPin); // 0 or 1
    
    // Determine error based on sensor states
    float error = 0.0;
    if (leftSensor || rightSensor) { // Check if at least one sensor is on the line
        lastDirection = leftSensor - rightSensor;    // Remember this direction (swapped)
        error = lastDirection * 0.5; // Scale error to -0.5 to +0.5 range
        lineLastSeen = millis();            // Update timestamp when line is detected
    }
    else { // Both sensors off line - handle line loss case
        error = lastDirection; // Use last known direction
    }
    
    // Initialize motor commands
    float throttle = 0.0;
    float steering = 0.0;
    
    // Check if timeout has been exceeded
    if (millis() - lineLastSeen <= LOST_TIMEOUT_MS) {
        // Determine alignment (1 - absolute value of error)
        float alignment = 1.0 - abs(error);
        
        // Calculate throttle factor using alignment threshold
        float throttle_factor = (alignment - ALIGN_THRESHOLD) / (1.0 - ALIGN_THRESHOLD);
        
        // Calculate throttle and steering based on alignment and error
        throttle = BASE_THROTTLE * throttle_factor;    // Can go negative for reverse
        steering = BASE_STEERING * error;                        // Steer proportionally to error
    }
    
    // Apply motor commands
    car.setThrottle(throttle);
    car.setSteering(steering);
}

void wanderCage() {
    // Constant settings
    const float THROTTLE = 0.3;
    const float STEERING = 0.3;
    const unsigned long SPIN_TIME_MS = 600;
    
    // Static variables for cage wandering state
    static unsigned long spinStartTime = 0;
    static int spinDirection = 0;    // -1 = right, +1 = left
    
    // Check if we're still within spin timeout
    if (millis() - spinStartTime < SPIN_TIME_MS) {
        // Continue spinning in the chosen direction
        car.setThrottle(0.0);
        car.setSteering(STEERING * spinDirection);
    } else {
        // Not spinning - read sensors and check for boundaries
        bool leftSensor = digitalRead(leftSensorPin);
        bool rightSensor = digitalRead(rightSensorPin);
        
        // Check if we hit a boundary
        if (leftSensor || rightSensor) {
            // Hit boundary - start spinning away from it
            spinStartTime = millis();
            
            if (leftSensor && !rightSensor) {
                spinDirection = -1;    // Left sensor hit - spin right (away from boundary) 
            } else {
                // Right sensor hit OR both sensors hit - spin left (away from boundary)
                spinDirection = 1;
            }
            
            car.setThrottle(0.0);
            car.setSteering(STEERING * spinDirection);
        } else {
            // No boundary detected - go straight ahead
            car.setThrottle(THROTTLE);
            car.setSteering(0.0);
        }
    }
}

void collisionAvoidance() {
    // Non-blocking collision avoidance using interrupt-based ultrasonic
    const int MAX_DISTANCE = 40;      // cm - no interference at this distance
    const float REDUCTION_RATE = 0.05; // throttle reduction per cm closer
    const float MIN_THROTTLE = -0.25;   // minimum throttle (max reverse speed)

    // Get user's intended velocities
    float userThrottle, userSteering;
    getUserVels(userThrottle, userSteering);
    
    // Get current distance (non-blocking, always fresh!)
    float distance = nbPulseIn();
    
    // Send telemetry (sendTelem already has built-in rate limiting)
    String distanceMsg = "D: " + String(distance, 1) + " cm";
    sendTelem(distanceMsg.c_str());
    
    // Apply proportional obstacle avoidance
    if (distance > 0 && distance < MAX_DISTANCE) {
        // Calculate throttle reduction based on distance
        float distanceFromMax = MAX_DISTANCE - distance;
        float throttleReduction = distanceFromMax * REDUCTION_RATE;
        
        // Apply reduction to user throttle
        float modifiedThrottle = userThrottle - throttleReduction;
        
        // Cap throttle to minimum value (prevent excessive reverse)
        modifiedThrottle = max(modifiedThrottle, MIN_THROTTLE);
        
        // Send telemetry about the modification
        String modMsg = "Throttle: " + String(userThrottle, 2) + " -> " + 
                       String(modifiedThrottle, 2) + " (reduced " + 
                       String(throttleReduction, 2) + ")";
        sendTelem(modMsg.c_str());
        
        userThrottle = modifiedThrottle;
    }
    
    // Apply commands (steering unaffected for evasive maneuvers)
    car.setThrottle(userThrottle);
    car.setSteering(userSteering);
}

void targetLock() {
    // Target lock mode: spin to find target, then backtrack
    const float SEARCH_SPEED = 0.2;        // Rotation speed while searching
    const int TARGET_DISTANCE = 30;        // cm - detection range for targets
    const float BACKTRACK_SPEED = -0.15;   // Opposite rotation speed for backtrack
    const unsigned long BACKTRACK_TIME = 100; // ms - how long to backtrack
    
    // Static variables for state machine
    static bool targetFound = false;
    static unsigned long backtrackStartTime = 0;
    
    // Get current distance (non-blocking, always fresh!)
    float distance = nbPulseIn();
    
    // State machine logic
    if (!targetFound) {
        // SEARCHING STATE
        String distanceMsg = "Scanning: " + String(distance, 1) + " cm";
        sendTelem(distanceMsg.c_str());
        
        if (distance > 0 && distance < TARGET_DISTANCE) {
            // Target detected - switch to backtrack state
            targetFound = true;
            backtrackStartTime = millis();
            
            String lockMsg = "TARGET FOUND! Starting backtrack...";
            sendTelem(lockMsg.c_str());
        } else {
            // No target - continue searching
            car.setThrottle(0.0);
            car.setSteering(SEARCH_SPEED);  // Spin right to search
        }
    } else {
        // BACKTRACKING STATE
        if (millis() - backtrackStartTime < BACKTRACK_TIME) {
            // Still backtracking
            car.setThrottle(0.0);
            car.setSteering(BACKTRACK_SPEED);  // Spin opposite direction
            
            String backMsg = "Backtracking... " + String(BACKTRACK_TIME - (millis() - backtrackStartTime)) + " ms left";
            sendTelem(backMsg.c_str());
        } else {
            // Backtrack complete - check if target is still in range
            if (distance > 0 && distance < TARGET_DISTANCE) {
                // Target still in range - maintain lock
                car.setThrottle(0.0);
                car.setSteering(0.0);
                
                String finalMsg = "TARGET LOCKED at " + String(distance, 1) + " cm!";
                sendTelem(finalMsg.c_str());
            } else {
                // Target lost or out of range - resume searching
                targetFound = false;  // Reset state to start searching again
                
                String lostMsg = "Target lost (" + String(distance, 1) + " cm) - resuming search...";
                sendTelem(lostMsg.c_str());
            }
        }
    }
}

void headbutt() {
    // Headbutt mode: spin to find target, then stabilize and ram it!
    const float SEARCH_SPEED = 0.2;        // Rotation speed while searching
    const float ATTACK_SPEED = 0.6;        // Forward speed when charging
    const int TARGET_DISTANCE = 30;        // cm - detection range for targets
    const float CORRECTION_SPIN = 0.0;     // Spin speed during correction (0 = just wait)
    const unsigned long CORRECTION_TIME = 2000; // ms - stabilization time before ramming
    
    // Static variables for state machine
    static bool targetFound = false;
    static unsigned long correctionStartTime = 0;
    
    // Get current distance (non-blocking, always fresh!)
    float distance = nbPulseIn();
    
    // State machine logic
    if (!targetFound) {
        // SEARCHING STATE
        String searchMsg = "Searching for target: " + String(distance, 1) + " cm";
        sendTelem(searchMsg.c_str());
        
        if (distance > 0 && distance < TARGET_DISTANCE) {
            // Target detected - switch to correction state
            targetFound = true;
            correctionStartTime = millis();
            
            String foundMsg = "TARGET FOUND! Stabilizing for attack...";
            sendTelem(foundMsg.c_str());
        } else {
            // No target - continue searching
            car.setThrottle(0.0);
            car.setSteering(SEARCH_SPEED);  // Spin right to search
        }
    } else {
        // CORRECTION STATE
        if (millis() - correctionStartTime < CORRECTION_TIME && distance > 0 && distance < TARGET_DISTANCE) {
            // Still correcting/stabilizing AND target still in range
            car.setThrottle(0.0);
            car.setSteering(CORRECTION_SPIN);  // Usually 0 for stabilization
            
            String corrMsg = "Stabilizing... " + String(CORRECTION_TIME - (millis() - correctionStartTime)) + " ms left";
            sendTelem(corrMsg.c_str());
        } else {
            // Correction complete OR target moved - check final target status
            if (distance > 0 && distance < TARGET_DISTANCE) {
                // Target still in range - CHARGE!!!
                car.setThrottle(ATTACK_SPEED);
                car.setSteering(0.0);  // Go straight
                
                String attackMsg = "CHARGING TARGET at " + String(distance, 1) + " cm!";
                sendTelem(attackMsg.c_str());
            } else {
                // Target lost or out of range - resume searching
                targetFound = false;  // Reset state to start searching again
                
                String lostMsg = "Target lost (" + String(distance, 1) + " cm) - resuming search...";
                sendTelem(lostMsg.c_str());
            }
        }
    }
}

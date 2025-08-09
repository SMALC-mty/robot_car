// MODE FUNCTIONS
void dance() {
    car.setSteering(0.5f);    // left
    delay(500);
    car.setSteering(-0.5f); // right
    delay(500);
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

void followLine() {
    // Constant settings - tuned for curvy tracks with chained 90° turns
    const unsigned long LOST_TIMEOUT_MS = 600;   // longer timeout for curve navigation
    const float BASE_THROTTLE = 0.35;             // reduced speed for better curve control
    const float BASE_STEERING = 0.2;             // increased steering authority for tight turns
    const float ALIGN_THRESHOLD = 0;           // allow more aggressive reversing when misaligned

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

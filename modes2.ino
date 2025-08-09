void wanderCage() {
    // Constant settings
    const float THROTTLE = 0.3;
    const float STEERING = 0.3;
    const float BOUNDARY_REVERSE = -0.2;    // Reverse throttle when backing from boundary
    const unsigned long SPIN_TIME_MS = 300;
    
    // Static variables for cage wandering state
    static unsigned long spinStartTime = 0;
    static int spinDirection = 0;    // -1 = right, +1 = left
    
    // Check if we're still within spin timeout
    if (millis() - spinStartTime < SPIN_TIME_MS) {
        // Continue spinning in the chosen direction
        car.setThrottle(BOUNDARY_REVERSE);  // Back away while spinning
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
            
            car.setThrottle(BOUNDARY_REVERSE);  // Back away from boundary while spinning
            car.setSteering(STEERING * spinDirection);
        } else {
            // No boundary detected - go straight ahead
            car.setThrottle(THROTTLE);
            car.setSteering(0.0);
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

void sumo() {
    // Sumo mode: find and ram targets while staying within cage boundaries
    const float SEARCH_SPEED = 0.2;        // Rotation speed while searching
    const float ATTACK_SPEED = 0.3;        // Forward speed when charging
    const int TARGET_DISTANCE = 40;        // cm - detection range for targets
    const float CORRECTION_SPIN = 0.0;     // Spin speed during correction (0 = just wait)
    const unsigned long CORRECTION_TIME = 300; // ms - shorter stabilization for aggressive sumo
    
    // Boundary avoidance constants (IDENTICAL to wanderCage)
    const float BOUNDARY_THROTTLE = 0.3;   // Forward speed when moving away from boundary
    const float BOUNDARY_STEERING = 0.3;   // Spin speed when avoiding boundaries
    const float BOUNDARY_REVERSE = -0.2;   // Reverse throttle when backing from boundary
    const unsigned long BOUNDARY_SPIN_TIME = 300; // ms - how long to spin away from boundary
    const unsigned long BOUNDARY_STRAIGHT_TIME = 2000; // ms - how long to go straight after spinning
    
    // Static variables for state machine
    static bool targetFound = false;
    static unsigned long correctionStartTime = 0;
    static unsigned long boundarySpinStartTime = 0;
    static unsigned long boundaryStraightStartTime = 0;
    static int spinDirection = 1;    // Initialize to left spin (1 = left, -1 = right)
    
    // Get current distance (always needed for target detection)
    float distance = nbPulseIn();
    
    // PHASE 1: Boundary spin avoidance (IDENTICAL to wanderCage)
    if (millis() - boundarySpinStartTime < BOUNDARY_SPIN_TIME) {
        // Continue spinning away from boundary - abort any target pursuit
        targetFound = false;  // Disengage target while avoiding boundary
        car.setThrottle(BOUNDARY_REVERSE);  // Back away while spinning
        car.setSteering(BOUNDARY_STEERING * spinDirection);
        
        String spinMsg = "Phase 1: Spinning away from boundary...";
        sendTelem(spinMsg.c_str());
        return;  // Skip all other logic while spinning
    }
    
    // PHASE 2: Go straight to build safe distance from boundary
    else if (millis() - boundaryStraightStartTime < BOUNDARY_STRAIGHT_TIME) {
        // Check for boundary hit during straight movement
        bool leftSensor = digitalRead(leftSensorPin);
        bool rightSensor = digitalRead(rightSensorPin);
        
        if (leftSensor || rightSensor) {
            // Hit boundary during Phase 2 - restart Phase 1 with fresh timers
            targetFound = false;  // Disengage any target pursuit
            boundarySpinStartTime = millis();        // Fresh spin timer
            boundaryStraightStartTime = millis();    // Fresh straight timer
            
            if (leftSensor && !rightSensor) {
                spinDirection = -1;    // Left sensor hit - spin right (away from boundary) 
            } else {
                // Right sensor hit OR both sensors hit - spin left (away from boundary)
                spinDirection = 1;
            }
            
            car.setThrottle(BOUNDARY_REVERSE);  // Back away from boundary while spinning
            car.setSteering(BOUNDARY_STEERING * spinDirection);
            
            String restartMsg = "BOUNDARY HIT during Phase 2! Restarting recovery...";
            sendTelem(restartMsg.c_str());
            return;  // Exit early to start Phase 1 again
        }
        
        // No boundary hit - continue going straight to build safe distance
        targetFound = false;  // Don't hunt targets while building safe distance
        car.setThrottle(BOUNDARY_THROTTLE);  // Go straight like wanderCage
        car.setSteering(0.0);
        
        String straightMsg = "Phase 2: Building safe distance from boundary...";
        sendTelem(straightMsg.c_str());
        return;  // Skip target hunting while building distance
    }
    
    // PHASE 3: Normal operation - read sensors and check for boundaries
    else {
        bool leftSensor = digitalRead(leftSensorPin);
        bool rightSensor = digitalRead(rightSensorPin);
        
        // Check if we hit a boundary
        if (leftSensor || rightSensor) {
            // Hit boundary - start two-phase recovery
            targetFound = false;  // Disengage any target pursuit
            boundarySpinStartTime = millis();
            boundaryStraightStartTime = millis();  // Both timers start together
            
            if (leftSensor && !rightSensor) {
                spinDirection = -1;    // Left sensor hit - spin right (away from boundary) 
            } else {
                // Right sensor hit OR both sensors hit - spin left (away from boundary)
                spinDirection = 1;
            }
            
            car.setThrottle(BOUNDARY_REVERSE);  // Back away from boundary while spinning
            car.setSteering(BOUNDARY_STEERING * spinDirection);
            
            String boundaryMsg = "BOUNDARY HIT! Starting two-phase recovery...";
            sendTelem(boundaryMsg.c_str());
            return;  // Exit early, boundary takes priority
        }
        
        // No boundary detected - proceed with target hunting
        if (!targetFound) {
            // SEARCHING STATE - scan in direction of last boundary spin
            String searchMsg = "Sumo hunting: " + String(distance, 1) + " cm";
            sendTelem(searchMsg.c_str());
            
            if (distance > 0 && distance < TARGET_DISTANCE) {
                // Target detected - switch to correction state
                targetFound = true;
                correctionStartTime = millis();
                
                String foundMsg = "SUMO TARGET! Preparing to ram...";
                sendTelem(foundMsg.c_str());
            } else {
                // No target - continue searching in direction of last boundary spin
                car.setThrottle(0.0);
                car.setSteering(SEARCH_SPEED * spinDirection);  // Maintain spin direction
            }
        } else {
            // CORRECTION & ATTACK STATE
            if (millis() - correctionStartTime < CORRECTION_TIME && distance > 0 && distance < TARGET_DISTANCE) {
                // Still correcting/stabilizing AND target still in range
                car.setThrottle(0.0);
                car.setSteering(CORRECTION_SPIN);  // Usually 0 for stabilization
                
                String corrMsg = "Targeting... " + String(CORRECTION_TIME - (millis() - correctionStartTime)) + " ms";
                sendTelem(corrMsg.c_str());
            } else {
                // Correction complete OR target moved - check final target status
                if (distance > 0 && distance < TARGET_DISTANCE) {
                    // Target still in range - SUMO CHARGE!!!
                    car.setThrottle(ATTACK_SPEED);
                    car.setSteering(0.0);  // Go straight
                    
                    String attackMsg = "SUMO RAMMING at " + String(distance, 1) + " cm!";
                    sendTelem(attackMsg.c_str());
                } else {
                    // Target lost or out of range - resume searching
                    targetFound = false;  // Reset state to start searching again
                    
                    String lostMsg = "Target escaped - resuming hunt...";
                    sendTelem(lostMsg.c_str());
                }
            }
        }
    }
}

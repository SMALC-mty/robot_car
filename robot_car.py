import serial
import time
import threading

class RobotCar:
    def __init__(self, port="COM7", baud_rate=9600, timeout=1, on_message=None):
        """
        Initialize RobotCar client
        
        Args:
            port: COM port for Bluetooth connection
            baud_rate: Serial communication speed
            timeout: Serial timeout in seconds
            on_message: Callback function for received messages (optional)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.on_message = on_message
        
        # State tracking for smart change detection
        self.prev_throttle = None
        self.prev_steering = None
        self.prev_throttle_speed = None
        self.prev_steering_speed = None
        self.prev_mode = None
        self.prev_trim = None
        
        # Rate limiting
        self.last_send_time = 0
        self.min_send_interval = 0.05  # 50ms minimum between sends
        
        # Background reading thread
        self._running = False
        self._read_thread = None
        
        # Connect to the robot
        self._connect()
    
    def _connect(self):
        """Establish serial connection to the robot"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            print(f"Connected to robot car on {self.port}")
            self._start_reading()
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            raise
    
    def _start_reading(self):
        """Start background thread to read serial messages"""
        if self.on_message and not self._running:
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
    
    def _read_loop(self):
        """Background thread loop to read serial messages"""
        while self._running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line and self.on_message:
                        self.on_message(line)
                time.sleep(0.01)  # Small delay to prevent busy waiting
            except Exception:
                # Silently handle read errors (connection issues, etc.)
                break
    
    def _send_command(self, command, force=False):
        """
        Send a command to the robot with rate limiting
        
        Args:
            command: String command to send
            force: If True, bypass rate limiting
        """
        if not self.serial or not self.serial.is_open:
            print("Serial connection not available")
            return False
            
        current_time = time.time()
        if not force and (current_time - self.last_send_time) < self.min_send_interval:
            return False  # Rate limited
            
        try:
            self.serial.write(command.encode())
            self.last_send_time = current_time
            return True
        except Exception as e:
            print(f"Error sending command '{command}': {e}")
            return False
    
    def _get_direction_command(self, throttle, steering):
        """Convert throttle/steering to direction character"""
        if throttle > 0 and steering == 0:      return 'F'  # Forward
        elif throttle < 0 and steering == 0:    return 'B'  # Backward  
        elif throttle == 0 and steering > 0:    return 'L'  # Rotate Left
        elif throttle == 0 and steering < 0:    return 'R'  # Rotate Right
        elif throttle > 0 and steering > 0:     return 'G'  # Forward + Left
        elif throttle > 0 and steering < 0:     return 'H'  # Forward + Right
        elif throttle < 0 and steering > 0:     return 'I'  # Backward + Left
        elif throttle < 0 and steering < 0:     return 'J'  # Backward + Right
        elif throttle == 0 and steering == 0:   return 'S'  # Stop
        else: return None
    
    def move(self, throttle, steering, force=False):
        """
        Move the robot with throttle and steering values
        
        Args:
            throttle: -1.0 to 1.0 (negative = reverse, positive = forward)
            steering: -1.0 to 1.0 (negative = right, positive = left)
            force: If True, send command even if values haven't changed
        """
        # Clamp values to valid ranges
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))
        
        # Convert to discrete direction values (-1, 0, 1)
        thr_dir = -1 if throttle < -0.05 else 1 if throttle > 0.05 else 0
        st_dir = -1 if steering < -0.05 else 1 if steering > 0.05 else 0
        
        # Convert throttle/steering magnitudes to speed values (0.0 to 1.0)
        throttle_speed = abs(throttle)
        steering_speed = abs(steering)
        
        # Check if anything actually changed
        changed = (force or 
                  self.prev_throttle != thr_dir or 
                  self.prev_steering != st_dir or
                  self.prev_throttle_speed != throttle_speed or
                  self.prev_steering_speed != steering_speed)
        
        if not changed:
            return False
        
        # Get direction command
        direction = self._get_direction_command(thr_dir, st_dir)
        if direction is None:
            return False
        
        # Convert speeds to command characters (0-9 for throttle, !-* for steering)
        throttle_digit = min(9, int(throttle_speed * 10))
        throttle_char = str(throttle_digit)
        
        steering_digit = min(9, int(steering_speed * 10))
        steering_char = chr(ord('!') + steering_digit)
        
        # Build and send command
        command = direction + throttle_char + steering_char
        
        if self._send_command(command, force):
            # Update state tracking
            self.prev_throttle = thr_dir
            self.prev_steering = st_dir
            self.prev_throttle_speed = throttle_speed
            self.prev_steering_speed = steering_speed
            
            return True
        
        return False
    
    def set_mode(self, mode, force=False):
        """
        Set robot operating mode
        
        Args:
            mode: Mode number 0-9
            force: If True, send command even if mode hasn't changed
        """
        # Validate mode number
        if not isinstance(mode, int) or mode < 0 or mode > 9:
            return False
        
        # Convert mode number to character (internal protocol)
        mode_char = chr(ord('a') + mode)
        
        # Check if mode actually changed
        if not force and self.prev_mode == mode_char:
            return False
        
        if self._send_command(mode_char, force):
            self.prev_mode = mode_char
            return True
        
        return False
    
    def set_trim(self, trim_value, force=False):
        """
        Set motor trim adjustment
        
        Args:
            trim_value: -9 to +9 trim adjustment (0 = no trim)
            force: If True, send command even if trim hasn't changed
        """
        # Clamp trim value
        trim_value = max(-9, min(9, int(trim_value)))
        
        # Convert to trim character (s = center, k-r = negative, t-~ = positive)
        trim_char = chr(ord('s') + trim_value)
        
        # Check if trim actually changed
        if not force and self.prev_trim == trim_char:
            return False
        
        if self._send_command(trim_char, force):
            self.prev_trim = trim_char
            return True
        
        return False
    
    def stop(self, force=True):
        """
        Emergency stop - always forces the command
        
        Args:
            force: If True (default), bypass change detection and rate limiting
        """
        return self.move(0, 0, force=force)
    
    def disconnect(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.stop(force=True)  # Stop the robot before disconnecting
            time.sleep(0.1)  # Give time for stop command to be processed
            
            # Stop the reading thread
            self._running = False
            if self._read_thread and self._read_thread.is_alive():
                self._read_thread.join(timeout=1)
            
            self.serial.close()
            print("Disconnected from robot car")
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup"""
        self.disconnect()
    
    def is_connected(self):
        """Check if serial connection is active"""
        return self.serial and self.serial.is_open
    
    def get_connection_info(self):
        """Get connection information"""
        return {
            'port': self.port,
            'baud_rate': self.baud_rate,
            'connected': self.is_connected(),
            'last_send_time': self.last_send_time
        }

# Example usage
if __name__ == '__main__':
    def handle_robot_message(message):
        print(message)
    
    car = RobotCar(on_message=handle_robot_message)
    pass # Start a REPL session here

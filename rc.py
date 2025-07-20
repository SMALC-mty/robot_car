import os
import math
import time
import serial
from input_manager.input_man import is_pressed, get_axis, rising_edge

def get_robot_command(throttle, steering):
    """Convert throttle/steering values to robot command characters"""
    # Get the direction command
    if throttle > 0 and steering == 0:      direction = 'F'  # Forward
    elif throttle < 0 and steering == 0:    direction = 'B'  # Backward  
    elif throttle == 0 and steering > 0:    direction = 'L'  # Rotate Left
    elif throttle == 0 and steering < 0:    direction = 'R'  # Rotate Right
    elif throttle > 0 and steering > 0:     direction = 'G'  # Forward + Left
    elif throttle > 0 and steering < 0:     direction = 'H'  # Forward + Right
    elif throttle < 0 and steering > 0:     direction = 'I'  # Backward + Left
    elif throttle < 0 and steering < 0:     direction = 'J'  # Backward + Right
    elif throttle == 0 and steering == 0:   direction = 'S'  # Stop
    else: return None
    
    # Convert throttle magnitude to speed digit (0-9)
    throttle_digit = min(9, int(abs(throttle) * 10))
    throttle_speed_char = str(throttle_digit)
    
    # Convert steering magnitude to steering speed symbol (!"#$%&'()*)
    steering_digit = min(9, int(abs(steering) * 10))
    steering_speed_char = chr(ord('!') + steering_digit)  # ! + offset
    
    # Return direction + throttle speed + steering speed
    return direction + throttle_speed_char + steering_speed_char

def ctrl_input():
    # Separate slow/fast multipliers for throttle and steering
    slow_thr = 0.4
    fast_thr = 1.0
    slow_st = 0.3
    fast_st = 0.8

    # Get keyboard input
    key_thr = 1 if pr("w") else -1 if pr("s") else 0
    key_st = 1 if pr("a") else -1 if pr("d") else 0
    key_boost = 1 if pr("c") else 0

    # Get controller input
    joy_thr = get_axis('LY')
    joy_st = -get_axis('RX')
    con_boost = max(get_axis('RT'), get_axis('LT'))

    # Calculate the higher of the two absolute values (keyboard vs controller)
    thr = key_thr if abs(key_thr) > abs(joy_thr) else joy_thr
    st = key_st if abs(key_st) > abs(joy_st) else joy_st
    boost = max(con_boost, key_boost)

    # Interpolate throttle and steering speeds separately
    thr_speed = slow_thr + (fast_thr - slow_thr) * boost
    st_speed = slow_st + (fast_st - slow_st) * boost

    # Apply speed scaling
    thr *= thr_speed
    st *= st_speed

    return thr, st

def mode_input():
    """Check for mode selection keys 1-0, return mode letter a-j or None"""
    if is_pressed('t'):
        # If 't' is pressed, return None to indicate no mode change
        return None
    mode_keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0']
    
    for i, key in enumerate(mode_keys):
        if rising_edge(key):
            return chr(ord('a') + i)  # 1->a, 2->b, ..., 0->j
    
    return None

def trim_input():
    """Check for trim adjustment keys t+1-9, return trim character k-~ or None"""
    if not is_pressed('t'):
        return None
        
    trim_keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9']
    
    for i, key in enumerate(trim_keys):
        if rising_edge(key):
            # Map 1-9 to -8 to +8, with 5 as center (0)
            trim_step = (i - 4) * 2  # 1->-8, 2->-6, ..., 5->0, ..., 9->+8
            trim_char = chr(ord('s') + trim_step)  # 's' + offset
            return trim_char
    
    return None

# Initialize Bluetooth serial connection
try:
    bt_serial = serial.Serial('COM8', 9600, timeout=1)
    print("Connected to robot car on COM8")
    time.sleep(2)  # Give Arduino time to initialize
except serial.SerialException as e:
    print(f"Failed to connect to COM8: {e}")
    exit()



pr = is_pressed
try:
    prev_command = None
    last_send_time = 0
    send_interval = 0.1  # Fixed interval for sending commands
    
    while True:
        current_time = time.time()
        time_elapsed = (current_time - last_send_time) >= send_interval
        
        # Check for mode input first (always check, regardless of interval)
        mode = mode_input()
        trim = trim_input()
        
        if mode:
            bt_serial.write(mode.encode())
            print(f"Mode switch: {mode}")
            last_send_time = current_time
        elif trim:
            bt_serial.write(trim.encode())
            # Calculate trim integer for display
            trim_step = ord(trim) - ord('s')
            print(f"Trim set: {trim_step:+d}")
            last_send_time = current_time
        elif time_elapsed:
            # Regular movement commands (only when interval elapsed)
            thr, st = ctrl_input()
            command = get_robot_command(thr, st)
            
            # Only send if command actually changed (smart change detection)
            if command and command != prev_command:
                bt_serial.write(command.encode())
                print(f"Sent: {command} (thr={thr:.1f}, st={st:.1f})")
                prev_command = command
                last_send_time = current_time
        
        time.sleep(0.02)  # High frequency input polling for automatic retry
        
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    if 'bt_serial' in locals():
        bt_serial.write(b'S')  # Stop the robot
        bt_serial.close()
        print("Disconnected from robot car")

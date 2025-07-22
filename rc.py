import time
from input_manager.input_man import is_pressed, get_axis, rising_edge
from robot_car import RobotCar

def ctrl_input():
    # Dynamic base_thr adjustment with up/down arrows
    ctrl_input.base_thr = getattr(ctrl_input, 'base_thr', 0.5)
    
    if rising_edge("Key.up","DPAD_UP"):
        ctrl_input.base_thr = min(1.0, ctrl_input.base_thr + 0.1)
        print(f"Base throttle: {ctrl_input.base_thr:.1f}")
    elif rising_edge("Key.down","DPAD_DOWN"):
        ctrl_input.base_thr = max(0.1, ctrl_input.base_thr - 0.1)
        print(f"Base throttle: {ctrl_input.base_thr:.1f}")
    
    # Separate base/fast multipliers for throttle and steering
    base_thr = ctrl_input.base_thr
    fast_thr = 1.0
    base_st = 0.4
    fast_st = 0.8

    # Get keyboard input
    key_thr = 1 if is_pressed("w") else -1 if is_pressed("s") else 0
    key_st = 1 if is_pressed("a") else -1 if is_pressed("d") else 0
    key_boost = 1 if is_pressed("c") else 0

    # Get controller input
    joy_thr = get_axis('LY')
    joy_st = -get_axis('RX')
    con_boost = max(get_axis('RT'), get_axis('LT'))

    # Calculate the higher of the two absolute values (keyboard vs controller)
    thr = key_thr if abs(key_thr) > abs(joy_thr) else joy_thr
    st = key_st if abs(key_st) > abs(joy_st) else joy_st
    boost = max(con_boost, key_boost)

    # Interpolate throttle and steering speeds separately
    thr_speed = base_thr + (fast_thr - base_thr) * boost
    st_speed = base_st + (fast_st - base_st) * boost

    # Apply speed scaling
    thr *= thr_speed
    st *= st_speed

    return thr, st

def mode_input():
    """Check for mode selection keys 1-0 or gamepad buttons, return mode number 0-9 or None"""
    mode_keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0']
    mode_btns = ["A", "B", "X"]
    
    # Check keyboard keys 1-0
    for i, key in enumerate(mode_keys):
        if rising_edge(key):
            return i  # 1->0, 2->1, ..., 0->9
    
    # Check gamepad buttons for first few modes
    for i, btn in enumerate(mode_btns):
        if rising_edge(btn):
            return i  # A->0, B->1, X->2
    
    return None

def trim_input():
    """Check for trim adjustment arrow keys, return trim value -9 to +9 or None"""
    trim_input.current_trim = getattr(trim_input, 'current_trim', 0)
    
    # Check for arrow key presses
    if rising_edge("Key.right","DPAD_RIGHT"):
        trim_input.current_trim = min(9, trim_input.current_trim + 1)
        return trim_input.current_trim
    elif rising_edge("Key.left","DPAD_LEFT"):
        trim_input.current_trim = max(-9, trim_input.current_trim - 1)
        return trim_input.current_trim
    
    return None

# Main control loop
with RobotCar(on_message=lambda msg: print(msg)) as car:
    try:
        while True:
            # Check for mode input
            mode = mode_input()
            if mode is not None:
                car.set_mode(mode)
            
            # Check for trim input
            trim = trim_input()
            if trim is not None:
                car.set_trim(trim)
            
            # Regular movement commands
            thr, st = ctrl_input()
            car.move(thr, st)
            
            time.sleep(0.02)  # High frequency input polling
            
    except KeyboardInterrupt:
        print("\nExiting...")

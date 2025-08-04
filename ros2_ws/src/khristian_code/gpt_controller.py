import espnow
import network
from machine import Pin, ADC
import time
from rotary_irq_esp import RotaryIRQ
import struct

# ===== Rotary Encoder Setup =====

encoders = []
initial_values = [0, 0, 0, 0, 0, 0]  # Initialize all motors to 0

encoder_pins = [
    (22, 15), (17, 4), (18, 5), (26, 27)
]

for i, (clk, dt) in enumerate(encoder_pins):
    r = RotaryIRQ(pin_num_clk=clk,
                  pin_num_dt=dt,
                  min_val=0,
                  max_val=100,
                  reverse=False,
                  range_mode=RotaryIRQ.RANGE_BOUNDED,
                  pull_up=False)
    # Initialize encoder with current motor value for motors 0 to 3
    r.set(initial_values[i])
    encoders.append(r)

# ===== Analog Inputs =====

motor_pins = {
    "forward_backward": ADC(Pin(32)),
    "left_right": ADC(Pin(33))
}

for pin in motor_pins.values():
    pin.atten(ADC.ATTN_11DB)

# ===== Button Setup =====

btn1 = Pin(25, Pin.IN, Pin.PULL_UP)  # Toggle encoder 2 motor (motor 2 <-> 4)
btn2 = Pin(26, Pin.IN, Pin.PULL_UP)  # Toggle encoder 3 motor (motor 3 <-> 5)

# ===== Motor Values =====

motor_values = [0, 0, 0, 0, 0, 0]  # Motors 0-5

encoder2_motor_index = 2  # toggles motor 2 and 4
encoder3_motor_index = 3  # toggles motor 3 and 5

# ===== Track last encoder positions for delta calculation =====

last_vals = [encoders[i].value() for i in range(4)]

# ===== ESP-NOW Setup =====

station = network.WLAN(network.STA_IF)
station.active(True)

esp = espnow.ESPNow()
esp.active(True)

peer = b'\x3c\x8a\x1f\xa0\xdf\xc0'  # Replace with your peer MAC
esp.add_peer(peer)

# ===== Helper Functions =====

def debounce(pin):
    time.sleep_ms(20)
    return not pin.value()

def toggle_encoder2_motor():
    global encoder2_motor_index
    # Save current encoder value back to current motor
    motor_values[encoder2_motor_index] = encoders[2].value()
    # Toggle motor index
    encoder2_motor_index = 4 if encoder2_motor_index == 2 else 2
    # Set encoder position to new motor's stored value
    encoders[2].set(motor_values[encoder2_motor_index])
    print("Encoder 2 now controls motor", encoder2_motor_index)

def toggle_encoder3_motor():
    global encoder3_motor_index
    motor_values[encoder3_motor_index] = encoders[3].value()
    encoder3_motor_index = 5 if encoder3_motor_index == 3 else 3
    encoders[3].set(motor_values[encoder3_motor_index])
    print("Encoder 3 now controls motor", encoder3_motor_index)

def check_buttons():
    if not btn1.value() and debounce(btn1):
        toggle_encoder2_motor()
        time.sleep(0.3)

    if not btn2.value() and debounce(btn2):
        toggle_encoder3_motor()
        time.sleep(0.3)

def update_motors():
    global last_vals
    # Encoder 0 and 1 directly control motors 0 and 1
    for i in [0, 1]:
        val = encoders[i].value()
        delta = val - last_vals[i]
        if delta != 0:
            motor_values[i] += delta
            motor_values[i] = max(0, min(100, motor_values[i]))
            print(f"Motor[{i}] = {motor_values[i]}")
            last_vals[i] = val

    # Encoder 2 controls motor 2 or 4
    val2 = encoders[2].value()
    delta2 = val2 - last_vals[2]
    if delta2 != 0:
        motor_values[encoder2_motor_index] += delta2
        motor_values[encoder2_motor_index] = max(0, min(100, motor_values[encoder2_motor_index]))
        print(f"Motor[{encoder2_motor_index}] = {motor_values[encoder2_motor_index]}")
        last_vals[2] = val2

    # Encoder 3 controls motor 3 or 5
    val3 = encoders[3].value()
    delta3 = val3 - last_vals[3]
    if delta3 != 0:
        motor_values[encoder3_motor_index] += delta3
        motor_values[encoder3_motor_index] = max(0, min(100, motor_values[encoder3_motor_index]))
        print(f"Motor[{encoder3_motor_index}] = {motor_values[encoder3_motor_index]}")
        last_vals[3] = val3

def read_inputs():
    """
    Pack all 6 motor values (0-100) and 2 analog inputs into bytes to send via ESP-NOW.
    """
    values = motor_values[:]  # Copy motor values

    # Read analog inputs and append
    for pin in motor_pins.values():
        values.append(pin.read())

    # Pack 8 unsigned shorts: 6 motors + 2 analog inputs
    packed = struct.pack('8H', *values)
    return packed

# ===== Main Loop =====

while True:
    check_buttons()
    update_motors()

    data = read_inputs()
    print("Sending:", data)
    esp.send(peer, data)

    time.sleep(0.05)

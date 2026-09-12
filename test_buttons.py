import RPi.GPIO as GPIO
import time

BUTTONS = {
    "BUTTON 1": 16,
    "BUTTON 2": 20,
    "BUTTON 3": 21,
}

GPIO.setmode(GPIO.BCM)

for pin in BUTTONS.values():
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("================================")
print("      EchoMap Button Test")
print("================================")
print("Press each button.")
print("Press Ctrl+C to exit.\n")

last_state = {name: GPIO.input(pin) for name, pin in BUTTONS.items()}

try:
    while True:
        for name, pin in BUTTONS.items():
            state = GPIO.input(pin)

            # Button pressed
            if state == GPIO.LOW and last_state[name] == GPIO.HIGH:
                print(f"{name} PRESSED (GPIO {pin})")

            # Button released
            if state == GPIO.HIGH and last_state[name] == GPIO.LOW:
                print(f"{name} RELEASED (GPIO {pin})")

            last_state[name] = state

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nTest stopped.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
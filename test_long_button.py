import RPi.GPIO as GPIO
import time

PIN = 16
LONG_PRESS = 3.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Button 1 long-press test")
print("Hold Button 1 for 3+ seconds, then release.")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        if GPIO.input(PIN) == GPIO.LOW:
            start = time.time()

            while GPIO.input(PIN) == GPIO.LOW:
                duration = time.time() - start

                if duration >= LONG_PRESS:
                    print(f"LONG PRESS detected: {duration:.1f} seconds")
                    break

                time.sleep(0.05)

            # Wait for release
            while GPIO.input(PIN) == GPIO.LOW:
                time.sleep(0.05)

            if duration < LONG_PRESS:
                print(f"SHORT PRESS detected: {duration:.1f} seconds")

            print()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    GPIO.cleanup()
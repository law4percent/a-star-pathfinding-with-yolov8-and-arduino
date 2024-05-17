import time

# Constants
ledPin1 = 13  # Placeholder for the first LED pin
ledPin2 = 12  # Placeholder for the second LED pin
sensorPin = "A0"  # Placeholder for the sensor pin

interval1 = 1  # Interval for the first LED (1 second)
interval2 = 0.5  # Interval for the second LED (0.5 second)
interval3 = 2  # Interval for reading the sensor (2 seconds)

# Variables to store the last time each task was executed
previousMillis1 = 0
previousMillis2 = 0
previousMillis3 = 0

# States for LEDs (to simulate digitalWrite and digitalRead)
ledState1 = False
ledState2 = False

def setup():
    # Placeholder setup, initialize states and hardware setup here if needed
    print("Setup complete")

def loop():
    global previousMillis1, previousMillis2, previousMillis3
    global ledState1, ledState2
    
    currentMillis = time.time()

    # Check if it's time to toggle the first LED
    if currentMillis - previousMillis1 >= interval1:
        previousMillis1 = currentMillis
        ledState1 = not ledState1
        print(f"LED 1 state: {'ON' if ledState1 else 'OFF'}")

    # Check if it's time to toggle the second LED
    if currentMillis - previousMillis2 >= interval2:
        previousMillis2 = currentMillis
        ledState2 = not ledState2
        print(f"LED 2 state: {'ON' if ledState2 else 'OFF'}")

    # Check if it's time to read the sensor
    if currentMillis - previousMillis3 >= interval3:
        previousMillis3 = currentMillis
        # Simulate sensor reading
        sensorValue = read_sensor(sensorPin)
        print(f"Sensor value: {sensorValue}")

def read_sensor(pin):
    # Placeholder function to simulate sensor reading
    # Replace this with actual sensor reading logic
    return 42  # Example sensor value

if __name__ == "__main__":
    setup()
    try:
        while True:
            loop()
            time.sleep(0.01)  # Small delay to prevent 100% CPU usage
    except KeyboardInterrupt:
        print("Program terminated")
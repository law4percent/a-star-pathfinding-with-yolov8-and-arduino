# import serial

# arduino_port = 'COM11'
# baud_rate = 9600
# myArduino = serial.Serial(arduino_port, baud_rate, timeout=1)

# def sendCommandToArduino(raw_data):
#     command = f"{raw_data}\n"
#     myArduino.write(command.encode('utf-8'))

import serial
import time

serial_port = 'COM3'  # Change this to match your Arduino's serial port
baud_rate = 9600  # Match this to your Arduino's baud rate

# Initialize the serial connection
arduino = serial.Serial(serial_port, baud_rate)
print("Serial connection established.")

try:
   while True:
        # Send a command to the Arduino
        command = "LED_ON"  # Example command to turn on an LED
        arduino.write(command.encode('utf-8'))
        print("Sent command to Arduino:", command)

        time.sleep(1)  # Wait for 1 second before sending the next command

except KeyboardInterrupt:
    print("\nExiting program.")
    arduino.close()  # Close the serial connection when the program exits


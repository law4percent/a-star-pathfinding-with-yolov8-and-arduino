import serial
import time

serial_port = 'COM20'  # Change this to match your Arduino's serial port
baud_rate = 9600  # Match this to your Arduino's baud rate

# Initialize the serial connection
arduino = serial.Serial(serial_port, baud_rate)
print("Serial connection established.")

def sendCommandToArduino(command):
    command += '\n'  # Ensure the command ends with a newline character
    arduino.write(command.encode('utf-8'))
    print("Sent command to Arduino:", command.strip())

def decodeReceivedDataCommand():
    received_command = arduino.readline().decode('utf-8').rstrip()
    print(f"Received: {received_command}")
    return received_command

while True:
    # Send a command to the Arduino
    command = input("Enter data: ")
    sendCommandToArduino(command)
    time.sleep(1)

    if arduino.in_waiting > 0:
        received_command = decodeReceivedDataCommand()

arduino.close()

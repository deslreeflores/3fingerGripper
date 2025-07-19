import serial
import time
import sys

if len(sys.argv) < 2:
    print("Usage: python serialNEW.py [open|close|stop]")
    sys.exit(1)

command = sys.argv[1].lower()
if command not in ["open", "close", "stop"]:
    print("Invalid command. Use 'open', 'close', or 'stop'.")
    sys.exit(1)

# starts serial communication on port 7
ser = serial.Serial('COM7', 9600, timeout=2)  # Replace with your RS485 port
time.sleep(2)

# command
ser.write((command + '\n').encode())
time.sleep(0.2)

#print response back 
response = ser.read_all()
print("Echoed:", response)

ser.close()

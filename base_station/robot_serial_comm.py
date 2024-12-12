import serial

# Set up the serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    print("Connected to /dev/ttyUSB0 at 9600 baud.")
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    exit(1)

try:
    while True:
        # Prompt the user for input
        user_input = input("Enter a number (1-9) to send over serial, or 'q' to quit.\n1: Winch IN\n2: Winch STOP\n3: Winch OUT\n4: Tether RELEASE\n5: Tether CLOSE\n6: Stabilize ON\n7: Stabilize OFF\n8: Motion FORWARD\n9: Motion STOP\n").strip()

        # Check for quit condition
        if user_input.lower() == 'q':
            print("Exiting program.")
            break

        # Validate and send the input
        if user_input in {'1', '2', '3', '4', '5', '6', '7', '8', '9'}:
            ser.write(user_input.encode())  # Send as bytes
            print(f"Sent: {user_input}\n")
        else:
            print("Invalid input. Please enter 1, 2, or 3.")
except KeyboardInterrupt:
    print("\nProgram interrupted.")
finally:
    # Close the serial connection
    ser.close()
    print("Serial port closed.")


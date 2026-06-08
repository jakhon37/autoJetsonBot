import serial
import time
import sys
import glob

def find_serial_port():
    ports = glob.glob('/dev/cu.usbmodem*')
    return ports[0] if ports else None

def test_verbose():
    port = find_serial_port()
    if not port:
        print("❌ No ESP32 found.")
        return

    print(f"🔌 Verbose Test on {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(2)
        
        # Flush initial junk
        ser.reset_input_buffer()
        
        print("💡 Toggling LED every 2 seconds. Watch the board!")
        print("Press Ctrl+C to stop.")
        
        state = 1
        while True:
            cmd = f"l {state}\n"
            print(f"➡️ Sending: {cmd.strip()}")
            ser.write(cmd.encode())
            
            # Read for 2 seconds and print EVERYTHING
            start = time.time()
            while time.time() - start < 2:
                if ser.in_waiting:
                    raw = ser.readline()
                    try:
                        line = raw.decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"⬅️ Received: {line}")
                    except:
                        print(f"⬅️ Raw: {raw}")
                time.sleep(0.01)
            
            state = 1 - state # Toggle
            
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals(): ser.close()

if __name__ == "__main__":
    test_verbose()

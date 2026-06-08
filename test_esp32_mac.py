import serial
import time
import sys
import glob

def find_serial_port():
    """Finds the first available serial port on macOS."""
    ports = glob.glob('/dev/cu.usbmodem*')
    if not ports:
        return None
    return ports[0]

def test_connection():
    port = find_serial_port()
    if not port:
        print("❌ Error: No ESP32 (usbmodem) found on Mac.")
        sys.exit(1)
    
    print(f"🔌 Connecting to ESP32 on {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2) # Wait for reset
        
        # Test LED ON
        print("💡 Sending: LED ON (l 1)")
        ser.write(b'l 1\n')
        time.sleep(1)
        
        # Test LED OFF
        print("🌑 Sending: LED OFF (l 0)")
        ser.write(b'l 0\n')
        time.sleep(1)
        
        # Check for telemetry
        print("📊 Waiting for telemetry...")
        for _ in range(10):
            line = ser.readline().decode('utf-8').strip()
            if line.startswith('e'):
                print(f"✅ Received telemetry: {line}")
                break
            time.sleep(0.1)
        else:
            print("⚠️ Warning: No telemetry received.")

        ser.close()
        print("🚀 Host-side test complete.")
        
    except Exception as e:
        print(f"❌ Serial Error: {e}")

if __name__ == "__main__":
    test_connection()

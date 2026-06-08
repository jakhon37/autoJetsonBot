import serial
import time
import sys

def test_connection():
    # Inside Docker, we expect robotmac.sh to map it to /dev/ttyACM0
    port = '/dev/ttyACM0'
    
    print(f"🐳 Docker: Connecting to virtual device {port}...")
    try:
        # Note: Baud rate is handled by socat on the host, 
        # but we set it here for completeness.
        ser = serial.Serial(port, 115200, timeout=1)
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test LED sequence
        print("💡 Sending: LED ON (l 1)")
        ser.write(b'l 1\n')
        time.sleep(1)
        
        print("🌑 Sending: LED OFF (l 0)")
        ser.write(b'l 0\n')
        time.sleep(1)
        
        # Verify telemetry bridge
        print("📊 Verifying telemetry bridge...")
        count = 0
        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"⬅️ Received from Bridge: {line}")
                    if line.startswith('e'):
                        count += 1
                        if count >= 2: break
            time.sleep(0.1)
        
        if count == 0:
            print("❌ Error: No data received through bridge. Check robotmac.sh status.")
            sys.exit(1)

        ser.close()
        print("🎉 Docker-to-ESP32 bridge test SUCCESSFUL.")
        
    except Exception as e:
        print(f"❌ Bridge Error: {e}")
        print("\nPossible fixes:")
        print("1. Run './robotmac.sh up' on the host.")
        print("2. Ensure the ESP32 is plugged into the Mac.")
        print("3. Check 'docker ps' to ensure the container name matches.")

if __name__ == "__main__":
    test_connection()

import serial
import serial.tools.list_ports
import time
import datetime
import struct
import sys
import os

BAUD_RATE = 921600

def find_arduino_port():
    """Auto-detect the Arduino R4 WiFi serial port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        # R4 WiFi shows up as "Arduino UNO R4 WiFi" or similar
        if "arduino" in desc or "ACM" in port.device:
            return port.device
    # Fallback: if there's only one port, use it
    if len(ports) == 1:
        return ports[0].device
    return None

def capture_single_image(port_name):
    try:
        # Use a longer timeout (2s) so reads don't return prematurely
        # during slow WiFi-to-Serial transfers
        ser = serial.Serial(port_name, BAUD_RATE, timeout=2)
        print(f"Connected to Arduino on {port_name} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        return

    # Force-reset the Arduino R4 WiFi by toggling DTR.
    # On a Raspberry Pi, simply opening the port may NOT auto-reset the board
    # (unlike the Arduino IDE which always does this).
    print("Resetting Arduino via DTR toggle...")
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(2)  # Give the board time to reboot and run setup()
    ser.reset_input_buffer()  # Discard any garbage from the reset

    print("Waiting for Arduino to connect to Wi-Fi (this may take 10-20 seconds)...")
    
    # --- Phase 1: Wait for SYSTEM_READY ---
    timeout_start = time.time()
    while True:
        if time.time() - timeout_start > 60:
            print("\nError: Timed out waiting for Arduino to initialize (60s).")
            print("Troubleshooting:")
            print("  1. Did you flash the updated .ino sketch to the Arduino R4 WiFi?")
            print("  2. Is the ELEGOO ESP32-CAM powered on and broadcasting its WiFi?")
            print("  3. Try pressing the Arduino's reset button, then re-run this script.")
            ser.close()
            return
        
        raw = ser.readline()
        if raw:
            line = raw.decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[Arduino Startup] {line}")
                if "SYSTEM_READY" in line:
                    break
    
    print("\nArduino is ready! Sending trigger command...")
    time.sleep(0.5) 
    
    # Flush anything left in the buffer, then send trigger
    ser.reset_input_buffer()
    ser.write(b'S')
    ser.flush()

    print("Waiting for image data...")

    # --- Phase 2: Read debug lines until we see the binary magic header ---
    # We read line-by-line until we encounter the magic bytes.
    # The magic bytes (0xFF 0xAA 0xBB 0xCC) will NOT appear in normal UTF-8 text.
    
    buffer = bytearray()
    timeout_start = time.time()
    
    try:
        while True:
            if time.time() - timeout_start > 20:
                print("Error: Timed out waiting for image.")
                break

            # Read available data
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(waiting)
            else:
                # Small blocking read to avoid busy-spin
                chunk = ser.read(1)
                if not chunk:
                    continue
            
            buffer.extend(chunk)

            # Look for the magic header in the buffer
            magic_idx = buffer.find(b'\xff\xaa\xbb\xcc')
            
            if magic_idx == -1:
                # No magic header yet — everything before this point is text
                # Print any complete text lines
                while True:
                    nl = buffer.find(b'\n')
                    if nl == -1:
                        break
                    line = buffer[:nl].decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino] {line}")
                    del buffer[:nl+1]
                continue
            
            # We found the magic header!
            # Print any text lines BEFORE the magic header
            if magic_idx > 0:
                text_before = buffer[:magic_idx].decode('utf-8', errors='ignore').strip()
                if text_before:
                    for tl in text_before.split('\n'):
                        tl = tl.strip()
                        if tl:
                            print(f"[Arduino] {tl}")
            
            # Trim buffer to start at the magic header
            buffer = buffer[magic_idx:]
            
            # We need at least 8 bytes (4 magic + 4 length)
            while len(buffer) < 8:
                more = ser.read(8 - len(buffer))
                if not more:
                    continue
                buffer.extend(more)
            
            # Parse image length (big-endian uint32)
            img_length = struct.unpack('>I', buffer[4:8])[0]
            print(f"[Arduino] Image header received, expecting {img_length} bytes...")
            
            # Read the image body
            body_start = 8
            total_needed = body_start + img_length
            
            while len(buffer) < total_needed:
                remaining = total_needed - len(buffer)
                # Read in chunks, with a safety timeout
                to_read = min(remaining, 4096)
                more = ser.read(to_read)
                if more:
                    buffer.extend(more)
                    pct = (len(buffer) - body_start) / img_length * 100
                    print(f"\r  Progress: {len(buffer) - body_start}/{img_length} bytes ({pct:.0f}%)", end='', flush=True)
                    
            print()  # newline after progress
            
            img_data = buffer[body_start:body_start + img_length]
            
            # Validate JPEG magic bytes
            if img_data[:2] == b'\xff\xd8':
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"capture_{timestamp}.jpg"
                with open(filename, 'wb') as f:
                    f.write(img_data)
                print(f"\n[SUCCESS] Image saved to: {os.path.abspath(filename)} ({img_length} bytes)")
            else:
                print(f"\n[ERROR] Received {img_length} bytes but data doesn't start with JPEG header (got: {img_data[:4].hex()})")
                # Save it anyway for debugging
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"capture_debug_{timestamp}.bin"
                with open(filename, 'wb') as f:
                    f.write(img_data)
                print(f"  Debug data saved to: {filename}")
            break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else find_arduino_port()
        
    if port:
        capture_single_image(port)
    else:
        print("Could not find an Arduino serial port automatically.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
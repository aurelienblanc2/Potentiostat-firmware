import serial #Need pip install pyserial
import serial.tools.list_ports
import subprocess
import time
import os
import sys

# ----------- CONFIGURATION -----------
BAUD_RATE = 115200              # Must match MCU UART config
BOOT_COMMAND = b'BOOT_DFU_MODE' # Binary send by USB
DFU_VENDOR = 2022               # VID
DFU_PRODUCT = 22099             # PID
# --------------------------------------

def find_com_port():

    ports = serial.tools.list_ports.comports()

    if not ports:
        raise ValueError(
            "No ports found, verify that the device is connected (and flashed once) then try again"
        )

    for port in ports:
        if (port.vid == DFU_VENDOR) and (port.pid == DFU_PRODUCT):
            return port.name

    raise ValueError("No compatible port found, verify that the device is connected (and flashed once) then try again")


def send_command(command):

    try:
        com_port = find_com_port()
        print(f"[USB CDC] Sending {command} command to {com_port}...")
        with serial.Serial(com_port, BAUD_RATE, timeout=2) as ser:
            ser.reset_input_buffer()
            ser.write(command)
            print("[USB CDC] Command sent.")

            # Read response
            timeout = time.time() + 3  # 2 seconds max
            while time.time() < timeout:
                if ser.in_waiting:
                    response = ser.read(ser.in_waiting).decode(errors='ignore')
                    print(f"[USB CDC] Received: {response.strip()}")
                    break
            else:
                print("[USB CDC] No response received.")

    except (serial.SerialException, ValueError):
        print(f"[!] Error opening COM port")
        sys.exit(1)

def wait_for_dfu(timeout=10):
    print(f"[DFU] Waiting for STM32 DFU device to appear on USB ({timeout}s)...")
    for i in range(timeout):
        try:
            result = subprocess.run(['dfu-util', '-l'], capture_output=True, text=True)
            if 'Found DFU' in result.stdout:
                print("[DFU] DFU device detected.")
                return True
        except FileNotFoundError:
            print("[!] dfu-util not found in PATH.")
            sys.exit(1)
        time.sleep(1)
    print("[!] DFU device not found in time.")
    sys.exit(1)

def flash_firmware():
    print("[DFU] Flashing firmware via USB...")
    flash_cmd = [
        'dfu-util',
        '-a', '0',             # Alternate setting 0 (main flash)
        '-d', f"'{DFU_VENDOR}:{DFU_PRODUCT}'",
        '-s', '0x08000000:leave',  # Start of flash + auto-leave DFU mode
        '-D', BIN_FILE_PATH
    ]
    result = subprocess.run(flash_cmd, capture_output=True, text=True)

    # Print stdout for user feedback
    if result.stdout:
        print(result.stdout)

    # Filter non-critical warnings from stderr
    critical_errors = []
    for line in result.stderr.splitlines():
        if not (
                "Invalid DFU suffix signature" in line
                or "Error during download get_status" in line
                or "A valid DFU suffix will be required in a future dfu-util release" in line
        ):
            critical_errors.append(line)

    if result.returncode != 0 and critical_errors:
        print("[!] Flashing failed with critical error(s):")
        for err in critical_errors:
            print(err)
        sys.exit(1)

    print("[DFU] Flashing complete")

def wait_for_cdc(timeout=10):
    print("[USB CDC] Waiting for new firmware to appear...")
    for _ in range(timeout):
        try:
            com_port = find_com_port()
            with serial.Serial(com_port, BAUD_RATE, timeout=1) as ser:
                print("[USB CDC] Device is back.")
                return True
        except (serial.SerialException, ValueError):
            time.sleep(1)
    raise RuntimeError("New firmware did not enumerate CDC")


if __name__ == '__main__':

    if len(sys.argv) == 2:
        BIN_FILE_PATH = sys.argv[1]
        BIN_FILE_PATH.replace("/", os.sep)
    else:
        print("[!] Arg not found")
        sys.exit(1)

    if not os.path.exists(BIN_FILE_PATH):
        print(f"[!] File not found: {BIN_FILE_PATH}")
        sys.exit(1)

    send_command(BOOT_COMMAND)
    wait_for_dfu()
    flash_firmware()
    wait_for_cdc()

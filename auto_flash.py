import serial #Need pip install pyserial
import serial.tools.list_ports
import subprocess
import time
import os
import sys

# ----------- CONFIGURATION -----------
BAUD_RATE = 115200                       # Must match MCU UART config
BOOT_COMMAND = b'BOOT_DFU_MODE'          # Binary send by USB
VENDOR_ID_POTENTIOSTAT = 2022            # VID Potentiostat
PRODUCT_ID_POTENTIOSTAT = 22099          # PID Potentiostat
DFU_VENDOR = "0483"                      # VID STMicroelectronics
DFU_PRODUCT = "df11"                     # ST DFU in FS Mode
# --------------------------------------

def find_com_port():

    ports = serial.tools.list_ports.comports()
    list_port = []

    if not ports:
        raise ValueError(
            "No ports found, verify that the device is connected (and flashed once) then try again"
        )

    for port in ports:
        if (port.vid == VENDOR_ID_POTENTIOSTAT) and (port.pid == PRODUCT_ID_POTENTIOSTAT):
            list_port.append(port.name)

    if len(list_port) == 0:
        raise ValueError("No compatible port found, verify that the device is connected (and flashed once) then try again")
    else:
        return list_port


def send_command(port, command):

    try:
        print(f"[USB CDC] Sending {command} command to {port}...")
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


def find_dfu_serial():
    result = subprocess.run(['dfu-util', '--list'], capture_output=True, text=True)
    device_blocks = result.stdout.strip().split('\n')
    list_serial = []

    for block in device_blocks:
        if (("["+DFU_VENDOR+":"+DFU_PRODUCT+"]") in block) and ('serial' in block):
            for cat in block.split(','):
                if "serial" in cat:
                    list_serial.append(cat.split('"')[-2])

    if not list_serial:
        raise ValueError(
            "No dfu found, verify that the device is connected and in bootloader mode"
        )
    else:
        list_serial = list(set(list_serial))
        return list_serial


def flash_firmware(serial_id=None):
    print("[DFU] Flashing firmware via USB...")
    if serial_id is None:
        flash_cmd = [
            'dfu-util',
            '-a', '0',             # Alternate setting 0 (main flash)
            '-d', f'{DFU_VENDOR}:{DFU_PRODUCT}',
            '-s', '0x08000000:leave',  # Start of flash + auto-leave DFU mode
            '-D', BIN_FILE_PATH
        ]
    else:
        flash_cmd = [
            'dfu-util',
            '-a', '0',             # Alternate setting 0 (main flash)
            '-d', f'{DFU_VENDOR}:{DFU_PRODUCT}',
            '-s', '0x08000000:leave',  # Start of flash + auto-leave DFU mode
            '-D', BIN_FILE_PATH,
            '-S', serial_id
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


def wait_for_cdc(nb_com_port,timeout=10):
    print("[USB CDC] Waiting for new firmware to appear...")
    for _ in range(timeout):
        try:
            list_port = find_com_port()
            if len(list_port) == nb_com_port:
                for port in list_port:
                    with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
                        print(f"[USB CDC] {port} Device is back.")
                print(f"[USB CDC] All Devices are back.")
                return True

        except (serial.SerialException, ValueError):
            time.sleep(1)
    raise RuntimeError("New firmware did not enumerate CDC")


if __name__ == '__main__':

    bSetup = 0

    if len(sys.argv) == 2:
        BIN_FILE_PATH = sys.argv[1]
        BIN_FILE_PATH.replace("/", os.sep)

    elif len(sys.argv) == 3:
        BIN_FILE_PATH = sys.argv[1]
        BIN_FILE_PATH.replace("/", os.sep)

        if sys.argv[2] == "setup":
            bSetup = 1
        else:
            print("[!] ValueError on the second argument, for the first flashing add 'setup', otherwise don't provide second argument")
            sys.exit(1)

    elif len(sys.argv) < 2:
        print("[!] Argument not found")
        sys.exit(1)

    else:
        print("[!] Too many arguments provided")
        sys.exit(1)


    if not os.path.exists(BIN_FILE_PATH):
        print(f"[!] File not found: {BIN_FILE_PATH}")
        sys.exit(1)


    if bSetup == 0:
        list_ports = find_com_port()
        for com_port in list_ports:
            send_command(com_port,BOOT_COMMAND)
            wait_for_dfu()
            flash_firmware()
        wait_for_cdc(len(list_ports))
    else:
        list_serials = find_dfu_serial()
        for serial_id in list_serials:
            flash_firmware(serial_id)
        wait_for_cdc(len(list_serials))
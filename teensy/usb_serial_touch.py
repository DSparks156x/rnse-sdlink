# usb_serial_touch.py -- PlatformIO pre-script: adds a USB_SERIAL_TOUCH type (CDC serial + HID multitouch)
# to the Teensy 4 core's usb_desc.h. Teensyduino only ships touch+SEREMU (slow HID serial), which would break
# the CDC console and USB throughput. Idempotent. The block goes FIRST in the #if chain because PlatformIO's
# teensy builder also defines USB_SERIAL for any USB type it doesn't know.
# Arduino IDE users: paste BLOCK into cores/teensy4/usb_desc.h the same way and add -DUSB_SERIAL_TOUCH.
import os

Import("env")  # noqa: F821  (PlatformIO SCons global)

MARK = "#if defined(USB_SERIAL_TOUCH)"
BLOCK = MARK + """   // added by RNS-E sdemu usb_serial_touch.py
  #define VENDOR_ID		0x16C0
  #define PRODUCT_ID		0x04D5
  #define MANUFACTURER_NAME	{'T','e','e','n','s','y','d','u','i','n','o'}
  #define MANUFACTURER_NAME_LEN	11
  #define PRODUCT_NAME		{'S','D','e','m','u',' ','T','o','u','c','h'}
  #define PRODUCT_NAME_LEN	11
  #define EP0_SIZE		64
  #define NUM_ENDPOINTS		5
  #define NUM_USB_BUFFERS	12
  #define NUM_INTERFACE		3
  #define CDC_IAD_DESCRIPTOR    1
  #define CDC_STATUS_INTERFACE	0
  #define CDC_DATA_INTERFACE	1
  #define CDC_ACM_ENDPOINT	2
  #define CDC_RX_ENDPOINT       3
  #define CDC_TX_ENDPOINT       4
  #define CDC_ACM_SIZE          16
  #define CDC_RX_SIZE_480       512
  #define CDC_TX_SIZE_480       512
  #define CDC_RX_SIZE_12        64
  #define CDC_TX_SIZE_12        64
  #define MULTITOUCH_INTERFACE  2
  #define MULTITOUCH_ENDPOINT   5
  #define MULTITOUCH_SIZE       9
  #define MULTITOUCH_FINGERS    10
  #define ENDPOINT2_CONFIG	ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT
  #define ENDPOINT3_CONFIG	ENDPOINT_RECEIVE_BULK + ENDPOINT_TRANSMIT_UNUSED
  #define ENDPOINT4_CONFIG      ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_BULK
  #define ENDPOINT5_CONFIG      ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT

"""

core = os.path.join(env.PioPlatform().get_package_dir("framework-arduinoteensy"), "cores", "teensy4")  # noqa: F821
path = os.path.join(core, "usb_desc.h")
with open(path, "r", newline="") as f:
    src = f.read()
if MARK not in src:
    anchor = "#if defined(USB_SERIAL)\n"
    if anchor not in src:
        raise SystemExit("usb_serial_touch.py: anchor not found in " + path)
    src = src.replace(anchor, BLOCK + "#elif defined(USB_SERIAL)\n", 1)
    with open(path, "w", newline="") as f:
        f.write(src)
    print("usb_serial_touch.py: patched " + path)

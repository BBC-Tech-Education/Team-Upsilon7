import csi, time
from machine import UART, LED

led = LED("LED_GREEN")
led.on()

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.VGA)

cam.snapshot(time=2000)

clock = time.clock()
clock.reset()

uart = UART(1, 115200) #timeout_char=1000
led.off()



flag_no_data =  True
flag_stop =     False
flag_vicitm =   False
flag_pack_1 =   False
flag_pack_2 =   False


def function_send(value):
    """
    Send data to the Teensy 4.1:
        10 = Drop one packages
        20 = Drop two packages
        30 = LED
        70 = Stop
        90 = No victim
    """

    uart.write(bytearray([255,value, 254]))


while True:
    clock.tick()
    img = cam.snapshot()

    function_send(90)
    time.sleep(0.05)

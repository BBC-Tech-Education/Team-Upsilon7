import csi, time, ml, tof
from ml.postprocessing.edgeimpulse import Fomo
from machine import UART

# Initialize the sensor.
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)     # or csi.GRAYSCALE
csi0.framesize(csi.QVGA)       # 320x240
csi0.window((240, 240))        # 240x240 window
# csi0.skip_frames(time=2000)    # let AE settle

# Init UART with Teensy
uart = UART(1, 115200) #timeout_char=1000

# Init LRF
tof.init()


# Load the model + Edge Impulse FOMO post-processor.
model = ml.Model("/rom/trained.tflite", postprocess=Fomo(threshold=0.5))

# Labels: ml.Model auto-loads "<model_basename>.txt" (e.g. trained.txt),
# NOT labels.txt. Either rename labels.txt -> trained.txt, or load manually:
labels = [line.rstrip('\n') for line in open("labels.txt")] if model.labels is None else model.labels

# colors = [
#     (255,   0,   0), (0, 255,   0), (255, 255,   0),
#     (0,   0, 255), (255,   0, 255), (0, 255, 255),
#     (255, 255, 255),
# ]


def function_send(value):
    """
    Send data to the Teensy 4.1:
        10 = Drop one packages
        30 = LED
        90 = No victim
    """
    uart.write(bytearray([value]))


def distance_check(max_distance):
    depth_list, depth_min, depth_max = tof.read_depth()
    average_distance = 0
    for i in range (4,7,1):
        for j in range(4,7,1):
            row, col = i, j
            index = (row * 8) + col
            average_distance += depth_list[index]

    average_distance = average_distance /16
    #print(average_distance)

    if(average_distance < max_distance):
        return 1
    else:
        return 0


times_repeated = 0
send_1 = False
send_3 = False


clock = time.clock()
while True:
    #time_1 = time.ticks_ms()
    img = csi0.snapshot()
    detected_victim = False

    # --- FOMO Detection ---
    predictions = model.predict([img])
    for i, detection_list in enumerate(predictions):
        if i == 0 or len(detection_list) == 0:
            continue
        label = labels[i].strip().upper()

        if label == "U":
            detected_victim = 1
            send_3 = 1

        elif label == "H":
            detected_victim = 1
            send_1 = 1

    if (distance_check(200) != 1):
        send_1 = 0
        send_3 = 0

    times_repeated = 2
    if times_repeated > 1:
        if send_1:  # sends a 1 because it has seen a U
            function_send(10)
            print("Sent 1 to ESP")

        elif send_3:  # sends a 2 because it has seen a H
            function_send(30)
            print("Sent 3 to ESP")

        else:  # sends a 9 because nothing is there
            function_send(90)
            print("Sent 9 to ESP")

        times_repeated = 0  # reset it
        send_1 = 0
        send_3 = 0

    #print("Time: ", (time.ticks_ms() - time_1))

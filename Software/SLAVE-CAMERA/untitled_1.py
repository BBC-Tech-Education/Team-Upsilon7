import csi, time, ml
from ml.postprocessing.edgeimpulse import Fomo

# Initialize the sensor.
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)     # or csi.GRAYSCALE
csi0.framesize(csi.QVGA)       # 320x240
csi0.window((240, 240))        # 240x240 window
#csi0.skip_frames(time=2000)    # let AE settle

# Load the model + Edge Impulse FOMO post-processor.
model = ml.Model("trained.tflite", postprocess=Fomo(threshold=0.5))

# Labels: ml.Model auto-loads "<model_basename>.txt" (e.g. trained.txt),
# NOT labels.txt. Either rename labels.txt -> trained.txt, or load manually:
labels = [line.rstrip('\n') for line in open("labels.txt")] if model.labels is None else model.labels

colors = [
    (255,   0,   0), (  0, 255,   0), (255, 255,   0),
    (  0,   0, 255), (255,   0, 255), (  0, 255, 255),
    (255, 255, 255),
]

times_repeated = 0
send_1 = False
send_2 = False
send_3 = False

clock = time.clock()
while True:
    time_1 = time.ticks_ms()
    img = csi0.snapshot()
    detected_victim = False

    if times_repeated <= 1:
        # --- FOMO Detection ---
        predictions = model.predict([img])
        for i, detection_list in enumerate(predictions):
            if i == 0 or len(detection_list) == 0:
                continue
            label = labels[i].strip().upper()

            if label == "S":
                pass

            elif label == "U":
                detected_victim = True
                send_1 = True

            elif label == "H":
                detected_victim = True
                send_2 = True

    times_repeated += 1

    if times_repeated > 1:
        if send_1:  # sends a 1 because it has seen a U
            #uart.write("5\n")
            #uart.write(bytes([255, 234, 1]))
            #esp_com.com_send_1()
            print("Sent 1 to ESP")

        elif send_2:  # sends a 2 because it has seen a H
            #uart.write("5\n")
            #uart.write(bytes([255, 234, 2]))
            #esp_com.com_send_2()
            print("Sent 2 to ESP")

        else:  # sends a 9 because nothing is there
            #uart.write("5\n")
            #uart.write(bytes([255, 234, 9]))
            #esp_com.com_send_9()
            print("Sent 9 to ESP")

        times_repeated = 0  # reset it
        send_1 = False
        send_2 = False

    print("Time: ", (time.ticks_ms() - time_1))

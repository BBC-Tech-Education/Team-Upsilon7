import tof

tof.init()

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

while True:
   print(distance_check(200))

import matplotlib.pyplot as plt

# Data for without line of sight
iterations_wlos = list(range(1, 31))
explored_area = [
    4.422500133514404, 11.710000038146973, 19.322500228881836, 26.66499900817871,
    34.3224983215332, 41.57999801635742, 49.369998931884766, 57.04999923706055,
    64.8125, 72.61000061035156, 80.40499877929688, 87.75499725341797,
    94.63499450683594, 102.13749694824219, 108.94499969482422, 115.11499786376953,
    120.86499786376953, 126.25499725341797, 131.4449920654297, 136.59750366210938,
    140.22499084472656, 143.13499450683594, 145.6374969482422, 147.79750061035156,
    149.61000061035156, 151.1199951171875, 152.4449920654297, 153.6425018310547,
    154.77249145507812, 155.71749877929688
]

execution_time= [
    0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 2, 2, 2, 1
]

explored_area_wlos = [3.9475, 9.805, 16.74, 22.1975, 29.895, 35.8575, 42.8725, 49.425, 55.5275,
                  62.465, 69.44, 76.455, 82.57, 87.26, 92.2425, 97.005, 101.4225, 106.425,
                  110.9175, 114.8625, 118.3725, 121.51, 123.8875, 125.8275, 127.7425,
                  129.6375, 131.27, 132.92, 134.4425, 135.8075 ]

execution_time_wlos = [
    4, 7, 11, 37, 20, 26, 31, 37, 43, 48, 53, 60, 64, 68, 76, 79, 82, 84, 91, 92,
    93, 96, 95, 97, 100, 100, 101, 96, 97, 95
]

explored_area_wlos_wpd30 = [
    3.9475000000000007, 9.805000000000001, 15.015000000000002, 20.157500000000002,
    24.007500000000004, 29.960000000000004, 36.392500000000005, 40.947500000000005,
    46.80250000000001, 53.01500000000001, 59.41000000000001, 65.48750000000001,
    72.25500000000001, 78.46750000000002, 83.43250000000002, 88.88250000000002,
    94.22500000000002, 98.21500000000002, 102.85750000000002, 108.30500000000002,
    111.77750000000002, 115.50500000000002, 117.92000000000002, 120.00750000000002,
    121.95000000000002, 123.56000000000003, 124.96000000000002, 126.35250000000002,
    127.97250000000003, 129.51500000000001
]

execution_time_wlos_wpd30 = [
    4, 5, 5, 6, 6, 7, 8, 8, 9, 10, 11, 13, 14, 14, 15, 17, 17, 17, 18, 19,
    20, 20, 21, 20, 20, 20, 20, 19, 19, 19
]


total_area = 157
explored_area_percent_wLOS = [(area / total_area) * 100 for area in explored_area]
explored_area_percent_wls = [(area / total_area) * 100 for area in explored_area_wlos]

# Adjusting execution time to be cumulative
cumulative_time_wlos = [sum(execution_time[:i+1]) for i in range(len(execution_time_wlos))]
cumulative_time_wls = [sum(execution_time_wlos[:i+1]) for i in range(len(execution_time_wlos))]


# Adjusting explored area to percentage of total area (157)
explored_area_percent_wlos_wpd30 = [(area / total_area) * 100 for area in explored_area_wlos_wpd30]

# Adjusting execution time to be cumulative with a pathfinding delay of 30 added at the start
cumulative_time_wlos_wpd30 = [30 + sum(execution_time_wlos_wpd30[:i+1]) for i in range(len(execution_time_wlos_wpd30))]

# Creating the figure and the axes for the adjusted data
plt.figure(figsize=(10, 6))

plt.plot(iterations_wlos, explored_area_percent_wLOS, 'g-', label='Explored Area % w/o LOS')
plt.plot(iterations_wlos, explored_area_percent_wls, 'b-', label='Explored Area % w/ LOS')
plt.plot(iterations_wlos, explored_area_percent_wlos_wpd30, 'r-', label='Explored Area % w/ LOS + WPD 30')

plt.xlabel('Iteration')
plt.ylabel('Explored Area (%)')
plt.title('Explored Area Percentage per Iteration')
plt.legend(loc='upper left')

plt.twinx()
plt.plot(iterations_wlos, cumulative_time_wlos, 'g--', label='Time w/o LOS')
plt.plot(iterations_wlos, cumulative_time_wls, 'b--', label=' Time w/ LOS')
plt.plot(iterations_wlos, cumulative_time_wlos_wpd30, 'r--', label=' Time w/ LOS + WPD 30')
plt.ylabel('Computational Time (s)')

plt.legend(loc='lower right')

plt.show()
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

"""
python3 compare_real_sim.py log_real.csv log_sim.csv
if IndexError pops up probably because of incomplete data at end of file
"""


def readCSV(filename):
    """
    returns
    t = [1.2, 1.3, 1.4, ...]
    pos0 = [[x0, x1, x2,... ], [y0, y1, y2,...], [z0, ...]]
    """
    t = []
    pos0 = [[], [], []]
    pos1 = [[], [], []]
    print(f"reading {filename}...")
    with open(filename) as csvfile:
        reader = csv.reader(csvfile)
        is_first_row = True
        for row in reader:
            if is_first_row:
                is_first_row = False
                continue
            t.append(float(row[0]))
            for i in range(3):
                pos0[i].append(float(row[1+i]))
            for i in range(3):
                pos1[i].append(float(row[4+i]))
    return t, pos0, pos1


real_filename = sys.argv[1]
real_t, real_pos0, real_pos1 = readCSV(real_filename)

sim_filename = sys.argv[2]
sim_t, sim_pos0, sim_pos1 = readCSV(sim_filename)

start_time = max(real_t[0], sim_t[0])
end_time = min(real_t[-1], sim_t[-1])

print(f"real measurement: {real_t[0]} ~ {real_t[-1]}")
print(f"sim measurement: {sim_t[0]} ~ {sim_t[-1]}")
print(f"overlapping time: {start_time} ~ {end_time}")

t_interp = []
real_pos0_interp = [[], [], []]
real_pos1_interp = [[], [], []]
sim_pos0_interp = [[], [], []]
sim_pos1_interp = [[], [], []]
err0_interp = []
err1_interp = []

for t in np.arange(start_time, end_time, step=0.1):
    t_interp.append(t)
    real_pos0_t = np.zeros(3)
    real_pos1_t = np.zeros(3)
    sim_pos0_t = np.zeros(3)
    sim_pos1_t = np.zeros(3)
    for i in range(3):
        real_pos0_t[i] = np.interp(t, real_t, real_pos0[i])
        # fix any initial offset here...
        if i == 2:
            real_pos0_t[i] += 0.008
        real_pos0_interp[i].append(real_pos0_t[i])

        real_pos1_t[i] = np.interp(t, real_t, real_pos1[i])
        if i == 2:
            real_pos1_t[i] += 0.008
        if i == 1:
            real_pos1_t[i] += 0.012
        real_pos1_interp[i].append(real_pos1_t[i])

        sim_pos0_t[i] = np.interp(t, sim_t, sim_pos0[i])
        if i == 0:
            # flip x & y axes
            sim_pos0_t[i] *= -1
        sim_pos0_interp[i].append(sim_pos0_t[i])

        sim_pos1_t[i] = np.interp(t, sim_t, sim_pos1[i])
        if i == 0:
            sim_pos1_t[i] *= -1
        sim_pos1_interp[i].append(sim_pos1_t[i])
    err0_interp.append(np.linalg.norm(real_pos0_t-sim_pos0_t))
    err1_interp.append(np.linalg.norm(real_pos1_t-sim_pos1_t))

for i in range(3):
    xyz = ["x", "y", "z"]
    plt.plot(t_interp, real_pos1_interp[i], label=f"real1_{xyz[i]}")

    plt.plot(t_interp, sim_pos1_interp[i], label=f"sim1_{xyz[i]}")

plt.plot(t_interp, err1_interp, label="err0")

print(f"average error for top segment: {np.average(err0_interp)}\tbottom segment: {np.average(err1_interp)}")
plt.legend()
plt.show()

with open("compare_real_sim.csv", "w") as csvfile:
    csvWriter = csv.writer(csvfile, delimiter=",")
    firstRow = ["time(sec)"]
    xyz = ["x", "y", "z"]
    for segment_i in range(2):
        for xyz_i in range(3):
            firstRow.append(f"real_seg{segment_i}_{xyz[xyz_i]}")
            firstRow.append(f"sim_seg{segment_i}_{xyz[xyz_i]}")

        firstRow.append(f"err_seg{segment_i}")
    csvWriter.writerow(firstRow)
    
    for i in range(len(t_interp)):
        data = [t_interp[i]]
        for xyz_i in range(3):
            data.append(real_pos0_interp[xyz_i][i])
            data.append(sim_pos0_interp[xyz_i][i])
        data.append(err0_interp[i])
        for xyz_i in range(3):
            data.append(real_pos1_interp[xyz_i][i])
            data.append(sim_pos1_interp[xyz_i][i])
        data.append(err1_interp[i])
        csvWriter.writerow(data)
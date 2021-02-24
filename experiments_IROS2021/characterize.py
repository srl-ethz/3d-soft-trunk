from softtrunk_pybind_module import SoftTrunkModel
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

"""
python3 characterize.py log_curvature.csv log_pressure.csv
"""
segments = 2
# first, cook the curvature data
filename = sys.argv[1]
curvature_t = []
curvature_Q = [[], [], [], []]# saves curvature of each segment
curvature_poly_deg = 25
curvature_domain = [2, 5.2] # only use this range
curvature_poly = []   # save polyfitted curvatre to here

with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    is_first_row = True
    for row in reader:
        if is_first_row:
            is_first_row = False
            continue
        timestamp = float(row[0])/10**6
        if curvature_domain[0] - 0.1 < timestamp < curvature_domain[1] + 0.1:
            # use values a bit outside the range so that all values within range are fitted
            curvature_t.append(timestamp)
            for i in range(2*segments):
                curvature_Q[i].append(float(row[i+1]))


print(f"fitting {curvature_poly_deg}-th order polynomial to curvature data.")
for i in range(2*segments):
    curvature_poly.append(np.polynomial.Polynomial.fit(curvature_t, curvature_Q[i], deg=curvature_poly_deg))
    curvature_interp_t, curvature_intep_Q = curvature_poly[i].linspace(n=100, domain=curvature_domain)
    plt.plot(curvature_t, curvature_Q[i], ".-", label=f"Q{i}")
    plt.plot(curvature_interp_t, curvature_intep_Q, ".-", label=f"Q{i}_poly")

plt.xlabel("t [sec]")
plt.legend()
plt.show()
from softtrunk_pybind_module import Controller, State
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

segments = 3
filename = sys.argv[1]

log_t = []
log_q = []
for i in range(2 * segments):
    log_q.append([]) # if you do log_q = [] * (segment*2) there is a problem that all elements refer to same array

poly_deg = 5
polyfit = []
polyfit_d = []
polyfit_dd = []

print(f"reading from recorded data: {filename}...")
with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    is_first_row = True
    for row in reader:
        if is_first_row:
            is_first_row = False
            continue
        log_t.append(float(row[0]))
        for i in range(2 * segments):
            log_q[i].append(float(row[1+i]))

print(f"fitting {poly_deg}-th order polynomial to data...")
for i in range(2*segments):
    polyfit.append(np.polynomial.Polynomial.fit(log_t, log_q[i], deg = poly_deg))
    polyfit_d.append(polyfit[i].deriv(m=1))
    polyfit_dd.append(polyfit[i].deriv(m=2))
    
    plot_t, plot_q = polyfit[i].linspace(n=100, domain=[log_t[0], log_t[-1]])
    plt.plot(plot_t, plot_q, ".-", label=f"q{i}")

plt.xlabel("t [sec]")
plt.legend()
plt.title(f"{poly_deg}-th degree polyfit")
print("check that this looks like a good fit to recorded data. If not, change poly_deg and run again")
plt.show()
plt.close()
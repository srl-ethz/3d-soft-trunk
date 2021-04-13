from softtrunk_pybind_module import SoftTrunkModel
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

"""
python3 characterize.py log_curvature.csv log_pressure.csv

Process the data acquired by characterization_actuation.
before running this, set shear_modulus and drag_coef in SoftTrunkModel.h to 1.
edit as necessary to fix xyz frame mismatch, chamber ID mismath between real and model.
"""
segments = 1
sections_per_segment = 1

# first, calculate polynomial fit for the curvature data so that velocity and acceleration can be calculated
# experiment with curvature_poly_deg, curvature_domain so that the fitted curves make sense
timespan = [5.3, 8.4] # timespan used for polyfit
samples = 24  # how many samples (equally spaced within timespan) to use. Make sure results are stable even when you change this.


filename = sys.argv[1]
# variables to store values read from csv
curvature_t = []
curvature_Q = [[], [], [], []]  # saves curvature of each segment

curvature_poly_deg = 5
curvature_poly = []   # save polyfitted curvature to here

print(f"reading from curvature data file: {filename} ...")
with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    is_first_row = True
    for row in reader:
        if is_first_row:
            is_first_row = False
            continue
        timestamp = float(row[0])/10**6
        if timespan[0] - 0.1 < timestamp < timespan[1] + 0.1:
            # use values a bit outside the range so that all values within range are fitted
            curvature_t.append(timestamp)
            for i in range(2*segments):
                curvature_Q[i].append(float(row[i+1]))


print(f"fitting {curvature_poly_deg}-th order polynomial to curvature data.")
for i in range(2*segments):
    curvature_poly.append(np.polynomial.Polynomial.fit(curvature_t, curvature_Q[i], deg=curvature_poly_deg))
    curvature_interp_t, curvature_intep_Q = curvature_poly[i].linspace(n=100, domain=timespan)
    plt.plot(curvature_t, curvature_Q[i], ".-", label=f"Q{i}")
    plt.plot(curvature_interp_t, curvature_intep_Q, ".-", label=f"Q{i}_poly")

print("calculating derivatives")
curvature_poly_d = []
curvature_poly_dd = []

for i in range(2*segments):
    curvature_poly_d.append(curvature_poly[i].deriv(m=1))
    curvature_poly_dd.append(curvature_poly[i].deriv(m=2))
    plt.plot(curvature_t, [curvature_poly_d[i](t) for t in curvature_t], ".-", label=f"dQ{i}_poly")
    plt.plot(curvature_t, [curvature_poly_dd[i](t) for t in curvature_t], ".-", label=f"ddQ{i}_poly")


plt.xlabel("t [sec]")
plt.legend()
plt.title(f"{curvature_poly_deg}-th degree polyfit")
plt.show()
plt.close()

# next, process the pressure data. Just use linear interpolation for this.
pressure_t = []
pressure_p = [[], [], [], [], [], []]
filename = sys.argv[2]
print(f"reading from pressure data file: {filename} ...")
with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    is_first_row = True
    for row in reader:
        if is_first_row:
            is_first_row = False
            continue
        timestamp = float(row[0])
        if timespan[0] -0.1 < timestamp < timespan[1] + 0.1:
            pressure_t.append(timestamp)
            for i in range(3*segments):
                pressure_p[i].append(100* float(row[i+(3*segments+1)])) # get measured pressure (convert mbar to Pa)

for i in range(3*segments):
    plt.plot(pressure_t, pressure_p[i], ".-", label=f"p{i}")
plt.legend()
plt.show()

stm = SoftTrunkModel()

# qPrime is the vector collecting 4 M_I q for all samples,
# dqPrime is the vector collecting M_I dq for all samples. (see paper for details, equation (15))
# these vectors contain all the measurements to be used in the characterization
qPrime = np.zeros(2*sections_per_segment * segments * samples)
dqPrime = np.zeros(qPrime.shape)
f_knownPrime = np.zeros(qPrime.shape)

q = np.zeros(2*segments*sections_per_segment)
dq = np.zeros(q.shape)
ddq = np.zeros(q.shape)
p = np.zeros(3*segments)

sample_i = 0
for t in np.linspace(timespan[0], timespan[1], samples):
    # first get the polynomially approximated pose and their derivatives
    for i_seg in range(segments):
        # assume the segment is constant curvature, i.e. assign same curvature to all PCC sections in a segment
        q[2*i_seg*sections_per_segment:2*(i_seg+1)*sections_per_segment] = np.tile([curvature_poly[2*i_seg](t)/sections_per_segment, curvature_poly[2*i_seg+1](t)/sections_per_segment], sections_per_segment)
        dq[2*i_seg*sections_per_segment:2*(i_seg+1)*sections_per_segment] = np.tile([curvature_poly_d[2*i_seg](t)/sections_per_segment, curvature_poly_d[2*i_seg+1](t)/sections_per_segment], sections_per_segment)
        ddq[2*i_seg*sections_per_segment:2*(i_seg+1)*sections_per_segment] = np.tile([curvature_poly_dd[2*i_seg](t)/sections_per_segment, curvature_poly_dd[2*i_seg+1](t)/sections_per_segment], sections_per_segment)
    # correct the frame mismatch between model and Qualisys data... edit this as necessary
    q *= -1
    dq *= -1
    ddq *= -1

    stm.updateState(q, dq)
    B, c, g, K, D, A, J = stm.getModel()

    # calculate pressure at this time
    for i_chamber in range(3*segments):
        p[i_chamber] = np.interp(t, pressure_t, pressure_p[i_chamber])
    # rotate the chambers as necessary, if it is different from model...

    # this combines all the forces except elastic & dissipative
    f_known = np.matmul(A, p) - np.matmul(B, ddq) - c - g

    # add to the expanded vectors
    qPrime[2*sections_per_segment * segments * sample_i: 2*sections_per_segment * segments * (sample_i+1)] = q* K.diagonal()
    dqPrime[2*sections_per_segment * segments * sample_i: 2*sections_per_segment * segments * (sample_i+1)] = dq* D.diagonal()
    f_knownPrime[2*sections_per_segment * segments * sample_i: 2*sections_per_segment * segments * (sample_i+1)] = f_known
    sample_i += 1

qPrime_combined = np.zeros((qPrime.size, 2))
qPrime_combined[:,0] = qPrime
qPrime_combined[:,1] = dqPrime
result = np.matmul(np.linalg.pinv(qPrime_combined), f_knownPrime)

print(f"result:\n\tshear modulus: {result[0]}\n\tdrag coef:{result[1]}")
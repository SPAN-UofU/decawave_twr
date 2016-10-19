import numpy as np

fname_ref = 'ref.txt'
fname_sync = 'sync.txt'

data_ref = np.loadtxt(fname_ref, delimiter=' ')
data_sync = np.loadtxt(fname_sync, delimiter=' ')

T_tx1_ts_dtu = data_sync[:, 0]
T_tx1_sc_dtu = data_sync[:, 1]
T_rx1_ts_dtu = data_sync[:, 2]
T_rx1_sc_dtu = data_sync[:, 3]

T_rx2_ts_dtu = data_ref[:, 0]
T_rx2_sc_dtu = data_ref[:, 1]
T_tx2_ts_dtu = data_ref[:, 2]
T_tx2_sc_dtu = data_ref[:, 3]

delta_ts_dtu = T_tx2_ts_dtu - T_rx2_ts_dtu
delta_sc_dtu = T_tx2_sc_dtu - T_rx2_sc_dtu

Delta_ts_dtu = (T_rx1_ts_dtu-T_tx1_ts_dtu-delta_ts_dtu)/2.
Delta_sc_dtu = (T_rx1_sc_dtu-T_tx1_sc_dtu-delta_sc_dtu)/2.

phi_ts_dtu = T_rx2_ts_dtu-T_tx1_ts_dtu-Delta_ts_dtu
phi_sc_dtu = T_rx2_sc_dtu-T_tx1_sc_dtu-Delta_sc_dtu

print np.array(['Delta (dtu)', 'Delta (ns)'])
print np.vstack((Delta_ts_dtu, 1e9*Delta_ts_dtu/(499.2e6*128.))).T

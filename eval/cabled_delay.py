import sys
import numpy as np
import scipy.constants as sp

class cable():

	# Initialize
	def __init__(self, lcm, delay, psol, color):
		self.length_cm = lcm
		self.delay_ns_per_ft = delay
		self.psol = psol
		self.color = color
		
		self.dtu_2_sec = 1.0/(499.2e6*128.)
		self.ns_per_ft_2_s_per_m = 3.2808398950131/1e9
		self.sol_mps = 299792458.0
		
		self.Delta_ts_dtu = None

		self.data_ref = None
		self.data_sync = None

		self.__initialize()
		
	# Get Delta_dtu
	def get_Delta_dtu(self):
		return self.Delta_ts_dtu
	
	# Get Delta (ns)
	def get_Delta_ns(self):
		return self.Delta_ts_dtu*self.dtu_2_sec
	
	# Get cable delay (ns)
	def get_cable_delay_ns(self):
		ret_val1 = (self.length_cm/100.)*(self.delay_ns_per_ft*self.ns_per_ft_2_s_per_m)
		ret_val2 = self.length_cm*100./(self.psol*100.*self.sol_mps)
		return (ret_val1+ret_val2)/2.

	# Compute Delta
	def __compute_Delta_dtu(self):
		T_tx1_ts_dtu = self.data_sync[:, 0]
		# T_tx1_sc_dtu = self.data_sync[:, 1]
		T_rx1_ts_dtu = self.data_sync[:, 2]
		# T_rx1_sc_dtu = self.data_sync[:, 3]

		T_rx2_ts_dtu = self.data_ref[:, 0]
		# T_rx2_sc_dtu = self.data_ref[:, 1]
		T_tx2_ts_dtu = self.data_ref[:, 2]
		# T_tx2_sc_dtu = self.data_ref[:, 3]

		delta_ts_dtu = T_tx2_ts_dtu - T_rx2_ts_dtu
		# delta_sc_dtu = T_tx2_sc_dtu - T_rx2_sc_dtu

		self.Delta_ts_dtu = (T_rx1_ts_dtu-T_tx1_ts_dtu-delta_ts_dtu)/2.
		# Delta_sc_dtu = (T_rx1_sc_dtu-T_tx1_sc_dtu-delta_sc_dtu)/2.


	# Initialze stuff
	def __initialize(self):
		self.data_ref = np.loadtxt('ref_%s.txt' % self.color, delimiter=' ')
		self.data_sync = np.loadtxt('sync_%s.txt' % self.color, delimiter=' ')

		if self.data_ref.shape[0] != self.data_sync.shape[0]:
			sys.stderr.write('file lengths are not the same. Quitting.')
			quit()
		
		self.__compute_Delta_dtu()



# Create cable objects
gold_cable = cable(61.0, 1.463, 69.5, 'gold')
blk_cable = cable(95.0, 1.210, 84.0, 'black')

print np.mean(gold_cable.get_Delta_ns()) - gold_cable.get_cable_delay_ns()
print np.mean(blk_cable.get_Delta_ns()) - blk_cable.get_cable_delay_ns()

# print np.array(['Delta (dtu)', 'Delta (ns)', 'Distance (m)'])
# print np.vstack((Delta_ts_dtu, 1e9*Delta_ts_dtu/(499.2e6*128.), (vop*sp.speed_of_light*Delta_ts_dtu/(499.2e6*128.)))).T

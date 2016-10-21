import sys
import numpy as np
import scipy.constants as sp

class cable():

	# Initialize
	def __init__(self, cl_cm, al_cm, delay, psol, color, db):
		self.cable_len_cm = cl_cm
		self.att_len_cm = al_cm
		self.delay_ns_per_ft = delay
		self.psol = psol
		self.color = color
		self.db = db
		
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
	
	# Get Delta (sec)
	def get_Delta(self):
		return self.Delta_ts_dtu*self.dtu_2_sec
	
	# Get cable delay
	def get_cable_delay(self):
		ret_val1 = (self.cable_len_cm/100.)*(self.delay_ns_per_ft*self.ns_per_ft_2_s_per_m)
		ret_val2 = self.cable_len_cm*100./(self.psol*100.*self.sol_mps)
		return ret_val1
		# return (ret_val1+ret_val2)/2.
	
	# Get average measured delay
	def get_avg_measured_delay(self):
		return np.mean(self.get_Delta())
	
	# Get A and Delta_{color,db}
	def get_terms(self):
		d_cdb = self.get_avg_measured_delay()-self.get_cable_delay()
		A = (self.att_len_cm*100.)/(100.*self.sol_mps)
		return A,d_cdb
	
	# compute x
	def get_x_y(self,A_60,A_40,D_60,D_40):
		x = (A_60-A_40)/float(D_60-D_40)
		y = D_40 - (A_40/x)
		return x,y

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
		self.data_ref = np.loadtxt('ref_%sdb_%s.txt' % (self.db, self.color), delimiter=' ')
		self.data_sync = np.loadtxt('sync_%sdb_%s.txt' % (self.db, self.color), delimiter=' ')

		if self.data_ref.shape[0] != self.data_sync.shape[0]:
			sys.stderr.write('file lengths are not the same. Quitting.')
			quit()
		
		self.__compute_Delta_dtu()



# Create cable objects
gold_40_cable = cable(61.0, 4.5, 1.463, 69.5, 'gold', '40')
gold_60_cable = cable(61.0, 6.5, 1.463, 69.5, 'gold', '60')
blk_40_cable = cable(95.0, 4.5, 1.210, 84.0, 'black', '40')
blk_60_cable = cable(95.0, 6.5, 1.210, 84.0, 'black', '60')

A_g_40, D_g_40 = gold_40_cable.get_terms()
A_g_60, D_g_60 = gold_60_cable.get_terms()
A_b_40, D_b_40 = blk_40_cable.get_terms()
A_b_60, D_b_60 = blk_60_cable.get_terms()

print gold_40_cable.get_x_y(A_g_60,A_g_40,D_g_60,D_g_40)
print blk_40_cable.get_x_y(A_b_60,A_b_40,D_b_60,D_b_40)

# print np.array(['Delta (dtu)', 'Delta (ns)', 'Distance (m)'])
# print np.vstack((Delta_ts_dtu, 1e9*Delta_ts_dtu/(499.2e6*128.), (vop*sp.speed_of_light*Delta_ts_dtu/(499.2e6*128.)))).T

'''
Created on Oct 26, 2016

@author: pete
'''

# This script reads time of flight measurements printed to stdout from the SYNC 
# node and builds a histogram.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import subprocess
import sys

matplotlib.rcParams.update({'font.size':18})

# This class implements a circular buffer
class myCircBuff:
    
    # Initializer
    def __init__(self,buff_len_,data_type=np.float):
        self.B = buff_len_ # number of vectors you want in the circular buffer
        
        self.C = np.nan*np.ones(buff_len_,dtype=data_type) # The circular buffer 
        self.num_obs = 0 # number of vectors added to the circular buffer
        self.open_idx = 0 # the index of the next open slot in buffer
        
    # Adds a new vector to the circular buffer
    def add_observation(self,obs_):
        '''Adds a new vector to the circular buffer'''
        # Overwrite the oldest observation with the current observation
        self.C[self.open_idx] = obs_
        
        # Update the index of the open index
        self.open_idx = (self.open_idx+1) % self.B
        
        # Increment the number of values in the buffer as needed        
        self.num_obs = np.minimum(self.B,self.num_obs+1)
    
    # returns the number of vectors in the buffer
    def get_num_in_buff(self):
        ''' returns the number of vectors in the buffer '''
        return self.num_obs
    
    # returns a 1 if the buffer is full, 0 otherwise
    def is_full(self):
        ''' returns a 1 if the buffer is full, 0 otherwise '''
        return self.num_obs == self.B
    
    # return the entire buffer as is
    def get_buffer(self):
        return 1*self.C

# Get tof from stdin
def get_tof():
    # read in line from stdout
    line = sys.stdin.readline()
    
    # Get a list of the entries
    stripped_line = line.split(' ')
    
    # If 'offset' is an entry, print the time of flight 
    if 'offset:' not in stripped_line:
        return 0
    
    # Get index of the tof value and save it
    tof_idx = stripped_line.index("offset:")+1
    tof = float(stripped_line[tof_idx])
        
    # print tof

    return tof

####################################
# Start script here
####################################
if __name__ == '__main__':

    # set a max on the tof that is allowed to get into the buffer
    max_tof = 1e-6

    # Get the first 'buff_len' samples to get a guess on how to set xmin and xmax
    # for the figure
    buff_len = 10
    circ_buff = myCircBuff(buff_len)
    tof_range = [0,0]
    while True:
        tof = get_tof()

        # filter out large tofs
        if (tof == 0) | (tof > max_tof):
            continue
        
        # Add the tof to a circular buffer
        circ_buff.add_observation(tof)
        
        # once the buffer is full, get the range and break
        if circ_buff.is_full():
            buff = circ_buff.get_buffer()
            tof_range = [buff.min(),buff.max()]
            break
    
    # Create new buffer where we store the tofs
    buff_len = 100
    circ_buff = myCircBuff(buff_len)
    num_bins = 10 # number of bins on the histogram
    plot_every_n = 2 # must be 2 or more

    # Set the x-min and x-max for the figure. Also get the ymax
    xlim_min = tof_range[0]-0.5*(tof_range[1]-tof_range[0])
    xlim_max = tof_range[1]+0.5*(tof_range[1]-tof_range[0])
    ymax = float(buff_len)/num_bins*5.0*2.0

    fig, ax = plt.subplots()
    ax.hist(1e-9*np.random.randn(buff_len),bins=num_bins)
    line_mean, = ax.plot(np.nan,np.nan,color='k',lw=2,linestyle='-')
    line_std, = ax.plot(np.nan,np.nan,color='k',lw=2,linestyle='--')
    text_mean = ax.text(xlim_min, 0.9*ymax, 'mean: %e' % 1e-9)
    text_std  = ax.text(xlim_min, 0.7*ymax, 'std: %e' % 0.1e-9)
    ax.set_xlim(xlim_min,xlim_max)
    ax.set_ylim(0,ymax)
    ax.set_ylabel('Count')
    ax.set_xlabel('Time of Flight (sec)')
    fig.canvas.draw()
    plt.show(block=False)

    counter = 0
    while True:
        tof = get_tof()

        # filter out large tofs
        if (tof == 0) | (tof > max_tof):
            continue
        
        # Add the tof to a circular buffer
        circ_buff.add_observation(tof)

        # update figure every 'plot_every_n' tofs
        counter += 1
        if counter < plot_every_n:
            continue
        else:
            counter = 0

        # get the samples from the buffer and remove nans
        x = circ_buff.get_buffer()
        x_nonan = x[np.isnan(x) == 0]
        # print x_nonan
        
        # Get mean and standard deviation of the tofs
        m = np.mean(x_nonan)
        s  = np.std(x_nonan)

        # Set the lines for the mean and standard deviation
        line_mean.set_xdata([m,m])
        line_mean.set_ydata([-1,2*ymax])
        line_std.set_xdata([m-s,m-s,m+s,m+s])
        line_std.set_ydata([-1,2*ymax,2*ymax,-1])

        # Set the text for mean and std
        text_mean.set_text('mean:\n %3.4e' % m)
        text_std.set_text('std:\n %3.4e' % s)
        
        # Get the current histogram
        count, bins, patch_list = ax.hist(x_nonan,bins=num_bins)
        
        # Update the figure
        ax.draw_artist(ax.patch)
        ax.draw_artist(line_mean)
        ax.draw_artist(line_std)
        ax.draw_artist(text_mean)
        ax.draw_artist(text_std)
        for patch_item in patch_list:
            patch_item.set_facecolor('red')
            patch_item.set_color('red')
            patch_item.set_ec('k')
            ax.draw_artist(patch_item)
        fig.canvas.update()
        fig.canvas.flush_events()
    
    
    
    
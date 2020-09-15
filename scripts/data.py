#!/usr/bin/env python3

# Lib
import sys
import time
import numpy as np
import csv
import os



class Data:
    """
    Toolset to manage data matrices.
    """
    def __init__(self, name, directory, filename, column_names, min_write_interval=4.0):
        self.csv_init(directory, filename, column_names)
        # TODO freq max de guardado
        self.name = name
        self.column_names = column_names
        self.file = directory + filename
        self.init = False
        self.size = 0
        self.min_write_interval = min_write_interval
        self.last_write = 0 

    def csv_init(self, directory, filename, column_names):
        utils.check_dir(directory)
        np.savetxt(directory+filename, column_names, delimiter=',', fmt='%s')

    def read(self):
        self.data = np.loadtxt(self.file, delimiter=',', skiprows=1)
        if len(self.data)>1:
            self.update_size()
            self.init = True
        
    def write(self):
        dt = time.time() - self.last_write
        if dt >= self.min_write_interval:
            np.savetxt(self.file, self.data, delimiter=',', fmt='%s')
            self.last_write = time.time()

    def append(self, sample):
        try:
            self.append2csv(sample)
            self.append2mat(sample)
        except:
            print('['+self.name+']: Couldnt add sample')

    def append2csv(self,sample):
        with open(self.file, 'ab') as csvfile:
            np.savetxt(csvfile, sample, delimiter=',', fmt='%s')
            
    def append2mat(self,sample):
        if self.init:
            self.data = np.append(self.data, sample, axis=0)
        else:
            self.data = np.array(sample)
            self.init = True
        self.update_size()

    def get(self): 
        if self.init:
            return self.data
        else:
            print('['+self.name+']: Can not retrieve data. Data class not initialized')
            return []

    def update_size(self):
        self.size = self.data.shape[0]

    def get_size(self):
        return self.size

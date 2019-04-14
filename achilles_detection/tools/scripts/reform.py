#!/usr/bin/env python      
# -*- coding: utf-8 -*-
import __future__
import os 
import linecache
import numpy as np
import pandas as pd
import operator
from functools import reduce

import matplotlib.pyplot as plt


def main():
    #sample = pd.read_csv('../data/sample_Z.csv', encoding='gbk')
    #file = "../data/input_Z.txt"

    sample = pd.read_csv('../data/sample_X.csv', encoding='gbk')
    file = "../data/input_X.txt"

    #sample = pd.read_csv('../data/sample_Y.csv', encoding='gbk')
    #file = "../data/input_Y.txt"
    fd = open(file, 'w')

    
    distance = pd.DataFrame(sample, columns = ['A']).values.T
    distance = reduce(operator.add,distance)
    data = pd.DataFrame(sample, columns = ['B','C','D','E','F']).values.T

    print(distance)
    print(data)

    for i in range(0,len(distance)):
        fd.write( str(distance[i]) )
        fd.write( ',' )

    for j in range(0,5):
        fd.write('\n')
        for i in range(0,len(distance)):
            fd.write( str(data[j,i]) )
            fd.write( ',' )
    

    fd.close()


if __name__ == '__main__':
    main()
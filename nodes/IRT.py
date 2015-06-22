#!/usr/bin/env python
#coding: utf-8
"""
Created on Wed Apr  1 23:36:12 2015

@author: ferran
"""

"""
 This model assumes a dichotomy which means an answer can be either correct or
 incorrect. The discrimination (a) and difficulty (b) parameters are extracted 
 assuming a model of non-linear regression. The guessing parameter (c) is the
 probability of answer correctly choosing blindly among 4 options.
 The upper bound (u) allows to set up a maximum probability of being correct 
 given the rt, L1 and L2 described below.
"""

#import numpy as np
#
## Parameters calculated by non-linear regression for each type of question:
#a = -1.34   # Discrimination
#b = 0.037   # Difficulty
#
#c = 0.25    # Guessing parameter 
#u = 0.76    # Upper bound
#rt = 10.0   # Response Time
#L1 = 45.8   # Number of characters in question
#L2 = 23.4   # Number of characters in all responses

"""
 The Identity Time Response (ITR) contains an exponential curve whose
 behaviour comprends 4 regions R1 to R4:
  R1: Low time response. High probability of low engagement
  R2: Rational answering time
  R3: Peak of the exponential. High probbility of right answer
  R4: Settling region. Not taken into account
"""

## Form of the model: P(correct|rt,L1, L2) = c + (d-c)/1+exp^(-a(-rt+b(L1+L2)))
#P_correct = c + ((u - c) / (1 + np.exp(-a * (-rt + b * (L1 + L2)))))
#
## Probability of being disengaged
#P_disengaged = (u - P_correct) / (u - c)
#
## Probability of being engaged
#P_engaged = 1 - P_disengaged
#
#print P_correct
#print P_disengaged
#print P_engaged


#Model problems:
#    - Assumes a multiple choice of 4 (c) -> Answer correct or not
#    - Assumes that the answer is either right or wrong -> Given a threshold
#    - Assumes the reading time (L1) -> Not necessary
#    - Assumes a maximum performance (u) -> Minimum distance in to cluster
#    - Classifies the questions on groups -> Do it depending on size
#    - Calculates (a) and (b) based on non-linear regression -> Do the same

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# TODO: Take into account the time spend before answering the question

import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

 
class engagement_model():
        
    def __init__(self):
        
        csv_file = '/home/ferran/catkin_ws/src/emotional-manager/nodes/Hard.csv'        
        
        # Define initial parameters.
        self.u = 0.8     # Upper bound.
        self.L = 4.5     # Mean number of characters in word.
        
        
        # Initial parameters guess.
        self.a = -0.1
        self.b = -0.1
        
        # Read the csv.
        self.data = pd.read_csv(csv_file)  
        self.resp_time = self.data['rt']
        self.P_correct = self.data['P']
        
        # Convert it to numpy array.
        self.resp_time = np.array(self.resp_time) 
        self.P_correct = np.array(self.P_correct)
        
    def func(self, resp_time, a, b):
        return self.u / (1 + np.exp(-a * (-resp_time + b * self.L)))
  
    def train_model(self):
        
        popt, pcov = curve_fit(self.func, self.resp_time, self.P_correct, p0 = (self.a, self.b))
        
        self.a = popt[0]
        self.b = popt[1]
        residuals = self.P_correct - self.func(self.resp_time, self.a, self.b)
        fres = sum(residuals**2)
     
    def plot_data(self):        
        # Plot points.
        fig = plt.figure()
        ax = fig.add_subplot(111)

        #ax.plot(self.resp_time, self.P_correct, '*')
        plt.xlabel('Response Time')
        plt.ylabel('P(correct)');
        
        # Plot curve fitting.
        curvex = np.linspace(0, 10, 100)
        curvey = self.func(curvex, self.a, self.b)
        ax.plot(self.resp_time, self.P_correct, 'og')
        line = ax.plot(curvex, curvey)
        setp(line, linewidth=2, color='r')
        plt.xlabel('Response Time (s)')
        plt.ylabel('P(correct)')
        plt.xticks(np.arange(0, 10, 0.5))

        #Grid
        grid(b=True, which='major', linewidth=0.1, color='k', linestyle='-')

        #Plot vertical lines and text
        axvline(2, linewidth=1, color='b')        
        axvline(4.5 , linewidth=1, color='b')
        axvline(7, linewidth=1, color='b')
        
        ax.text(0.75, 0.65, r'R1', fontsize=15)
        ax.text(3.0, 0.65, r'R2', fontsize=15)
        ax.text(5.75, 0.65, r'R3', fontsize=15)
        ax.text(8.75, 0.65, r'R4', fontsize=15)
        
        plt.show()
        
    def get_engagement(self, rt):
        # Form of the model: P(correct|rt,L) = u/1+exp^(-a(-rt+b*L))
        P_corr = self.func(rt, self.a, self.b)        
        # Probability of being disengaged
        P_disengaged = (self.u - P_corr) / self.u        
        # Probability of being engaged
        P_engaged = 1 - P_disengaged        
        #plt.plot(rt, P_corr, 'o')

        return P_engaged, P_disengaged, P_corr
        
 
# Main function.
if __name__ == '__main__':
    myModel = engagement_model()
    myModel.train_model()
    myModel.plot_data()
    P_results = myModel.get_engagement(4)
    
    print P_results
    
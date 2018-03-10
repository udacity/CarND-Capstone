# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:16:48 2018

@author: Danilo Canivel
This need to be runned at the cli just when you want to generate a new trained model
"""
from tl_classifier import TLClassifier

clf = TLClassifier(for_real=False)


test_red = ['../../../../data/trafficlights/google/red1.png',
            '../../../../data/trafficlights/google/red2.png']

test_green = ['../../../../data/trafficlights/google/green1.jpg', 
              '../../../../data/trafficlights/google/green2.png', 
              '../../../../data/trafficlights/google/green3.png']

test_yellow_single = '../../../../data/trafficlights/google/yellow1.png'

test_red_yellow = ['../../../../data/trafficlights/google/red1.png',
                   '../../../../data/trafficlights/google/yellow2.png',
                   '../../../../data/trafficlights/google/red2.png',
                   ]


print('Testing RED')

state = clf.get_classification_batch_argmax(image_list=test_red)

if(state == 0):
    print('RED', state)
elif(state == 1):
    print('YELLOW', state)
else:
    print('GREEN', state)
    
print('Testing GREEN')

state = clf.get_classification_batch_argmax(image_list=test_green)

if(state == 0):
    print('RED', state)
elif(state == 1):
    print('YELLOW', state)
else:
    print('GREEN', state)
    
print('Testing YELLOW SINGLE')

state = clf.get_classification(image=test_yellow_single)

if(state == 0):
    print('RED', state)
elif(state == 1):
    print('YELLOW', state)
else:
    print('GREEN', state)
    
print('Testing RED MIX YELLOW, should return RED')

state = clf.get_classification_batch_argmax(image_list=test_red_yellow)

if(state == 0):
    print('RED', state)
elif(state == 1):
    print('YELLOW', state)
else:
    print('GREEN', state)
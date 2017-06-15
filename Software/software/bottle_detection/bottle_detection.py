"""
Run this script to start a bottle detection. 
"""

import haar_detector as hd

def run():
    bottle_detector = hd.HaarDetector('classifiers/training2_0.xml', 'bottle')
    print('bottle_detector created: ')
    bottle_detector.visualize(True)
    print('Visualisation: ', bottle_detector.bool_visualisation)
    bottle_list = bottle_detector.automated_detect()
    final_bottle = bottle_detector.decision(bottle_list)
    return 'success'

"""
Evtl anstatt zeitfenster, anzahl bilder nehmen! für while loop

Include frequency of detection, since if something is detected constantly we can be quite sure about it to actually be 
the object we're looking for. If it is only sporadic it might only be a false positive detection and should therefore be
discarded.
"""


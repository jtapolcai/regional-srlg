#!/usr/bin/python

import sys
import glob, os
import fnmatch

files = set()

for root, dirnames, filenames in os.walk('.'):
	for filename in fnmatch.filter(filenames, '*.html'):
		files.add(filename)
		
sorted_files = sorted(files)
        
print "The SRLGs are circular disk failures with given number of nodes"

for file in sorted_files :
	print
	print "*"+file[:-20] 
	for root, dirnames, filenames in os.walk('.'):
		for filename in fnmatch.filter(filenames, '*.html'):
			if (filename==file):
				print "[node"+root[2:]+"](http://htmlpreview.github.io/?https://github.com/jtapolcai/regional-srlg/blob/master/nodes-in"+root[1:]+"/"+filename+")"
				#print os.path.join(root, filename)


#*16-node Pan-European optical
#[50km](http://htmlpreview.github.io/?https://github.com/jtapolcai/regional-srlg/blob/master/radius/50/16_optic_pan_eu_scaled_reduced.html)
#[100km](http://htmlpreview.github.io/?https://github.com/jtapolcai/regional-srlg/blob/master/radius/100/16_optic_pan_eu_scaled_reduced.html)
#[200km](http://htmlpreview.github.io/?https://github.com/jtapolcai/regional-srlg/blob/master/radius/200/16_optic_pan_eu_scaled_reduced.html)
#[500km](http://htmlpreview.github.io/?https://github.com/jtapolcai/regional-srlg/blob/master/radius/500/16_optic_pan_eu_scaled_reduced.html)

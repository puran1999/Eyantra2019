# -*- coding: utf-8 -*-
"""
**************************************************************************
*                  IMAGE PROCESSING (e-Yantra 2016)
*                  ================================
*  This software is intended to teach image processing concepts
*
*  MODULE: checkversion
*  Filename: checkversion.py
*  Version: 1.0.0  
*  Date: October 25, 2016
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""

import hashlib
import time

print("==============================================")
result = []
try:
	import platform
	sysver = platform.uname()
	isysver = 1;
	result.append(sysver)
	# print "System version is:", sysver

except ImportError:
    # print "No Information about OS"
    sysver = ""
    result.append(sysver)
    isysver = 0;
    pass
    #sys.exit(0)

try: 
	import platform

	pyver = platform.python_version()
	result.append(pyver[0:3])
	print("Required Python Version is 3.7.x")
	if (pyver[0:3] == '3.7'):
		print("Installed Python Version is: ", pyver)
		print("Python Installation is OK!!")
		print("==============================================")
	else:
		print("Installed Python Version is: ", pyver)
		print("Python Installation is NOT OK .. Please re-install the correct version!!")
		print("==============================================")
except ImportError:
	print("Import Error - re-check installation procedure ")
	pyver = ""
	result.append(pyver)
	pass

try:
	import cv2
	opencvver = cv2.__version__
	isopencv = 1;
	result.append(opencvver[0:3])
	print("Required Opencv version is: 4.1.1")
	if (opencvver[0:3] == '4.1'):
		print("Installed Opencv Version is: ", opencvver)
		print("Opencv Installation is OK!!")
		print("==============================================")
	else:
		print("Installed Opencv Version is: ", opencvver)
		print("Opencv Installation is NOT OK .. Please re-install the correct version!!")
		print("==============================================")
except ImportError:
    print("Import Error - re-check installation procedure.")
    isopencv = 0;
    result.append(opencvver)
    pass
    #sys.exit(0)

try:
	import numpy
	numpyver = numpy.__version__
	isnumpy = 1
	result.append(numpyver)
	print("Required Numpy version is: 1.17.3")
	if (numpyver == '1.17.3'):
		print("Installed Numpy Version is: ", numpyver)
		print("Numpy Installation is OK!!")
		print("==============================================")
	else:
		print("Installed Numpy Version is: ", numpyver)
		print("Numpy Installation is NOT OK .. Please re-install the correct version!!")
		print("==============================================")
except ImportError:
	print("Import Error - re-check installation procedure.")
	isnumpy = 0
	numpyver = ""
	result.append(numpyver)
	pass
 
if ((isopencv == 0) or (isnumpy == 0)):
	sys.exit(0)
else:
	pass

# print result
py_version= result[1].split('.')
cv_version = result[2].split('.')
np_version = result[3].split('.')

# print py_version, cv_version, np_version

py_version = ' '.join(py_version[0:2])
cv_version = ' '.join(cv_version)
np_version = ' '.join(np_version)

all_version = py_version+' '+cv_version+' '+np_version

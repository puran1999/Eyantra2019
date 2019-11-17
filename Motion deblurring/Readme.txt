The original code file is "deconvolution.py".

This code generates a window with 3 sliders (Angle, d, SNR) and real time output image to adjust motion deblurring parameters.
Also, motion deblurring is set to "linear" by default,
which can be changed to "circular" by pressing space bar key.
However, Linear is OK. No need to change that.

Input image can be changed from line 72.

For "eyantra.jpg" file, the appropriate values of sliders are:-
Angle = 90 degree
    d = 20
  SNR = 25 dB
(As found by the hit and trial methodology)
=========================================================================

My progress of shortening this code and hardcoding the 3 values can be found in
the code "deconvolution - modified.py"

RaspiVidYUV
===========
Modified RaspiVid.C to directly output/process YUV/RGB data.

There are two types of buffer data:

1. YUV: w'*h' intensities(Y) + (w'/2)*(h'/2) U colour values + (w'/2)*(h'/2) V colour values
2. RGB: w'*h' rgb values. r1,g1,b1,r2,g2,b2, ...
 
Buffer order: starting from top left pixel, going row for row to the bottom.

Size: w' is the image width extended to be a multiple of 32
      h' is the image height extended to be a multiple of 16


Find pixel with maximum intensity "-fm":
----------------------------------------
Search the Y from the YUV buffer for the max value.

Print intensity of single pixel "-pi 123,456":
----------------------------------------------
Output: "time in sec" intensity

Export data to plot with gnuplot:
---------------------------------

Export frames to file and later split the file to individual frames:

    ./raspividyuv -w 160 -h 120 -g3d > frames.dat
    awk '{if(NR>1)print "#"$0> "frame" NR-1 ".dat"}' RS='#'  frames.dat
    gnuplot

In gnuplot each frame can be plotted with:

    gnuplot> set dgrid3d 30,30
    gnuplot> set hidden3d
    gnuplot> splot "frame1.dat" u 1:2:3 with lines

Find pixel with specific RGB value "-rgbv 255,255,255":
-------------------------------------------------------
Very slow comparison of RGB values using "-rgbv 255,255,255" will output:

"time in us": (p1x,p2y) (p2x,p2y) ...

Count number of pixel with an intensity value below threshold "-fb 123":
------------------------------------------------------------------------
"time in s" "#of pixel below threshold"
    
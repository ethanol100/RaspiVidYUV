RaspiVidYUV
===========

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

    
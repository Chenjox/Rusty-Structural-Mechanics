set datafile separator ','
set multiplot
#set terminal postscript
#set output '| ps2pdf - output.pdf'
set grid xtics ytics
set lmargin 5

set xrange [0:6]

set origin 0.0, 0.0

plot for [col=2:4] 'out1.csv' using 1:col with lines lc rgb "red"

#plot for [col=5:7] 'out1.csv' using 1:(column(col)*100) with lines lc rgb "blue"

unset ytics

#plot 'out1.csv' using 1:(0) with lines

pause -1

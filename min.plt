set datafile separator ','
#set terminal postscript
#set output '| ps2pdf - output.pdf'

set grid xtics ytics

set xlabel "D in [m]"
set ylabel "P in [m]"

plot 'determinant.txt'  using 1:2 with lines

pause -1

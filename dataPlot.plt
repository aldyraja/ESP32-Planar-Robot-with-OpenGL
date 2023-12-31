set datafile separator ','
set grid
set xlabel "time [s]"

set title "Respon Koordinat"
set ylabel "Posisi (Cartesian)"

set terminal pngcairo size 1024,768
set output "graph_posisix.png"
set autoscale
set xrange [1:20]
plot "dataLog.csv" usi 1:6 ti "X_d" w l lw 3, "" usi 1:4 ti "Xres" w l lw 3, "" usi 1:8 ti "Xcmd" w l lw 3

set term wxt 0
replot

set terminal pngcairo size 1024,768
set output "graph_posisiy.png"
set autoscale
set xrange [1:20]
plot "dataLog.csv" usi 1:7 ti "Y_d" w l lw 3, "" usi 1:5 ti "Yres" w l lw 3, "" usi 1:9 ti "Ycmd" w l lw 3

set term wxt 1
replot

set terminal pngcairo size 1024,768
set output "graph_posisi.png"
set autoscale
set xrange [1:20]
plot "dataLog.csv" usi 1:6 ti "X_d" w l lw 3, "" usi 1:4 ti "Xres" w l lw 3, "" usi 1:8 ti "Xcmd" w l lw 3, "" usi 1:7 ti "Y_d" w l lw 3, "" usi 1:5 ti "Yres" w l lw 3, "" usi 1:9 ti "Ycmd" w l lw 3

set term wxt 2
replot

set title "Respon Sudut"
set ylabel "Sudut (Derajat)"

set terminal pngcairo size 1024,768
set output "graph_derajat.png"
set autoscale
set xrange [1:20]
plot "dataLog.csv" usi 1:2 ti "q1" w l lw 3, "" usi 1:3 ti "q2" w l lw 3

set term wxt 3
replot



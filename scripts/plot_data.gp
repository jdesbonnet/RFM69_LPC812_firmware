set terminal pngcairo size 800,480
set output "plot.png"
set title "Pickle Jar Sensor"
set xlabel "Time"
set ylabel "Temperature °C"
set xdata time
set timefmt '%Y%m%d-%H%M%S'
set grid
plot 't.t' using 1:3 with linespoints title 'DS18B20'


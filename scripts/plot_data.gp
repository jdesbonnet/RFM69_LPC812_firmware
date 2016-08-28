set terminal pngcairo size 800,480
set output "plot.png"
set title "Pickle Jar Sensor"
set xlabel "Time"
set ylabel "Temperature °C"
set xdata time
set timefmt '%Y%m%d-%H%M%S'
set grid

# derivitive function
d(y) = ($0 == 0) ? (y1 = y, 1/0) : (y2 = y1, y1 = y, y1-y2)

plot 't.t' using 1:3 with lines title 'DS18B20', '' using 1:(d($2)*10) with impulses lt 3 title 'Rain tip bucket'


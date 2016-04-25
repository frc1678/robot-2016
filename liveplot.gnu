set datafile separator ","
set yrange [-1:1]
pause 1
plot "<(tail -n 5000 /tmp/liveplot.csv)" using 0:1 with lines
reread

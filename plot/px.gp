reset
set xlabel "time"
set ylabel "P_x (position)"

plot "data/px_nominal.dat" with line title "P_x nominal" lw 3, \
     "data/px_true.dat"    with line title "P_x true"    lw 3
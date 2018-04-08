set term pdf enh color dashed linewidth 2 dl 1 size 1.5, 1
set output 'ramp2d_test.pdf'
set key off
set arrow from 0,0 to 1,0
set arrow from 0,1 to 1,1
set xrange [-0.5:3.2]
set yrange [-0.1:2]
set parametric
plot 'ramp2d_testc.txt' with lines, 'ramp2d_testix.txt' with points lt 0 pt 4, 'ramp2d_testiy.txt' with points lt 2 pt 12, 'ramp2d_testp.txt' with points lt -1 pt 7

c = 3 * 10^8;
d_res = 1;
r_max = 300;

% TODO : Find the Bsweep of chirp for 1 m resolution
b_sweep = c / (2 * d_res);

% TODO : Calculate the chirp time based on the Radar's Max Range
t_chrip = 5.5 * (2 * r_max / c);

% TODO : define the frequency shifts
beat_freq = [0 1.1e6 13e6 24e6];

% Display the calculated range
calculated_range = (c * t_chrip * beat_freq) / (2 * b_sweep);
disp(calculated_range);

%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Ps = 3e-3;

%Antenna Gain (linear)
G = 10000;

%Minimum Detectable Power
Pe = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3 * 10^8;

%TODO: Calculate the wavelength
lambda = c / fc;

%TODO : Measure the Maximum Range a Radar can see.
range = ((Ps * (G^2) * (lambda^2) * RCS) / (Pe * (4 * pi)^3))^0.25;
disp(range)

noise_level = zeros(2, 3);
noise_level(1, 1) = 1;
noise_level(1, 2) = 2;
noise_level(1, 3) = 3;
noise_level(2, 1) = 4;
noise_level(2, 2) = 5;
noise_level(2, 3) = 6;
disp(noise_level)
disp(sum(noise_level))
disp(sum(sum(noise_level)))

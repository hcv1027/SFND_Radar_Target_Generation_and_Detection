[range-fft]: ./images/range-fft.jpg "range-fft"
[2D_CFAR]: ./images/2D_CFAR.png "2D_CFAR"
[2D_CFAR_result]: ./images/2D_CFAR_result.png "2D_CFAR_result"

---

# SFND Radar Target Generation and Detection
## 1. FMCW Waveform Design
```m
%% Radar Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 70 m/s
d_res = 1;
d_max = 200;
v_res = 3;
v_max = 70;
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
c = 3e8;

% *%TODO* :
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (t_chirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
bandwidth = c / (2 * d_res);
t_chirp = 2 * 5.5 * d_max / c;
slope = bandwidth / t_chirp;
```

## 2. Simulation Loop
```m
for i = 1:length(t)
    % For each time stamp update the Range of the Target for constant velocity.
    r_t(i) = d0 + v0 * t(i);
    td(i) = 2 * r_t(i) / c;

    % For each time sample we need update the transmitted and received signal.
    Tx(i) = cos(2 * pi * (fc * t(i) + slope * t(i)^2/2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + slope * (t(i) - td(i))^2/2));

    % Now by mixing the Transmit and Receive generate the beat signal
    % This is done by element wise matrix multiplication of Transmit and
    % Receiver Signal
    Mix(i) = times(Tx, Rx);
end
```

## 3. Range FFT (1st FFT)
I set initial range `d0 = 100`. Below image shows that Range-FFT correctly measures the range. 

![range-fft]

```m
%run the FFT on the beat signal along the range bins dimension (Nr) and normalize.
range_fft = fft(Mix, Nr) ./ Nr;

% Take the absolute value of FFT output
range_fft = abs(range_fft);

% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
range_fft = range_fft(1:(Nr / 2));
```

## 4. 2D CFAR
### Description:
#### The following steps can be used to implement 2D-CFAR: 
1. Determine the number of Training cells for each dimension $T_r$ and $T_d$. Similarly, pick the number of guard cells $G_r$ and $G_d$.
2. Slide the *Cell Under Test (CUT)* across the complete cell matrix.
3. Select the grid that includes the training, guard and test cells. $Grid Size = (2T_r+2G_r+1)(2T_d+2G_d+1)$.
4. The total number of cells in the guard region and cell under test: $(2G_r+1)(2G_d+1)$.
5. This gives the Training Cells: $(2T_r+2G_r+1)(2T_d+2G_d+1) - (2G_r+1)(2G_d+1)$.
6. Measure and average the noise across all the training cells. This gives the threshold.
7. Add the offset (if in signal strength in **dB**) to the threshold to keep the false alarm to the minimum.
8. Determine the signal level at the Cell Under Test.
9.  If the CUT signal level is greater than the Threshold, assign a value of 1, else equate it to zero. 
10. Since the cell under test are not located at the edges, due to the training cells occupying the edges, we suppress the edges to zero. Any cell value that is neither 1 nor a 0, assign it a zero.

![2D_CFAR]

My training, guard cells size and offset:
```m
t_r = 3;
t_d = 2;

g_r = 2;
g_d = 1;

offset = 1.3;
```

I create a matrix `CFAR_2D` which size is same as RDM, and set the default as `0`.
```m
CFAR_2D = zeros(size(RDM));
```

Calculate the value of `grid_size` and `training_cell_size`:
```m
grid_size = (2 * t_r + 2 * g_r + 1) * (2 * t_d + 2 * g_d + 1);
training_cell_size = grid_size - (2 * g_r + 1) * (2 * g_d + 1);
```

Using two for-loop to iterate each *CUT*, compute the noise level and compare with RDM. If the cell in RDM is greater than `threshold`, then set relative cell in `CFAR_2D` to 1, otherwise, set it to 0.
```m
for j = 1:Nd - 2 * (t_r + g_r)
    for i = 1:Nr / 2 - 2 * (t_d + g_d)
        grid_cells = db2pow(RDM(i:i + 2 * (t_d +g_d), j:j + 2 * (t_r + g_r)));
        grid_cells(t_d + 1:t_d + 2 * g_d + 1, t_r + 1:t_r + 2 * g_r + 1) = 0;
        average_noise_level = pow2db(sum(sum(grid_cells)) / training_cell_size);
        threshold = average_noise_level * offset;
        if RDM(i + t_d + g_d, j + t_r + g_r) > threshold
            CFAR_2D(i + t_d + g_d, j + t_r + g_r) = 1;
        else
            CFAR_2D(i + t_d + g_d, j + t_r + g_r) = 0;
        end
    end
end
```

![2D_CFAR_result]




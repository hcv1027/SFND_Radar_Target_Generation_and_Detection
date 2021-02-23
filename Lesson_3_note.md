[resolution]: ./images/resolution.png "resolution"
[range_resolution]: ./images/range_resolution.png "range_resolution"
[signal_trip_time]: ./images/signal_trip_time.png "signal_trip_time"
[range_estimation]: ./images/range_estimation.png "range_estimation"
[range_calculation]: ./images/range_calculation.png "range_calculation"
[doppler_speed_measurement]: ./images/doppler_speed_measurement.png "doppler_speed_measurement"
[FMCW_doppler_estimation]: ./images/FMCW_doppler_estimation.png "FMCW_doppler_estimation"
[doppler_estimation]: ./images/doppler_estimation.png "doppler_estimation"
[animation_doppler_effect]: ./images/animation_doppler_effect.gif "animation_doppler_effect"
[fmcw_prinzip]: ./images/fmcw_prinzip.png "fmcw_prinzip"
[Time_Domain_2_Frequency_Domain]: ./images/Time_Domain_2_Frequency_Domain.png "Time_Domain_2_Frequency_Domain"
[FastFourierTransform]: ./images/FastFourierTransform.png "FastFourierTransform"
[range_fft]: ./images/range_fft.png "range_fft"
[output_of_range_fft]: ./images/output_of_range_fft.png "output_of_range_fft"
[2d_fft_01]: ./images/2d_fft_01.png "2d_fft_01"
[2d_fft_02]: ./images/2d_fft_02.png "2d_fft_02"
[range_doppler_map]: ./images/range_doppler_map.png "range_doppler_map"
[clutter_01]: ./images/clutter_01.png "clutter_01"
[clutter_02]: ./images/clutter_02.png "clutter_02"
[CFAR_01]: ./images/CFAR_01.png "CFAR_01"
[CA-CFAR_01]: ./images/CA-CFAR_01.png "CA-CFAR_01"
[CA-CFAR_02]: ./images/CA-CFAR_02.png "CA-CFAR_02"
[2D_CFAR]: ./images/2D_CFAR.png "2D_CFAR"
[AOA_01]: ./images/AOA_01.png "AOA_01"
[AOA_02]: ./images/AOA_02.png "AOA_02"
[AOA_03]: ./images/AOA_03.png "AOA_03"
[AOA_04]: ./images/AOA_04.png "AOA_04"
[AOA_05]: ./images/AOA_05.png "AOA_05"
[AOA_06]: ./images/AOA_06.png "AOA_06"
[AOA_07]: ./images/AOA_07.png "AOA_07"
[AOA_08]: ./images/AOA_08.png "AOA_08"
[AOA_09]: ./images/AOA_09.png "AOA_09"
[AOA_10]: ./images/AOA_10.png "AOA_10"
[AOA_11]: ./images/AOA_11.png "AOA_11"
[what_is_chirp]: ./images/what_is_chirp.png "what_is_chirp"
[what_is_mixer]: ./images/what_is_mixer.png "what_is_mixer"
[IF_signal]: ./images/IF_signal.png "IF_signal"
[measure_range_01]: ./images/measure_range_01.png "measure_range_01"
[measure_range_02]: ./images/measure_range_02.png "measure_range_02"
[Nyquist–Shannon_sampling_theorem]: ./images/Nyquist–Shannon_sampling_theorem.jpeg "Nyquist–Shannon_sampling_theorem"
[max_range]: ./images/max_range.png "max_range"
[module_1_summary]: ./images/module_1_summary.png "module_1_summary"
[module_1_key_concept]: ./images/module_1_key_concept.png "module_1_key_concept"
[phase_of_IF_signal]: ./images/phase_of_IF_signal.png "phase_of_IF_signal"
[phase_sensitivity]: ./images/phase_sensitivity.png "phase_sensitivity"
[max_velocity]: ./images/max_velocity.png "max_velocity"
[measure_velocity_01]: ./images/measure_velocity_01.png "measure_velocity_01"
[measure_velocity_02]: ./images/measure_velocity_02.png "measure_velocity_02"
[measure_velocity_03]: ./images/measure_velocity_03.png "measure_velocity_03"
[fft_complex_sequency]: ./images/fft_complex_sequency.png "fft_complex_sequency"
[velocity_resolution]: ./images/velocity_resolution.png "velocity_resolution"
[Visualizing_2D-FFT]: ./images/Visualizing_2D-FFT.png "Visualizing_2D-FFT"
[fft_2d_01]: ./images/fft_2d_01.png "fft_2d_01"


# Introduction to mmwaveSensing: FMCW Radars
## Module 1 : Range Estimation
![what_is_chirp]

Define:
- $f_c$: Start frequency
- $B$: Bandwidth
- $T_c$: Duration
- $S$: Slope

![what_is_mixer]

![IF_signal]

Define:
- c: Speed of light
- d: Distance between radar and object

$\tau = 2d / c$

$S_\tau = S \cdot \tau = \frac{S2d}{c}$

![measure_range_01]

* 目標距離與中頻信號頻率$f_0$的關係：$d = \frac{cf_0}{2S}$。其中$c$是光速，$S$是chirp的斜率。
* 距離分辨率：$d_{res} = \frac{c}{2B}$，僅與bandwidth $B$有關。
* 最遠距離：$d_{max} = \frac{F_sc}{2S}$，與chirp斜率$S$和 IF 信號的採樣率($F_s$)有關。

注意：此時的硬件是單TX單RX的配置。
1. 在發射天線的一個 chirp 發射時間 0 ~ $T_c$ 內，接收天線也接收信號，記錄下每個接收到信號的時刻，並完成裁剪信號以及混頻操作，此時混頻器輸出的是由若干個正弦波疊加的模擬信號。注意：這個信號是連續週期模擬信號，最小正週期是所包圍的正弦信號最小正週期的最小公約數。
2. 繪製上述信號的頻譜。以採樣率 $F_s$ 在RX最晚接收到信號的時間 $\tau \to T_c$ 時間區間內進行上述信號的ADC採樣，注意**採樣率必須大於2倍的最大中頻信號頻率**(下面有翻到的證明)。採樣完成後進行FFT運算可以提取出頻譜圖中的峰值，即所有的IF信號中的頻率，每個頻率可以換算出目標的距離，在[參考5](https://zhuanlan.zhihu.com/p/158711587)中給出了使用FFT對信號進行頻率分析的理論和代碼。

這裡稱之為Range-FFT，需要對連續信號採樣，實際上是使用DFT來分析CTFT（連續信號的FFT），只需要提取出波峰所對應的頻率值即可，然後通過上面的公式轉換為距離。

註：
1. 在評論區有一個非常好的問題是如果物體在運動的話，那麼測得的中頻信號會包含有多普勒頻率。在視頻中有一個非常重要的結論是IF信號的相位相對頻率對物體的運動更加敏感，一個例子中假設物體移動了1mm，那麼相位會變化180度，頻率會變化333Hz,看起來很大，但是在整個觀察窗口中佔的比例非常小。
2. 採樣頻率：對於一個連續訊號取樣的時候，會參考Sampling Theorem 定最低取樣頻率。

   Proof:
   ![Nyquist–Shannon_sampling_theorem]
   Nyquist–Shannon sampling theorem 說明了**當取樣頻率為原訊號之最高頻率之兩倍時，才可以正確的重建原始訊號**。
但是通常會使用OverSampling，用遠高於 2f 的頻率來取樣，提升訊號的品質。


![measure_range_02]

![max_range]

![module_1_summary]

![module_1_key_concept]

## Module 2 : The Phase of the IF signal
![phase_of_IF_signal]

$\Delta \Phi = 2\pi f_c \Delta \tau = \frac{4\pi \Delta d}{\lambda}$, where $f_c = c / \lambda$ and $\Delta \tau = 2\Delta d / c$

一個phase($2\pi$) $\times$ 一秒有幾個phase($f_c$) $\times$ 時間($\Delta \tau$) 

![phase_sensitivity]

![measure_velocity_01]

## Module 3 : Velocity Estimation
![max_velocity]

使用位於同一距離處的多個物體進行的速度測量如果速度不同的多個移動物體在測量時與雷達的距離相同，則雙線性調頻脈衝速度測量方法不起作用。這些物體由於與雷達的距離相同，因而會生成IF 頻率完全相同的反射線性調頻脈衝。因此，Range-FFT 會產生單個峰值，該峰值表示來自所有這些距離相同的物體的合併信號。簡單的相位比較技術將不起作用。

![measure_velocity_02]

在這種情況下，為了測量速度，雷達系統必須發射兩個以上的線性調頻脈衝。它發射一組 N 個等間隔線性調頻脈衝。這組線性調頻脈衝稱為線性調頻脈衝幀(frame)。

Range-FFT 處理反射的一組線性調頻脈衝，從而產生一組 N 個位置完全相同的峰值(peak)，但每個峰值都有一個不同的相位(phase)，包含來自這兩個物體的相位成分。稱為Doppler-FFT 的第二個 FFT 在 N 個相量上執行，以分辨兩個物體。

對於等距不同速的兩個物體，在同一幀內，通過Range-FFT 後在峰值處提取各相位，並做Doppler-FFT，會產生兩個具有不同的峰值，其對應的橫坐標為各物體的相位差。

![measure_velocity_03]

注意：這種信號處理稱之為Doppler-FFT，利用DFT來求DTFT的結果。這裡的速度求解的是物體相對雷達的速度，而且方向是物體與雷達的連續方向。

![fft_complex_sequency]

![velocity_resolution]

Note: 
1. $N$ is the number of chirps.
2. $v_{res} = \frac{\lambda}{2NT_c} = \frac{\lambda}{2T_f}$

![Visualizing_2D-FFT]

### Summary:
* $v = \frac{\lambda \omega}{4\pi T_c}$，其中$\omega$是Doppler-FFT峰值所對應的頻率。
* $v_{res} = \frac{\lambda}{2NT_c} = \frac{\lambda}{2T_f}$ ($NT_c = T_f$)，速度分辨率與每個chirp的發射時長、每frame的個數有關，亦即與每個frame的時長有關。
* $v_{max} = \frac{\lambda}{4T_c}$，最大速度僅與每個chirp的發射時長有關

## Module 4 : Some System Design topics
![fft_2d_01]

Note: 
1. The x-axis is actually the frequency corresponding to the range-FFT bins, but since range is proportional to the IF frequency, I can equivalently plot this axis as the range axis.
2. The y-axis is actually the discrete angular frequencies corresponding to the Doppler-FFT, but since those discrete angular frequencies are proportional to the velocity, I can equivalently plot this axis as a velocity axis.
3. The Doppler-FFT can only be performed once all the range-FFTs have become available. That is, once all these rows are populated. So there should be sufficient memory in the system to store the contents of all the range-FFTs corresponding to a frame.

## Module 5 : Angle Estimation
![AOA_06]

Why are these two expressions off by a factor of 2?

Ans: 因為上面的公式裡的距離(d)是來回的，下面的公式裡的距離(d)是單趟的。

![AOA_07]

 The peak location is very **insensitive** to small changes in distance between the radar and the object. However, **the phase difference** between these two peaks is going to be given by $2\pi d \sin(\theta)$, $d\sin(\theta)$ being the additional distance, divided by $\lambda$. And once you have measured this phase difference by comparing these two signals, the signals at these two peaks, you can just invert this equation to calculate the angle of arrival.

![AOA_08]

$\sin(\theta)斜率在\theta \approx 0時很大，在\theta \approx 90^\circ時趨近於0$。所以在$\theta \approx 0$時比較sensitive。

![AOA_09]

![AOA_10]

每一個RX收到的phase都是兩個object組合而成的，所以無法區分。

![AOA_11]


---

# Range, Velocity, and Angle Resolution
![resolution]
Capability of a radar to resolve two targets based on differences in their distance, angle and velocity. 

The capability of a radar to resolve targets is very critical for an accurate perception.

**Range Resolution**: It is the capability of the radar to distinguish between two targets that are very close to each other in range. If a radar has range resolution of 4 meters then it cannot separate on range basis a pedestrian standing 1 m away from the car.

The range resolution is solely dependent on the bandwidth of the chirp $B_{sweep}$:

$$
d_{res} = \frac{c}{2B_{sweep}}
$$

![range_resolution]

**Velocity Resolution**: If two targets have the same range they can still be resolved if they are traveling at different velocities. The velocity resolution is dependent on the number of chirps. As discussed for our case we selected to send 128 chirps. A higher number of chirps increases the velocity resolution, but it also takes longer to process the signal.

**Angle Resolution**: Radar is capable of separating two targets spatially. If two targets are at similar range travelling at same velocities, then they can still be resolved based on their angle in radar coordinate system. Angle resolution depends on different parameters depending on the angle estimation technique used. We will cover this in more detail in the next lesson.

---

# Range Estimation
## Range Estimation Overview
![signal_trip_time]
Radar determines the range of the target by measuring the trip time of the electromagnetic signal it radiates. It is known that EM wave travels at a known speed (300,000,000 m/s), so to determine the range the radar needs to calculate the trip time. How?

Answer : **By measuring the shift in the frequency.**


## Range Estimation Equation
![range_estimation]

The FMCW waveform has the characteristic that the frequency varies linearly with time. If radar can determine the delta between the received frequency and hardware's continuously ramping frequency then it can calculate the trip time and hence the range. We further divide Range estimate by 2, since the frequency delta corresponds to two way trip.

It is important to understand that if a target is stationary then a transmitted frequency and received frequency are the same. But, the ramping frequency within the hardware is continuously changing with time. So, when we take the delta (beat frequency) between the received and ramping frequency we get the trip time.

$$
R = \frac{c T_s f_b}{2B_{sweep}}
$$

Here, $f_b$ is the beat frequency, which is measured by the radar by subtracting the received frequency from the hardware's ramping frequency:

$$
f_b ​= f_{ramping}​ − f_{received​}
$$

As seen in the equation, the range calculation requires chirp time $T_s$​ and chirp Bandwidth $B_{sweep}$. Those values are determined as we define the configuration of the radar based on its range resolution and trip time for Radar's maximum range.

## System Level Range Calculation
![range_calculation]
Range Calculation - System Level

As seen in the image above, the Synthesizer generates FMCW chirp for a given $B_{sweep}$ and $T_s$. Let's say the signal gets transmitted at 77GHz and it returns to the radar after hitting the target in a certain time duration. The radar receiver captures the signal, processes (subtraction) and measures the frequency delta between received signal and linearly ramping signal. This delta in frequency is called as beat frequency and it is proportional to the trip time. So based on the equation above, the radar calculates the range.

## Range Estimation Exercise
Using the following MATLAB code sample, complete the TODOs to calculate the range in meters of four targets with respective measured beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz].

You can use the following parameter values:
* The radar maximum range = 300m
* The range resolution = 1m
* The speed of light $c = 3 \cdot 10^8$

Note : The sweep time can be computed based on the time needed for the signal to travel the maximum range. In general, for an FMCW radar system, the sweep time should be at least 5 to 6 times the round trip time. This example uses a factor of 5.5:

$T_{chrip} = 5.5 \cdot 2 \cdot R_{max​} / c$

```m
% Speed of light
c = 3 * 10^8;
% Range resolution
d_res = 1;
% Radar maximum range
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
```

---

# Doppler Estimation
## Doppler Estimation Overview
![doppler_speed_measurement]

The velocity estimation for radar is based on an age old phenomenon called the doppler effect. As per doppler theory an approaching target will shift an emitted and reflected frequency higher, whereas a receding target will shift the both frequencies to be lower than the transmitted frequency.

## Doppler-Effect
![animation_doppler_effect]

An animation illustrating how the Doppler effect causes a car engine or siren to sound higher in pitch when it is approaching than when it is receding. The red circles represent sound waves.


## FMCW Doppler Measurements
![FMCW_doppler_estimation]

![fmcw_prinzip]

As discussed above, there will be a shift in the received signal frequency due to the doppler effect of the target's velocity. The doppler shift is directly proportional to the velocity of the target as shown below.

$$
f_d = \frac{2 \nu_r}{\lambda}
$$

* $f_d$: shift in the transmitted frequency due to the doppler
* $\nu_r$: relative velocity of the target
* $\lambda$: wavelength of the signal

$f_r$: Shifted frequency

$f$: Original frequency

$$
\begin{aligned}
    f_r &= f(\frac{1+v_r/c}{1-v_r/c}) \\
        &= f(\frac{c+v_r}{c-v_r})
\end{aligned}
$$

The "beat frequency", (Doppler frequency $f_d$), is thus:

$f_d = f_r - f =(\frac{c+v_r}{c-v_r} - 1)f = \frac{2v_r}{c-v_r}f \underset{v_r \llless c}{\approx} \frac{2v_r}{c}f = \frac{2v_r}{\lambda}$
  
By measuring the shift in the frequency due to doppler, radar can determine the velocity. The receding target will have a negative velocity due to the frequency dropping lower, whereas the approaching target will have positive velocity as the frequency shifts higher.

The beat frequency comprises of both frequency components: $f_r$​ (frequency delta due to range) and $f_d$ (frequency shift due to velocity). Although, in the case of automotive radar the $f_d$ is very small in comparison to the $f_r$. Hence, the doppler velocity is calculated by measuring the rate of change of phase across multiple chirps.

The following equation shows the relationship between the rate of change of the phase $\Phi$, and the frequency:

$$
\frac{d\Phi}{dt} = frequency
$$

## DOPPLER PHASE SHIFT
Keeping that in consideration, we calculate the doppler frequency by measuring the rate of change of phase. The phase change occurs due to small displacement of a moving target for every chirp duration. Since, each chirp duration is generally in microseconds, it results in small displacement in mm (millimeters). These small displacements for every chirp leads to change in phase. Using this rate of change of phase we can determine the doppler frequency.

Let's see how :

If the path between a target and the radar is changed by an amount Δx, the phase of the wave received by radar is shifted by

$$
\Delta\varphi = \frac{\Delta x}{\lambda} \qquad (\lambda = 2\pi \, or \, 360^\circ)
$$
$$
\Delta\varphi = f \ast \frac{\Delta x}{c} \qquad (\lambda = c / f)
$$

where $\lambda$ and f are, respectively, the wavelength and frequency of the signal and c is the speed of propagation. The resulting change in observed frequency is:
$$
\Delta f = \frac{\Delta \varphi}{\Delta t}
$$
where $\Delta t$ is the time taken for the observation of the phase change.

![doppler_estimation]

## Doppler Estimation Exercises
Using the following MATLAB code sample, complete the TODOs to calculate the velocity in m/s of four targets with the following doppler frequency shifts: [3 KHz, -4.5 KHz, 11 KHz, -3 KHz].

You can use the following parameter values:
* The radar's operating frequency = 77 GHz
* The speed of light $c = 3 \cdot 10^8$

```m
% Doppler Velocity Calculation
c = 3 * 10^8; %speed of light
frequency = 77e9; %frequency in Hz

% TODO: Calculate the wavelength
wavelength = c / frequency;

% TODO: Define the doppler shifts in Hz using the information from above
doppler_shifts = [3e3 -4.5e3 11e3 -3e3];

% TODO: Calculate the velocity of the targets fd = 2*vr/lambda
Vr = doppler_shifts * wavelength / 2;

% TODO: Display results
disp(Vr);

```

## Doppler Estimation Further Research

---

# Fast Fourier Transform (FFT)
## FFT Overview
So far we discussed the theory of range and doppler estimation along with the equations to calculate them. But, for a radar to efficiently process these measurements digitally, the signal needs to be converted from analog to digital domain and further from time domain to frequency domain.

ADC (Analog Digital Converter) converts the analog signal into digital. But, post ADC the Fast Fourier Transform is used to convert the signal from time domain to frequency domain. Conversion to frequency domain is important to do the spectral analysis of the signal and determine the shifts in frequency due to range and doppler.

![Time_Domain_2_Frequency_Domain]

The traveling signal is in time domain. Time domain signal comprises of multiple frequency components as shown in the image above. In order to separate out all frequency components the FFT technique is used.

For the purpose of this course we don’t have to get into mathematical details of FFT. But, it is important to understand the use of FFT in radar’s digital signal processing. It gives the frequency response of the return signal with each peak in frequency spectrum representing the detected target’s chararcterstics.

Learn more on FFT implementation [here](https://www.youtube.com/watch?v=t_NMmqTRPIY&feature=youtu.be).

## FFT and FMCW
![FastFourierTransform]

As seen in the image below, the Range FFTs are run for every sample on each chirp. Since each chirp is sampled N times, it will generate a range FFT block of N * (Number of chirps). These FFT blocks are also called FFT bins.

![range_fft]

Each bin in every column of the block represents an increasing range value so that the end of the last bin represents the maximum range of a radar. 

![output_of_range_fft]

Above is the output of the 1st stage FFT (i.e Range FFT). The three peaks in the frequency domain corresponds to the beat frequencies of three different cars located at 150, 240 and 300 m range from the ego vehicle.

## Fast Fourier Transform Exercise
In the following exercise, you will use a Fourier transform to find the frequency components of a signal buried in noise. Specify the parameters of a signal with a sampling frequency of 1 kHz and a signal duration of 1.5 seconds.

To implement the 1st stage FFT, you can use the following steps:

1. Define a signal. In this case (amplitude = A, frequency = f)
   ```m
   signal = A * sin(2 * pi * f * t)
   ```
2. Run the FFT for the signal using the MATLAB FFT function for the dimension of samples N. 
   ```m
   signal_fft = fft(signal, N);
   ```
   This returns the N-point DFT. If `N` is not specified, `signal_fft` is the same size as `signal`.
3. The output of FFT processing of a signal is a complex number (a+jb). Since we just care about the magnitude we take the absolute value ($(a^2+b^2)^{1/2}$) of the complex number.
   ```m
   signal_fft = abs(signal_fft);
   ```
4. FFT output generates a mirror image of the signal. But we are only interested in the positive half of signal length L, since it is the replica of negative half and has all the information we need.
   ```m
   signal_fft  = signal_fft(1:L/2+1)   
   ```
   Reference: [Converting a Two-Sided Power Spectrum to a Single-Sided Power Spectrum](https://zone.ni.com/reference/en-XX/help/371361J-01/lvanlsconcepts/lvac_convert_twosided_power_spec_to_singlesided/)
5. Plot this output against frequency.
   
You can use the following MATLAB starter code:
```m
Fs = 1000; % Sampling frequency
T = 1 / Fs; % Sampling period
L = 1500; % Length of signal
t = (0:L - 1) * T; % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
S = 0.7 * sin(2 * pi * 77 * t) + 2 * sin(2 * pi * 43 * t);

% Corrupt the signal with noise
X = S + 2 * randn(size(t));

% Plot the noisy signal in the time domain.
% It is difficult to identify the frequency components by looking at the signal X(t).
plot(1000 * t(1:50), X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal.
signal_fft = fft(X);

% TODO : Compute the two-sided spectrum P2.
% Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
% We take the amplitude of the normalized signal
P2 = abs(signal_fft / L);
% At the end, we just compute the single-sided spectrum as we reject the mirror image.
P1 = P2(1:L / 2 + 1)

% Plotting
f = Fs * (0:(L / 2)) / L;
plot(f, P1)
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
```

---

# The 2D FFT
Once the range bins are determined by running range FFT across all the chirps, a second FFT is implemented along the second dimension to determine the doppler frequency shift. As discussed, the doppler is estimated by processing the rate of change of phase across multiple chirps. Hence, the doppler FFT is implemented after all the chirps in the segment are sent and range FFTs are run on them.

The output of the first FFT gives the beat frequency, amplitude, and phase for each target. This phase varies as we move from one chirp to another (one bin to another on each row) due to the target’s small displacements. Once the second FFT is implemented it determines the rate of change of phase, which is nothing but the doppler frequency shift.

![2d_fft_01]

The output of Range Doppler response represents an image with Range on one axis and Doppler on the other. This image is called as Range Doppler Map (RDM). These maps are often used as user interface to understand the perception of the targets. 

![2d_fft_02]

![range_doppler_map]

## 2D FFT In MATLAB
The following steps can be used to compute a 2D FFT in MATLAB:
1. Take a 2D signal matrix 
2. In the case of Radar signal processing. Convert the signal in MxN matrix, where M is the size of Range FFT samples and N is the size of Doppler FFT samples:
   ```m
   signal  = reshape(signal, [M, N]);
   ```
3. Run the 2D FFT across both the dimensions.
   ```m
   signal_fft = fft2(signal, M, N);
   ```
   Note the following from the [documentation]()
4. Shift zero-frequency terms to the center of the array
   ```m
   signal_fft = fftshift(signal_fft);
   ```
5. Take the absolute value
   ```m
   signal_fft = abs(signal_fft);
   ```
6. Here since it is a 2D output, it can be plotted as an image. Hence, we use the `imagesc` function
   ```m
   imagesc(signal_fft);
   ```

## 2D FFT Exercise
In the following exercise, you will practice the 2D FFT in MATLAB using some generated data. The data generated below will have the correct shape already, so you should just need to use steps 3-6 from above. You can use the following starter code:
```m
```

---

# Clutter
False alarm: A false alarm is **"an erroneous radar target detection decision caused by noise or other interfering signals exceeding the detection threshold"**. In general, it is an indication of the presence of radar target when there is no valid aim.

Radar not only receive the reflected signals from the objects of interest, but also from the environment and unwanted objects. The backscatter from these unwanted sources is called as **clutter** (雜波).

These unwanted signals are generally produced by the reflections from the ground, sea, buildings, trees, rain, fog etc. The magnitude of the clutter signal depends upon:
* The nature of the surface - ground, water, snow (e.g deserts have low reflectivity, whereas the frozen snow has high reflectivity)
* Smoothness of the surface
* Grazing angle - Angle the radar beam makes with the surface
* Radar Frequency

![clutter_01]

source : [http://www.redalyc.org/jatsRepo/911/91149521004/index.html](http://www.redalyc.org/jatsRepo/911/91149521004/index.html)

## Clutter Thresholding
It is important to filter out clutter for successful detection of targets. This is critical in a driving scenario to avoid the car from suddenly braking in the absence of valid targets. This sudden braking happens when the radar detects reflections that are generated from the clutter.

One technique to remove clutter is to remove the signals having 0 doppler velocity. Since, the clutter in the driving scenario are often created by the stationary targets, the 0 doppler filtering can help get rid of them.

The downside of 0 doppler filtering is that the radar would not be able to detect the stationary targets in its path. This would lead to detection failures.

Another technique is to use *fixed clutter thresholding*. With fixed thresholding, signal below the threshold value is rejected. With this method, if the detection threshold is set too high, there will be very few false alarms, but it will also mask the valid targets. If the threshold is set too low, then it would lead to too many false alarms. In other words, the *false alarm rate* would be too high.

The false alarm rate is the rate of erroneous radar detections by noise or other interfering signals. It is a measure of the presence of detected radar targets when there is no valid target present.

![clutter_02]

## Dynamic Thresholding
Another approach to clutter thresholding is to use dynamic thresholding. Dynamic thresholding involves varying the threshold level to reduce the false alarm rate.

In the rest of this lesson, you will learn about a dynamic thresholding technique called **CFAR** (Constant False Alarm Rate). With this technique, the noise at every or group of range/doppler bins is monitored and the signal is compared to the local noise level. This comparison is used create a threshold which holds the false alarm rate constant. Let's have a look in the next concept!

## Further Research
See the resources [here](http://www.radartutorial.eu/11.coherent/co04.en.html) and [here](https://journals.sagepub.com/doi/pdf/10.1177/1550147717729793) for further information about clutter.

---

# CFAR (Constant False Alarm Rate)
Blue wearing line is CFAR threshold, the rest three are fixed thresholds with different offsets.

![CFAR_01]

The false alarm issue can be resolved by implementing the constant false alarm rate. CFAR varies the detection threshold based on the vehicle surroundings. The CFAR technique estimates the level of interference in radar range and doppler cells "Training Cells" on either or both the side of the "Cell Under Test". The estimate is then used to decide if the target is in the *Cell Under Test* (**CUT**).

The process loops across all the range cells and decides the presence of target based on the noise estimate. The basis of the process is that when noise is present, the cells around the cell of interest will contain a good estimate of the noise, i.e. it assumes that the noise or interference is spatially or temporarily homogeneous. Theoretically it will produce a constant false alarm rate, which is independent of the noise or clutter level.

There are multiple categories of CFAR:
* Cell Averaging CFAR (CA-CFAR)
* Ordered Statistics CFAR (OS CFAR)
* Maximum Minimum Statistic (MAMIS CFAR)
* And, multiple variants of CA-CFAR. 

Here, we will be covering the basic CA-CFAR.

## CA-CFAR
![CA-CFAR_01]

CA-CFAR is the most commonly used CFAR detection technique. As seen in the previous lesson, the FFT blocks are generated on implementing range and doppler FFTs across the number of chirps. The CFAR process includes the sliding of a window across the cells in FFT blocks. Each window consists of the following cells:

**Cell Under Test**: The cell that is tested to detect the presence of the target by comparing the signal level against the noise estimate (threshold).

**Training Cells**: The level of noise is measured over the Training Cells. The Training Cells can be divided into two regions, the cells lagging the CUT, called lagging Training Cells and the cells leading the CUT, called Leading Training Cells. The noise is estimated by averaging the noise under the training cells. In some cases either leading or lagging cell average is taken, while in the other the leading and lagging cell average is combined and the higher of two is considered for noise level estimate.

The number of training cells should be decided based on the environment. If a dense traffic scenario then the fewer training cells should be used, as closely spaced targets can impact the noise estimate.

**Guard Cells**: The cells just next to CUT are assigned as Guard Cells. The purpose of the Guard Cells is to avoid the target signal from leaking into the training cells that could adversely affect the noise estimate. The number of guard cells should be decided based on the leakage of the target signal out of the cell under test. If target reflections are strong they often get into surrounding bins.

**Threshold Factor (Offset)**: Use an offset value to scale the noise threshold. If the signal strength is defined in logarithmic form then add this offset value to the average noise estimate, else multiply it. 

![CA-CFAR_02]

## 1D CFAR Exercise
The following steps can be used to implement CFAR in the next MATLAB exercise. You can use the code template below to get started as well.

**T**: Number of Training Cells

**G**: Number of Guard Cells

**N**: Total number of Cells

1. Define the number of training cells and guard cells.
2. Start sliding the window one cell at a time across the complete FFT 1D array. Total window size should be: 2(T+G)+CUT.
3. For each step, sum the signal (noise) within all the leading or lagging training cells.
4. Average the sum to determine the noise threshold.
5. Using an appropriate offset value scale the threshold.
6. Now, measure the signal in the CUT, which is T+G+1 from the window starting point.
7. Compare the signal measured in 5 against the threshold measured in 4.
8. If the level of signal measured in CUT is smaller than the threshold measured, then assign 0 value to the signal within CUT.

---

# CFAR 2D
The 2D CFAR is similar to 1D CFAR, but is implemented in both dimensions of the range doppler block. The 2D CA-CFAR implementation involves the training cells occupying the cells surrounding the cell under test with a guard grid in between to prevent the impact of a target signal on the noise estimate.
![2D_CFAR]

## 2D CFAR Steps
You won't need to implement a 2D-CFAR yet, but you will implement a 2D CFAR on the range doppler output for your final project! The following steps can be used to implement 2D-CFAR in MATLAB: 
1. Determine the number of Training cells for each dimension $T_r$ and $T_d$. Similarly, pick the number of guard cells $G_r$ and $G_d$.
2. Slide the *Cell Under Test (CUT)* across the complete cell matrix.
3. Select the grid that includes the training, guard and test cells. $Grid Size = (2T_r+2G_r+1)(2T_d+2G_d+1)$.
4. The total number of cells in the guard region and cell under test: $(2G_r+1)(2G_d+1)$.
5. This gives the Training Cells: $(2T_r+2G_r+1)(2T_d+2G_d+1) - (2G_r+1)(2G_d+1)$.
6. Measure and average the noise across all the training cells. This gives the threshold.
7. Add the offset (if in signal strength in **dB**) to the threshold to keep the false alarm to the minimum.
8. Determine the signal level at the Cell Under Test.
9. If the CUT signal level is greater than the Threshold, assign a value of 1, else equate it to zero. 
10. Since the cell under test are not located at the edges, due to the training cells occupying the edges, we suppress the edges to zero. Any cell value that is neither 1 nor a 0, assign it a zero.

---

# Angle of Arrival Introduction
![AOA_01]

A *phased array antenna* is an antenna array that steers the beam electronically in the desired direction. The array steers the beam if each antenna element in an array is excited by the signal with certain phase values. This phenomenon is referred to as beam scanning. 

![AOA_02]

For antenna beam to steer in a desired direction, the phase shifters are programmed to have constant phase increments. If an antenna comprises of six radiating elements and the phase delta required to steer a beam in a given direction is 15 degrees, then the following would be the phase value on each element [0,15,30,45,60,75] degrees. The increment phase shift along with the spacing between antenna elements (d) determines the steering angle of an antenna using the following equation:

$$
\Phi = \frac{360 \cdot d \cdot \sin(\theta)}{\lambda}
$$

* $\Phi$: incremental phase shift
* $d$: spacing between antenna elements
* $\theta$: steering direction from the normal of the antenna surface
* $\lambda$: wavelength of the signal
  
As the radar scan the surroundings by steering the beam at the programmed angles, it can sense the angle of the return signal. This helps Radar create a spatial perception of the environment.

![AOA_03]

## Angle of Arrival
![AOA_04]

As the radar scans the surroundings by steering the beam at the programmed angles, it measures the SNR of reflected signals from targets located at different angles spatially. This helps in creating an angle of arrival vs SNR grid for radar's spatial perception.

![AOA_05]

## Further Research
For more information about phased array antennas see [here](http://www.radartutorial.eu/06.antennas/Phased%20Array%20Antenna.en.html).

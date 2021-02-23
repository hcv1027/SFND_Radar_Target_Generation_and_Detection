[wavelength]: ./images/wavelength.png "wavelength"
[frequency]: ./images/frequency.png "frequency"
[amplitude]: ./images/amplitude.png "amplitude"
[period]: ./images/period.svg "period"
[phase_shift]: ./images/phase_shift.svg "phase_shift"
[vertical_shift]: ./images/vertical_shift.svg "vertical_shift"
[phase_of_sinusoidal_waveform]: ./images/phase_of_sinusoidal_waveform.png "phase_of_sinusoidal_waveform"
[phase_of_a_signal]: ./images/phase_of_a_signal.png "phase_of_a_signal"
[wave_equation]: ./images/wave_equation.png "wave_equation"
[wave_and_cos]: ./images/wave_and_cos.png "wave_and_cos"
[fmcw_waveform]: ./images/fmcw_waveform.png "fmcw_waveform"
[fmcw_hardware]: ./images/fmcw_hardware.png "fmcw_hardware"
[radar_illumination]: ./images/radar_illumination.png "radar_illumination"
[antenna_pattern]: ./images/antenna_pattern.png "antenna_pattern"
[rcs]: ./images/rcs.png "rcs"
[signal_propagation]: ./images/signal_propagation.png "signal_propagation"
[signal_to_noise_ratio]: ./images/signal_to_noise_ratio.png "signal_to_noise_ratio"

# Signal properties

## Single Wave Parameters
Wavelength $\lambda$ is the physical length from one point of a wave to the same point on the next wave, and it is calculated as
$$
\lambda = \frac{speed \, of \, light}{frequency}
$$
The higher the frequency the smaller the wavelength.

![wavelength]

The *frequency* of a wave is the number of waves that pass by each second, and is measured in Hertz (Hz). The automotive radar generally operates at W band (76GHz - 81GHz). The signal at this frequency is referred to as millimeterWave since the wavelength is in mm.

The *Bandwidth* of a signal is the difference between the highest and the lowest frequency components in a continous band of frequencies.

The *Amplitude* is the strength of the signal. Often it corresponds to the power of the RF signal/electromagnetic field defined in dB/dBm. It is relevant while configuring the output power of the radar and sensing the received signal. Higher the amplitude of the Radar signal, more is the visibility of radar. Automotive Radar can operate at max of 55 dBm output power (316 W)

dB, dBm, mW, and W conversions can be found [here](https://www.rapidtables.com/electric/dBm.html).

![frequency]

![amplitude]

The **Period** goes from one peak to the next (or from any point to the next matching point):

![period]

$$
frequency = \frac{1}{period}
$$

$$
period = \frac{1}{frequency}
$$

The **Amplitude** is the height from the center line to the peak (or to the trough). Or we can measure the height from highest to lowest points and divide that by 2.

The **Phase Shift** is how far the function is shifted **horizontally** from the usual position.

![phase_shift]

The **Vertical Shift** is how far the function is shifted **vertically** from the usual position.

![vertical_shift]

## Phase of a Signal
*Phase* is a particular point in time on the cycle of a waveform, measured as an angle in degrees. A complete cycle is 360°. The phase for each argument value, relative to the start of the cycle, is shown in the image below, in degrees from 0° to 360° and in radians from 0 to 2π.

The frequency can also be defined as the first derivative of the phase with respect to the time.
$$
frequency = \frac{d\varphi}{dt}
$$

where

$$
\varphi = phase \, of \, the \, signal
$$

This property will be used in measuring the doppler frequency shift for a moving target.

![phase_of_sinusoidal_waveform]

The difference between the phases of two periodic signals is called the *phase difference*. At values of when the difference is zero, the two signals are said to be in phase, otherwise they are out of phase with each other.

The phase information processing is important as we go through doppler processing as well as Angle of Arrival techniques for radar.

![phase_of_a_signal]

## General Equation of a Wave
A wave travelling in space is defined by the following equation:
$$
y(t) = A \cos(2 \pi \cdot f_c \cdot t + \phi)
$$

* $A$ is the amplitude of the signal
* $f_c$ is the signal frequency
* $\phi$ is the phase of the signal

![wave_equation]

![wave_and_cos]

---

# FMCW radar (Frequency-Modulated Continuous Wave radar)

## FMCW Chirps
*What is a chirp?*

A chirp is a radar signal whose frequency is increasing or decreasing in time.

A *Frequency Modulated Continous Wave* (FMCW) is a signal in which the frequency increases/decreases with time. They are also referred to as upramps and downramps. The two most common waveform pattern used for FMCW radars are sawtooth and triangular. The sawtooth waveform generally uses just the upramps, whereas the triangular waveform uses both upramps and downramps.

Each **chirp** is defined by its slope. The slope is given by its chirp frequency bandwidth $B$ or $B_{sweep}$ (y-axis) and its chirp time $T_s$ (x-axis). Hence,

$$
Slope = \frac{B}{T_s}
$$

![fmcw_waveform]

The range resolution requirement decides the $B$, whereas the maximum velocity capability of a radar is determined by the chirp time $T_s$.

One *chirp sequence* or *segment* comprises of multiple chirps. Each chirp is sampled multiple times to give multiple range measurements and radar transmits in order to measure doppler velocity accurately.

$\tau$ denotes the round-trip time between the radar and the object

More reading:
1. [Continuous Wave Radar](https://www.radartutorial.eu/02.basics/Continuous%20Wave%20Radar.en.html)
2. [Frequency-Modulated Continuous-Wave Radar (FMCW Radar)](https://www.radartutorial.eu/02.basics/Frequency%20Modulated%20Continuous%20Wave%20Radar.en.html)

---

# FMCW Hardware
## FMCW Hardware Overview
![fmcw_hardware]

**Frequency Synthesizer**: The *frequency synthesizer* is the component that generates the frequency to bring the chirp frequency all the way to 77GHz in case of automotive radar.

**Power Amp**: The *power amp* amplifies the signal so the signal can reach long distance. Since the signal attenuates as it radiates, it needs higher power (amplitude) to reach targets at greater distances.

**Antenna**: The *antenna* converts the electrical energy into electromagnetic waves which radiate through the air, hit the target, and get reflected back toward the radar receiver antenna. The Antenna also increases the strength of the signal by focusing the energy in the desired direction. Additionally, the antenna pattern determines the field of view for the radar.

**Mixer**: In FMCW radar, the *mixer* multiplies the return signal with the sweeping signal generated by the frequency synthesizer. The operation works as frequency subtraction to give the frequency delta - also known as frequency shift or Intermediate frequency (IF). IF = Synthesizer Frequency - Return Signal Frequency.

**Processor**: The *processor* is the processing unit where all the Digital Signal processing, Detection, Tracking, Clustering, and other algorithms take place. This unit could be a microcontroller or even an FPGA.

## Antenna Details
As defined in the FMCW Hardware definitions, the antenna is a transducer that converts the electrical energy into electromagnetic waves. In the case of radar, these waves travel through the air and hit the target. Depending on the surface type and shape of the target, the waves get partially reflected back in the direction of the radar. The receiver antenna at the radar amplifies the received signal further and sends it to the receiver chain for further processing.

### The Antenna Pattern
![radar_illumination]

The *antenna pattern* is the geometric pattern of the strengths of the relative field emitted by the antenna.

The *beamwidth* of the antenna determines the field of view for the radar sensor. If the requirement for the radar is to just sense the targets in its own lane then the beamwidth needs to be small enough to cover the complete lane up to desired range. If the beamwidth is wider than the lane width, it will sense the targets in the other lanes as well.

![antenna_pattern]

Antenna radiation not only comprises of the main beam but the sidelobes as well. Antenna *sidelobes* are critical because they can generate false alarms and pick interference from undesired direction. As seen in the pattern, the sidelobes of the antenna point in different directions and can sense targets that are not in the main beam. To avoid sidelobe detections it is critical to suppress the sidelobe levels to more than 30dB from the peak of the main beam.

### Antenna Types
There are many types of antenna (dipole, patch, horn) that can be used at 77GHz, but the most commonly used antenna type in automotive radar is the patch antenna . The low cost, easy fabrication, and low profile of Patch Array Antennas makes them an ideal choice for automotive radar applications.

---

# Radar Cross Section
## Radar Cross Section Overview
The size and ability of a target to reflect radar energy is defined by a single term, $\sigma$, known as the radar cross-section, which has units of $m^2$. This unit shows that the radar cross section is an area. The target radar cross sectional area depends on:

* The target's physical geometry and exterior features:
  - Smooth edges or surface would scatter the waves in all directions, hence lower RCS. Whereas, sharp corners will focus the return signal back in the direction of the source leading to higher RCS. (Image below for different target geometries)
* The direction of the illuminating radar,
* The radar transmitter’s frequency,
* The material used in the cars, trucks, bicycles, and even in some cases, the clothing material for pedestrians.

![rcs]

source : [https://arxiv.org/pdf/1607.02434.pdf](https://arxiv.org/pdf/1607.02434.pdf)

## RCS Units
This RCS can also be defined using a logarithmic value (dB), since it increases the return signal strength. The formula for converting from RCS to dB is given by:
$$
RCS_{dB} = 10 \log(RCS_{m^2})
$$

---

# Radar Range Equation
## Range Equation Overview
Using the Radar Range equation we can design the radar transmitter, receiver, and antenna to have the desired power, gain and noise performance to meet the range requirements.

A long range radar designed to cover 300m range and detect a target with smaller cross section would need higher transmit power and more antenna gain as compared to a short range radar designed to cover just 50m for similar target. A target with higher cross section can be detected at a longer range as compared to a target with smaller cross section.

$$
R = \sqrt[4]{\frac{P_S \cdot G^2 \cdot \lambda^2 \cdot \sigma}{P_E \cdot (4\pi)^3}}
$$

* $R$ - Maximum Range a radar can detect targets.
* $P_s$​ - Transmitted Power from Radar (dBm)
* $G$ - Gain of the Transmit/Receive Antenna (dBi)
* $\lambda$ - Wavelength of the signal ($m$)
* $\sigma$ - radar cross section ($m^2$)
* $P_E$ - Minimum received power radar can detect.

![signal_propagation]

The image above shows the variation in the signal strength level as it travels through transmitter, over the air and at the receiver

The image above shows the variation in the signal strength level :

* The transmitter power
* Power Amplifiers further increase the signal strength - Transmit chain gain
* Signal is further amplified using an antenna
* One Way Path Loss represents the loss in the signal strength as it travels towards the target
* On getting reflected from the target the signal gets amplified based on the RCS of the target
* After RCS gain the signal travel back towards the radar and has similar loss in strength as going forward
* The receiver antenna amplifies the return signal before sending it to the processing unit

## Radar Detection
Below is an illustration showing the output of a radar's range detection. The peaks correspond to the strength of the return signal from targets and the frequency relates to the range. Relationship between frequency and range will be discussed in next lesson.

A radar cannot detect a signal that is below the noise level. The noise level is determined by the thermal noise generated by the receiver. To successfully detect a target, the return signal strength needs to be larger than the noise level. This is defined by a property called *signal to noise ratio*, or SNR.

SNR is a quantitative measure of a signal strength as compared to the level of noise. If the SNR is too low it becomes difficult for a radar to distinguish the signal from noise. Hence, higher SNR is desirable for successful detection of the target. Generally, a 7-13 dB SNR ensures successful detection in a road scenario.

![signal_to_noise_ratio]

The image above shows the logarithmic value of SNR = power level (in dBm) - noise level (dBm). The plot shows the output of Range FFT (discussed in Lesson 3). In general, the higher the SNR value, the greater are the chances of successful Radar detection.


# Radar Signal Processing – Delay Estimation and Target Localization

This project implements radar signal detection under noisy conditions using matched filtering in MATLAB/Octave. It was developed as part of the "Advanced Signal Processing" course at the Hebrew University of Jerusalem.

## 📌 Overview
The goal of this project is to estimate the delay of a received radar signal and use it to compute the 2D position of a target (e.g., an airplane on the runway) based on signal returns from two antennas.

Key topics include:
- Matched Filter implementation
- Maximum A Posteriori (MAP) delay estimation
- Delay-to-distance conversion using signal propagation speed
- Triangulation via circle and ellipse intersection
- Real-time target tracking based on periodic radar pulses

## 💡 Main Features
- Processing discrete-time chirp signals under Gaussian noise
- Estimating delay via peak detection in the matched filter output
- Calculating 2D target coordinates `(x, y)` from two delay values
- Plotting the target’s path over time
- Implemented entirely in MATLAB

## 📁 Files
- `sigvec.mat` – Transmitted radar pulse
- `delayedvecs.mat`, `delayedvecsQ2.mat` – Received signals for delay estimation
- `radarreception.mat` – Long received signal for target tracking
- `radardetect.m` – Main function implementing delay estimation and localization
- Additional scripts for MAP estimation and plotting

## 🛠️ Technologies
- MATLAB (compatible with Octave)
- Signal processing fundamentals
- Basic geometric localization

## 📘 Academic Context
This project was developed as a homework assignment for the Advanced Signal Processing course (Fall 2022–2023) at the Hebrew University of Jerusalem.

## 📷 Example Output
*(Add your plot image here if available, e.g., airplane path plot)*

---

Want to explore more? Check out the code and run it on example `.mat` files to simulate radar target tracking in 2D.

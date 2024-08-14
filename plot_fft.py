import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.signal import windows
from scipy.signal import savgol_filter, medfilt

# Parameters for Savitzky-Golay filter
window_length = 7  # Window length must be odd and greater than polyorder
polyorder = 3       # Polynomial order to fit over the window
kernel_size = 9       # FOr median filter

# Read csv file
df1 = pd.read_csv('/home/aaron/microros_ws/output.csv')

# Process data, connect every message together
data_columns1 = [col for col in df1.columns if 'data_' in col]
data1 = df1[data_columns1].values.flatten()  # flatten data point

# Apply Savitzky-Golay filter or Median filter (uncomment them if use it)
# data1 = savgol_filter(data1, window_length, polyorder)
# data1 = medfilt(data1, kernel_size=kernel_size)

# # Apply Hann window (uncomment them if use it)
# window = windows.hann(data1.size)
# data1 = data1 * window

# Run FFT
fft_result1 = fft(data1)

# Calculate frequency
sample_rate = 44100
freq1 = np.fft.fftfreq(data1.size, d=1/sample_rate)

# Select positive frequency part
positive_freq_indices1 = freq1 > 0
positive_freqs1 = freq1[positive_freq_indices1]
positive_fft_result1 = fft_result1[positive_freq_indices1]

# Plot
plt.figure(figsize=(15, 5))  # Create a single plot
plt.plot(positive_freqs1, np.abs(positive_fft_result1))  # freq vs amplitude
plt.title('FFT of Signal.csv')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
# plt.xlim(1000, 15000)  # Set x-axis limits to 1 to 600 Hz
plt.ylim(0, 20000000)

plt.grid(True)
plt.tight_layout()  # Adjust layout to prevent overlap
plt.show()

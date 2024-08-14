import pandas as pd
import matplotlib.pyplot as plt

# read csv file
df = pd.read_csv('/home/aaron/microros_ws/output.csv')

# Select the column from csv
time_column = df['time(second)']
data_columns = [col for col in df.columns if 'data_' in col]

plt.figure(figsize=(12, 8))

# Plot data points for each row
for data_col in data_columns:
    plt.scatter(time_column, df[data_col], s=1)  # use scatter, s controls the size of point

plt.title('Signal Plot over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Signal Value (mV)')   # After amplified
plt.ylabel('Signal Value (mV)')   # Before amplified

plt.grid(True)
plt.ylim(-1300, 1300)

plt.show()

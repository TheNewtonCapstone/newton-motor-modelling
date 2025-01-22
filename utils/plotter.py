import pandas as pd
import matplotlib.pyplot as plt
import sys
from random import randint

from matplotlib.pyplot import colors

# Define target value
# file_path = 'log/step_responses/csv/r_4.2'
file_path = sys.argv[1]
independent_arg = sys.argv[2]
dependent_arg = sys.argv[3:]
# Read the CSV file
df = pd.read_csv(file_path)
print(df.head())

# Set up the plotting style
plt.style.use('seaborn')
fig,ax1 = plt.subplots()
fig.tight_layout(pad=3.0)

line_colors= []
for i in range(len(dependent_arg)):
    line_colors.append('#%06X' % randint(0, 0xFFFFFF))

for arg in dependent_arg:
    ax1.plot(
        df[independent_arg],
        df[arg],
        linewidth=2,
        label=arg,
        color= line_colors[dependent_arg.index(arg)]
    )
# ax1.plot(df['current_time'], df['output'], linewidth=2, color='#27AE60', label='Output')
ax1.set_xlabel(independent_arg)
ax1.set_ylabel(dependent_arg)

ax1.grid(True)
ax1.legend()

# Save the plot
plt.show()

# exit(1)

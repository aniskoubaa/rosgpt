import pandas as pd
from sympy import Interval
import matplotlib.pyplot as plt
import numpy as np

# Read the first CSV file
file1 = pd.read_csv('/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_SEM_TOTAL_per_model_human_score.txt', header=None, names=['Model', 'Interval'])
file1[['avg', 'confid']] = file1['Interval'].str.extract(r'(\S+) \\pm (\S+)').astype(float)
file1['lower'] = file1['avg'] - file1['confid']
file1['upper'] = file1['avg'] + file1['confid']
# print(file1[['lower', 'upper']])
print(file1)

# Read the second CSV file
file2 = pd.read_csv('/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_confidence_GPT4score_total_per_model.txt', header=None, names=['Model', 'Interval'])
file2[['avg', 'confid']] = file2['Interval'].str.extract(r'(\S+) \\pm (\S+)').astype(float)
file2['lower'] = file2['avg'] - file2['confid']
file2['upper'] = file2['avg'] + file2['confid']
# print(file2[['lower', 'upper']])
print(file2)

# Create the figure and axis
fig, ax = plt.subplots()

# Plot rectangles
colors = ['blue' if model.startswith('LLaMA') else 'green' for model in file1['Model']]
for i, model in enumerate(file1['Model']):
    rect = plt.Rectangle((file1['lower'][i], file2['lower'][i]),
                         file1['upper'][i] - file1['lower'][i],
                         file2['upper'][i] - file2['lower'][i],
                         linewidth=1, edgecolor='black', facecolor=colors[i])
    ax.add_patch(rect)
    plt.text((file1['lower'][i] + file1['upper'][i]) / 2, (file2['lower'][i] + file2['upper'][i]) / 2, model,
             ha='center', va='center', color='white', size=13, weight='bold') # if model.startswith('GPT') else 'black')

# Set axis labels
plt.xlabel('Human score',size=22)
plt.ylabel('GPT score',size=22)

# Set plot limits
plt.xlim(0, 5)
plt.ylim(0, 5)
plt.xticks(size=22)
plt.yticks(size=22)
# Add grid with interrupted points
plt.grid(True, linestyle='--', linewidth=0.5)

plt.savefig('/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/Figures/average_and_confid_squares.png')
# Show the plot
plt.show()

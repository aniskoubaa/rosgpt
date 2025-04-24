import re
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Robotic_Arm_SELECTED_5pct_for_manual_eval.txt'
# output_file = 'scores.txt'
output_figure = input_file[:-4]+'-scores-PLOT.png'

# Read the CSV file using pandas
df = pd.read_csv(input_file, delimiter=';')

# Extract the 'Human score' column from index 0 to 49
# scores = df['Human score'][0:50]
scores = df['Human'][0:50].dropna()
print(list(scores))

# Save the scores to a new file
# with open(output_file, 'w') as f:
#     for score in scores:
#         f.write(str(score) + '\n')

# print(f'Scores saved in {output_file}')

# Calculate the average
average = scores.mean()
sem = scores.std()/np.sqrt(50)
print(f"Average: {average:.2f}")

# Plot the box-plot and histogram
plt.figure(figsize=(10, 5))

# Subplot for boxplot
plt.subplot(1, 2, 1)
plt.boxplot(scores)
plt.title("Box plot of Human scores")
plt.ylabel("Value")
plt.xticks([])

# Subplot for histogram
plt.subplot(1, 2, 2)
plt.hist(scores, bins=5, range=(0, 5), edgecolor='black')
plt.title("Histogram of Human scores")
plt.xlabel("Value")
plt.ylabel("Frequency")
plt.xticks(range(6))

plt.tight_layout()

# Save the figure to a file
plt.savefig(output_figure, dpi=300)
plt.show()

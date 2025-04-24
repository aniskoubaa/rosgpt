import re
import matplotlib.pyplot as plt

# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/Evaluated_JSON_commands_UAV.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/Evaluated_JSON_commands_Robotic_Arm.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations//Ground-Robot/Evaluated_JSON_commands_Ground_Robot_v2.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLamA-7b/Evaluated_JSON_commands_LLaMA7b_Ground_Robot.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_UAV.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_Robotic_Arm.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Ground_Robot.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_UAV.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Robotic_Arm.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/GPT4/Evaluated_JSON_commands_GPT4_Ground_Robot.txt'
# input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/GPT4/Evaluated_JSON_commands_GPT4_Robotic_Arm.txt'
input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT4/Evaluated_JSON_commands_GPT4_UAV.txt'

# output_file = input_file[:-4]+'-LLaMA2-7b-SCORES-only.txt'
# output_figure = input_file[:-4]+'-LLaMA2-7b-plots.png'
# output_file = input_file[:-4]+'-LLaMA2-70b-SCORES-only.txt'
# output_figure = input_file[:-4]+'-LLaMA2-70b-plots.png'
# output_file = input_file[:-4]+'-GPT3.5-SCORES-only.txt'
# output_figure = input_file[:-4]+'-GPT3.5-plots.png'
output_file = input_file[:-4]+'-GPT4-SCORES-only.txt'
output_figure = input_file[:-4]+'-GPT4-plots.png'

# 1. Read the file and extract numbers
numbers = []
with open(input_file, 'r') as f:
    for line in f:
        # match = re.search(r";\['(\d+)", line)
        match = re.search(r";\[['\"](\d+(?:\.\d+)?)", line)
        if match:
            numbers.append(float(match.group(1)))

# 2. Save the numbers to a new file
with open(output_file, 'w') as f:
    for num in numbers:
        f.write(str(num) + '\n')

print(f'Scores saved in {output_file}')

# 3. Calculate the average
average = sum(numbers) / len(numbers)
print('len=',len(numbers))
print(f"Average: {average:.2f}")

# 4. Plot a box-plot
# Plot a box-plot
plt.figure(figsize=(10, 5))

# Subplot for boxplot
plt.subplot(1, 2, 1)
plt.boxplot(numbers)
plt.title("Box plot of GPT4 scores")
plt.ylabel("Value")
plt.xticks([])  # This removes the x-ticks for the first subplot

# Subplot for histogram
plt.subplot(1, 2, 2)
# plt.hist(numbers, bins=5, range=(0, 5))#, align='left', alpha=0.7)  
# Specify the bins explicitly to ensure 0 and 5 are included as edges.
# bins = [0, 1, 2, 3, 4, 5]
# plt.hist(numbers, bins=bins, edgecolor='black')  
plt.hist(numbers, bins=5, range=(0, 5), edgecolor='black')
plt.title("Histogram of GPT4 scores")
plt.xlabel("Value")
plt.ylabel("Frequency")
plt.xticks(range(6))  # Ensure we have ticks for all integer values
# plt.xticks(bins)  

plt.tight_layout()
# Save the figure to a file
plt.savefig(output_figure, dpi=300)
plt.show()
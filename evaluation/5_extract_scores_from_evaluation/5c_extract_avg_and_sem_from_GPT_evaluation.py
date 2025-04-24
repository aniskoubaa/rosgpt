import re
import matplotlib.pyplot as plt
import os
import numpy as np

for input_file in ['/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT3.5/Evaluated_JSON_commands_UAV.txt','/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/GPT3.5/Evaluated_JSON_commands_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations//Ground-Robot/GPT3.5/Evaluated_JSON_commands_Ground_Robot_v2.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA-7b/Evaluated_JSON_commands_LLaMA7b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/GPT4/Evaluated_JSON_commands_GPT4_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/GPT4/Evaluated_JSON_commands_GPT4_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT4/Evaluated_JSON_commands_GPT4_UAV.txt','/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Robotic_Arm.txt']:


    print(input_file)

    # 1. Read the file and extract numbers
    numbers = []
    with open(input_file, 'r') as f:
        for line in f:
            # match = re.search(r";\['(\d+)", line)
            match = re.search(r";\[['\"](\d+(?:\.\d+)?)", line)
            if match:
                numbers.append(float(match.group(1)))

    # 2. Save the numbers to a new file
    # with open(output_file, 'w') as f:
    #     for num in numbers:
    #         f.write(str(num) + '\n')

    # print(f'Scores saved in {output_file}')

    # 3. Calculate the average
    average = sum(numbers) / len(numbers)
    sem = np.std(numbers)/np.sqrt(len(numbers))
    margin = sem*1.96 # For 95% confidence interval
    # print('len=',len(numbers))
    # print(f"Average: {average:.2f}")
    output_f = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_SEM_GPT4_score.txt'
    with open(output_f, 'a') as output_file:
                        model = input_file.split('/')[-2]
                        case = input_file.split('/')[-3]
                        # Your print statement will write directly to the file
                        # output_file.write(f"{model},{case},{average:.2f},{sem:.2f}\n")
                        output_file.write(f"{model},{case},{average:.2f} \pm {margin:.2f}\n")


print(f'Results saved in {output_f}')
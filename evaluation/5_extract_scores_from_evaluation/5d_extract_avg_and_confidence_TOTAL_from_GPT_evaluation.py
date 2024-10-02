import re
import matplotlib.pyplot as plt
import os
import numpy as np

numbers = {}

for input_file in ['/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT3.5/Evaluated_JSON_commands_UAV.txt','/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/GPT3.5/Evaluated_JSON_commands_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations//Ground-Robot/GPT3.5/Evaluated_JSON_commands_Ground_Robot_v2.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA-7b/Evaluated_JSON_commands_LLaMA7b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA-7b/Evaluated_JSON_commands_LLaMA-7b_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-70b/Evaluated_JSON_commands_LLaMA2-70b_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/GPT4/Evaluated_JSON_commands_GPT4_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/GPT4/Evaluated_JSON_commands_GPT4_Robotic_Arm.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT4/Evaluated_JSON_commands_GPT4_UAV.txt','/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Ground-Robot/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Ground_Robot.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_UAV.txt', '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Robotic_Arm.txt']:


    print(input_file)

    model = input_file.split('/')[-2]
    case = input_file.split('/')[-3]

    if not(model in numbers):
           numbers[model] = [] 
           
    # 1. Read the file and extract numbers
    
    with open(input_file, 'r') as f:
        
        for line in f:
            # match = re.search(r";\['(\d+)", line)
            match = re.search(r";\[['\"](\d+(?:\.\d+)?)", line)
            if match:
                numbers[model].append(float(match.group(1)))


output_f = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_confidence_GPT4score_total_per_model.txt'

for model in numbers:
    # 3. Calculate the average
    average = sum(numbers[model]) / len(numbers[model])
    sem = np.std(numbers[model])/np.sqrt(len(numbers[model]))
    margin = sem*1.96 # For 95% confidence interval

    with open(output_f, 'a') as output_file:
        output_file.write(f"{model},{average:.2f} \pm {margin:.2f}\n")


print(f'Results saved in {output_f}')
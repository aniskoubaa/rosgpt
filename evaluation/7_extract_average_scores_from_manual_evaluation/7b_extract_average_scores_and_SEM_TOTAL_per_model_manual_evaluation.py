import re
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os


models = ['LLaMA-7b','LLaMA2-7b','LLaMA2-70b','GPT3.5','GPT4']
cases = ['Ground_Robot','UAV','Robotic_Arm']

def find_files(folder_path):
    # List all files in the specified folder
    files = os.listdir(folder_path)

    # Filter files based on the criteria
    selected_files = [file for file in files if '5pct' in file and file.endswith('.txt')]

    return selected_files

for model in models:
    print(model)
    scores = pd.Series()
    for case in cases:    
        print(case)
        case2 = case.replace('_','-')
        # input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/'+case2+'/'+model+'/Evaluated_JSON_commands_'+model+'_'+case+'_5pct_for_manual_eval.txt'
        folder = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/'+case2+'/'+model
        print(folder)
        folder_exists = os.path.exists(folder)
        print(folder_exists)
        if folder_exists:
            selected_files = find_files(folder)
            print(selected_files)
            if len(selected_files)>0:
                input_file = os.path.join(folder,selected_files[0])
                print(input_file)
                # try:
                df = pd.read_csv(input_file, delimiter=';')


                # Extract the 'Human score' column from index 0 to 49
                # scores = df['Human score'][0:50]
                scores = pd.concat([scores, df['Human'][0:50].dropna()], ignore_index=True)
                print(list(scores))

    # Calculate the average
    average = scores.mean()
    sem = scores.std()/np.sqrt(len(scores))
    margin = sem*1.96 # For 95% confidence interval
    print(f"*********{model} / {case}: {average:.2f} +- {margin:.2f}")
    output_file_path = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_SEM_TOTAL_per_model_human_score.txt'
    with open(output_file_path, 'a') as output_file:
        # Your print statement will write directly to the file
        # output_file.write(f"{model},{case},{average:.2f},{sem:.2f}\n")
        output_file.write(f"{model},{average:.2f} \pm {margin:.2f}\n")
                # except:
                #     print('Problem')

            # plt.show()
print(f'Results saved in {output_file_path}')

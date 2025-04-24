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
                output_figure = input_file[:-4]+'-HIST.png'
                try:
                    df = pd.read_csv(input_file, delimiter=';')

                    # Extract the 'Human score' column from index 0 to 49
                    # scores = df['Human score'][0:50]
                    scores = df['Human'][0:50].dropna()
                    print(list(scores))
                    plt.figure(figsize=(10, 5))
                    plt.hist(scores, bins=5, range=(0, 5.01), edgecolor='black')
                    plt.xticks(range(6),size=20)  # Ensure we have ticks for all integer values
                    plt.yticks(size=20)
                    plt.tight_layout()
                    # Save the figure to a file
                    plt.savefig(output_figure, dpi=300)
                except:
                    print('Problem')


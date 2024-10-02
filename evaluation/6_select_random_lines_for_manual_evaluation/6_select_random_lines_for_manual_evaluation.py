import random

def select_random_lines(input_filename, output_filename, percentage=5):
    with open(input_filename, 'r') as infile:
        lines = infile.readlines()

    # Determine how many lines to select
    total_lines = len(lines)
    num_to_select = int(total_lines * (percentage / 100.0))
    
    # Randomly select line indices without replacement
    selected_indices = random.sample(range(total_lines), num_to_select)

    with open(output_filename, 'w') as outfile:
        for idx in selected_indices:
            # Write the selected line along with its index in the original file
            outfile.write(f"{idx + 1}: {lines[idx]}")


input_file = '/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Robotic_Arm.txt'
output_file = input_file[:-4]+'_SELECTED_5pct_for_manual_eval.txt'

if __name__ == "__main__":
    # Example usage
    select_random_lines(input_file, output_file)
    print(f'Output saved in: {output_file}')

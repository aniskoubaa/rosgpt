import openai

# OpenAI API setup (assuming you've already set up the openai library)
OPENAI_API_KEY = '****************************'
openai.api_key = OPENAI_API_KEY


def rephrase_command(command, num_rephrases=4):
    system_prompt = f"Rephrase the following command {num_rephrases} different ways."
    rephrased_cmds = []
    while len(rephrased_cmds) < num_rephrases:
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command},
                    {"role": "assistant", "content": ""},
                ]
            )
            rephrased_cmds.extend(response.choices[0].message['content'].split('\n'))
        except Exception as e:
            print(f"Error occurred while rephrasing: {e}")
            continue

    return rephrased_cmds[:num_rephrases]  # Ensure only the required number of rephrasings is returned

# Reading input commands from the file
# with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_Ground_Robot_base_commands_with_irrelevant.txt", "r") as infile:
# with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_UAV_base_commands_with_irrelevant.txt", "r") as infile:
with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_Robotic_Arm_base_commands_with_irrelevant.txt", "r") as infile:
    commands = infile.readlines()

# Processing each command and writing to the output file
# with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_Groubd_Robot_commands_with_5rephrasings.txt", "a") as outfile:
# with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_UAV_with_5rephrasings.txt", "a") as outfile:
with open("/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/NL_commands_Robotic_Arm_with_5rephrasings.txt", "a") as outfile:
    ind = 1
    for cmd in commands:
        print(ind)
        cmd = cmd.strip()
        rephrased_cmds = rephrase_command(cmd)
        outfile.write(f"{cmd}\n")
        for idx, rep_cmd in enumerate(rephrased_cmds, 1):
            outfile.write(f"{rep_cmd}\n")
        # outfile.write("\n")
        ind+=1

import openai
import os
import csv
import time
import json

# Set your OpenAI API key here
# openai.api_key = '******************' # aammar
# openai.api_key = '******************' # riotu free
# openai.api_key = '******************' # adel.ammar
openai.api_key = '******************' # riotu paid

nb_commands = 41
# nb_rephrasing = 5

# System prompt which instructs the model to generate commands and then rephrase
generate_command_prompt = """
Generate a command for a ground robot, without any other introduction, comment, numbers, nor conclusion.
Examples:
"Move 1 meter forward for two seconds."
"Rotate clockwise by 45 degrees."
"Turn right and move forward for 3 meters."
"Go to the kitchen and stop." 
"""
rephrase_instruction = "Generate 5 different wordings for the following ground robot command, without any other introduction, comment, numbers, nor conclusion."

def issue_commands(prompt, system_prompt=rephrase_instruction):
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
            {"role": "assistant", "content": ""},
        ]
    )
    assistant_reply = response.choices[0].message['content']
    return assistant_reply

with open("NL_commands_Groubd_Robot.txt", "a") as file:
    for i in range(nb_commands):
        print(i)
        # First, generate a base command
        try:
            base_command = issue_commands("", system_prompt=generate_command_prompt)
            rephrased_commands = issue_commands(base_command)

        except Exception as e:
            print(f"An error occurred: {e}")

        # file.write("Rephrased Commands:\n")
        for idx, rep_cmd in enumerate(rephrased_commands.split('\n'), 1):
            print((f"{rep_cmd}\n"))
            file.write(f"{rep_cmd}\n")
        # file.write("\n")

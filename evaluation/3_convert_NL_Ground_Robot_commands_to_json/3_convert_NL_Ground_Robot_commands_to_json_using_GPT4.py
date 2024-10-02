import os
import openai


openai.api_key = '******************'

instruction = """
Consider the following ontology:
{"action": "go_to_goal", "params": {"location": {"type": "str", "value": "Kitchen"}}} 
{"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}} 
{"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}} 
You will be given human language prompts, and you need to return a json conformant to the ontology. Any action not in the ontology must be ignored. If a field's value is undetermined, put a default reasonable value. Return only the json without any introduction, comments, nor conclusion. Here are some examples. 
prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}} 
prompt: "Rotate 60 degree in clockwise direction at 10 degrees per second and make pizza."
returns: {"action": "rotate", "params": {"angular_velocity": 10, "angle": 60, "is_clockwise": true, "unit": "degrees"}} 
"""

def convert_command_to_json(prompt, system_prompt=instruction):

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
            {"role": "assistant", "content": ""},
        ]
    )
    output = response.choices[0].message['content'].split('\n')
    print("Json:", output)
    return output

def process_input_file(input_file_name, output_file_name, start_from=1):
    i = 1
    with open(input_file_name, 'r') as input_file:
        with open(output_file_name, 'w') as output_file:  # 'a' mode to append in case of restarts
            for line in input_file:
                print(i)
                if i >= start_from:
                    line = line.strip()  # remove any trailing or leading whitespaces
                    if line:  # make sure line is not empty
                        try:
                            json_output = convert_command_to_json(line)
                            # Save the result to the output file immediately                           
                        except Exception as e:
                            print(f"Error processing line '{line}': {e}")
                            json_output=f"ERROR: {e}" + '\n'
                        
                        output_file.write(f'{line},{json_output}\n')
                i += 1


# Example of usage:
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_Ground_Robot_commands_with_5rephrasings.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/Converted_JSON_commands_Ground_Robot_GPT4.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

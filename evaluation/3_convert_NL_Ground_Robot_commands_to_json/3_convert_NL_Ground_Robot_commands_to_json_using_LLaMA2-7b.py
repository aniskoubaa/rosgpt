import os
import requests

API_URL = "https://api-inference.huggingface.co/models/meta-llama/Llama-2-7b-chat-hf"
headers = {"Authorization": "***********"}

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

def query(payload):
    response = requests.post(API_URL, headers=headers, json=payload)
    return response.json()

def convert_command_to_json(prompt, system_prompt=instruction):
    payload = {
        # "instruction": instruction,
        "inputs": instruction+'\nNow give the JSON corresponding to this prompt:\n'+prompt+'\nYour answer: ',
        "parameters": {
            "max_new_tokens": 256,
            "top_k": 40,
            "top_p": 0.1,
            "temperature": 0.5,
            "stream": True
        }
    }
    
    response = query(payload)
    # output = response.get('outputs', {}).get('text', '').split('\n')
    # print("Json:", output)
    return response

def process_input_file(input_file_name, output_file_name, start_from=1):
    i = 1
    answer='No JSON'
    with open(input_file_name, 'r') as input_file:
        with open(output_file_name, 'w') as output_file:
            for line in input_file:
                print(i)
                if i >= start_from:
                    line = line.strip()
                    if line:
                        try:
                            json_output = convert_command_to_json(line)
                            print(json_output)
                            answer = json_output[0]['generated_text'].split('\nYour answer: \n\n')[-1].split('}}')[0]+'}}'
                            print('-----------')
                            if answer=='': answer = 'No JSON'
                            print(answer)
                            print('-----------')
                        
                        except Exception as e:
                            print(f"Error processing line '{line}': {e}")
                            json_output=f"ERROR: {e}" + '\n'
                        
                        output_file.write(f'{line},{answer}\n')
                i += 1

# Example of usage:
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_Ground_Robot_commands_with_5rephrasings.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/JSON-converted-commands/LLaMA2-7b/Converted_JSON_commands_Ground_Robot_LLaMA2-7b_run2.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

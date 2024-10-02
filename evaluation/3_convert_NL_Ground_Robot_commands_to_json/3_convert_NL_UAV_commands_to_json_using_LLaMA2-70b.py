import os
import requests

API_URL = "https://api-inference.huggingface.co/models/meta-llama/Llama-2-70b-chat-hf"
headers = {"Authorization": "***********"}

instruction = """
Consider the following ontology:
{"action": "navigate_to_point",    "params": {  "coordinates": {    "type": "tuple",        "value": "(latitude, longitude, altitude)"}}},  {"action": "fly",    "params": {  "speed": 10.5,      "altitude": 100,      "direction": {    "type": "str",        "value": "north"
      },      "duration": 10,      "unit_speed": "m/s",      "unit_altitude": "meters",      "unit_duration": "seconds"
    }
  },  {"action": "hover",    "params": {  "duration": {    "type": "float",        "value": 15.0
      },      "altitude": {    "type": "float",        "value": 50.0
      },      "unit_duration": "seconds",      "unit_altitude": "meters"
    }
  },  {"action": "rotate",    "params": {  "angular_velocity": 0.5,      "angle": 90,      "is_clockwise": true,      "unit": "degrees"
    }
  },  {"action": "land",    "params": {  "location": {    "type": "tuple",        "value": "(latitude, longitude)"
      }
    }
  }
You will be given human language prompts, and you need to return a json conformant to the ontology. Any action not in the ontology must be ignored. If a field's value is undetermined, put a default reasonable value. Return only the json without any introduction, comments, nor conclusion. Here are some examples: 
prompt: "Navigate to coordinates 37.7749° N, 122.4194° W at an altitude of 100 meters."
returns: 
{
  "action": "navigate_to_point",  "params": {"coordinates": {  "type": "tuple",      "value": "(37.7749, -122.4194, 100)"
    }
  }
}
prompt: "Fly north at a speed of 12 m/s for 20 seconds at an altitude of 120 meters."
returns:
{
  "action": "fly",  "params": {"speed": 12.0,    "altitude": 120,    "direction": {  "type": "str",      "value": "north"
    },    "duration": 20,    "unit_speed": "m/s",    "unit_altitude": "meters",    "unit_duration": "seconds"
  }
}
prompt: "Hover for 30 seconds at 70 meters, then dance."
returns: 
{
  "action": "hover",  "params": {"duration": {  "type": "float",      "value": 30.0
    },    "altitude": {  "type": "float",      "value": 70.0
    },    "unit_duration": "seconds",    "unit_altitude": "meters"
  }
}
"""

def query(payload):
    response = requests.post(API_URL, headers=headers, json=payload)
    return response.json()

def convert_command_to_json(prompt, system_prompt=instruction):
    payload = {    # "instruction": instruction,        
        "inputs": instruction+'\nNow give the JSON corresponding to this prompt:\n'+prompt+'\nYour answer: ',        "parameters": {        "max_new_tokens": 256,            "top_k": 40,            "top_p": 0.1,            "temperature": 0.5,            "stream": True
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
                            answer = json_output[0]['generated_text'].split('\nYour answer: \n')[-1].split('\n\n')[0].replace('\n',' ')#+'}}'
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
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_UAV_with_5rephrasings.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/JSON-converted-commands/LLaMA2-70b/Converted_JSON_commands_UAV_LLaMA2-70b_run2.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

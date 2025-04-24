import os
import openai
import time

openai.api_key = '******************'

instruction = """
Consider the following ontology:
{
    "action": "navigate_to_point",
    "params": {
      "coordinates": {
        "type": "tuple",
        "value": "(latitude, longitude, altitude)"
      }
    }
  },
  {
    "action": "fly",
    "params": {
      "speed": 10.5,
      "altitude": 100,
      "direction": {
        "type": "str",
        "value": "north"
      },
      "duration": 10,
      "unit_speed": "m/s",
      "unit_altitude": "meters",
      "unit_duration": "seconds"
    }
  },
  {
    "action": "hover",
    "params": {
      "duration": {
        "type": "float",
        "value": 15.0
      },
      "altitude": {
        "type": "float",
        "value": 50.0
      },
      "unit_duration": "seconds",
      "unit_altitude": "meters"
    }
  },
  {
    "action": "rotate",
    "params": {
      "angular_velocity": 0.5,
      "angle": 90,
      "is_clockwise": true,
      "unit": "degrees"
    }
  },
  {
    "action": "land",
    "params": {
      "location": {
        "type": "tuple",
        "value": "(latitude, longitude)"
      }
    }
  }
You will be given human language prompts, and you need to return a json conformant to the ontology. Any action not in the ontology must be ignored. If a field's value is undetermined, put a default reasonable value. Return only the json without any introduction, comments, nor conclusion. Here are some examples: 
prompt: "Navigate to coordinates 37.7749° N, 122.4194° W at an altitude of 100 meters."
returns: 
{
  "action": "navigate_to_point",
  "params": {
    "coordinates": {
      "type": "tuple",
      "value": "(37.7749, -122.4194, 100)"
    }
  }
}
prompt: "Fly north at a speed of 12 m/s for 20 seconds at an altitude of 120 meters."
returns:
{
  "action": "fly",
  "params": {
    "speed": 12.0,
    "altitude": 120,
    "direction": {
      "type": "str",
      "value": "north"
    },
    "duration": 20,
    "unit_speed": "m/s",
    "unit_altitude": "meters",
    "unit_duration": "seconds"
  }
}
prompt: "Hover for 30 seconds at 70 meters, then dance."
returns: 
{
  "action": "hover",
  "params": {
    "duration": {
      "type": "float",
      "value": 30.0
    },
    "altitude": {
      "type": "float",
      "value": 70.0
    },
    "unit_duration": "seconds",
    "unit_altitude": "meters"
  }
}
"""

def convert_command_to_json(prompt, system_prompt=instruction):

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
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
                            time.sleep(5)
                        
                        output_file.write(f'{line},{json_output}\n')
                i += 1


# Example of usage:
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_UAV_with_5rephrasings.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/Converted_JSON_commands_UAV.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

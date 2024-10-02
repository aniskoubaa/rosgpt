import os
import openai

openai.api_key = '******************'  # GPT4 Nov2023
# openai.api_key = '******************'

instruction = """
Compare the following natural language command to the JSON structure(s) next to it in the same line. Then give a conformity score from 0 to 5, according to the matching between the natural language command and the JSON information and its conformity with the keys of this sample ontology:
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

Commands that don't exist in the sample keys should not be converted to JSON.
If a parameter's value is not specified in the natural language command, any reasonable value in the JSON is accepted.
Format your response as: Score; Justification.
Example: 5; Exactly conform.
"""

def compare_fields(prompt, system_prompt=instruction):

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
                            score_and_justification = compare_fields(line, instruction)
                            print(score_and_justification)
                            # score, justification = score_and_justification.split("; ", 1)
                            # Save the result to the output file immediately                           
                        except Exception as e:
                            print(f"Error processing line '{line}': {e}")
                            score_and_justification=f"ERROR: {e}" + '\n'
                        
                        output_file.write(f'{line};{score_and_justification}\n')
                i += 1


# Example of usage:
# model=''
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/JSON-converted-commands/GPT4/Converted_JSON_commands_UAV_GPT4.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/UAV/GPT4/Evaluated_JSON_commands_GPT4_UAV.txt"

process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

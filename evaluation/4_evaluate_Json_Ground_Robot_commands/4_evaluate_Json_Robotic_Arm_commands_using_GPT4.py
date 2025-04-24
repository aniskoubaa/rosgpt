import os
import openai

openai.api_key = '******************'  # GPT4 Nov2023
# openai.api_key = '******************'
# openai.api_key = '******************'

instruction = """
Compare the following natural language command to the JSON structure(s) next to it in the same line. Then give a conformity score from 0 to 5, according to the matching between the natural language command and the JSON information and its conformity with the keys of this sample ontology:
{
    "action": "move_joint",
    "params": {
      "joint_name": {
        "type": "str",
        "value": "elbow"
      },
      "angle": {
        "type": "float",
        "value": 45.0
      },
      "direction": {
        "type": "str",
        "value": "clockwise"
      },
      "speed": {
        "type": "float",
        "value": 0.5
      },
      "unit": "degrees",
      "unit_speed": "degrees/s"
    }
  },
  {
    "action": "extend_arm",
    "params": {
      "extension_length": {
        "type": "float",
        "value": 10.0
      },
      "speed": {
        "type": "float",
        "value": 1.0
      },
      "unit_length": "centimeters",
      "unit_speed": "cm/s"
    }
  },
  {
    "action": "grip_object",
    "params": {
      "force": {
        "type": "float",
        "value": 5.0
      },
      "duration": {
        "type": "float",
        "value": 2.0
      },
      "unit_force": "newtons",
      "unit_duration": "seconds"
    }
  },
  {
    "action": "release_grip",
    "params": {}
  },
  {
    "action": "set_orientation",
    "params": {
      "orientation": {
        "type": "str",
        "value": "upright"
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
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/JSON-converted-commands/LLaMA2-7b/Converted_JSON_commands_Robotic_Arm_LLaMA2-7b.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/Robotic-Arm/LLaMA2-7b/Evaluated_JSON_commands_LLaMA2-7b_Robotic_Arm.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

import os
import openai


openai.api_key = '******************'

instruction = """
Consider the following ontology for robotic arm commands:
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

You will be given human language prompts, and you need to return a json conformant to the ontology. Any action not in the ontology must be ignored. If a field's value is undetermined, put a default reasonable value. Return only the json without any introduction, comments, nor conclusion. Here are some examples: 
prompt: "Rotate the elbow joint 90 degrees counterclockwise at a speed of 1 degree per second."
returns: 
{
  "action": "move_joint",
  "params": {
    "joint_name": {
      "type": "str",
      "value": "elbow"
    },
    "angle": {
      "type": "float",
      "value": 90.0
    },
    "direction": {
      "type": "str",
      "value": "counterclockwise"
    },
    "speed": {
      "type": "float",
      "value": 1.0
    },
    "unit": "degrees",
    "unit_speed": "degrees/s"
  }
}

prompt: "Extend the arm 20 centimeters at a speed of 2 centimeters per second."
returns:
{
  "action": "extend_arm",
  "params": {
    "extension_length": {
      "type": "float",
      "value": 20.0
    },
    "speed": {
      "type": "float",
      "value": 2.0
    },
    "unit_length": "centimeters",
    "unit_speed": "cm/s"
  }
}

prompt: "Grip the object with a force of 10 newtons for 3 seconds."
returns: 
{
  "action": "grip_object",
  "params": {
    "force": {
      "type": "float",
      "value": 10.0
    },
    "duration": {
      "type": "float",
      "value": 3.0
    },
    "unit_force": "newtons",
    "unit_duration": "seconds"
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
                        
                        output_file.write(f'{line},{json_output}\n')
                i += 1


# Example of usage:
input_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_Robotic_Arm_with_5rephrasings.txt"
output_file = "/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/JSON-converted-commands/Converted_JSON_commands_Robotic_Arm.txt"
process_input_file(input_file, output_file, start_from=1)
print('Output saved in: ',output_file)

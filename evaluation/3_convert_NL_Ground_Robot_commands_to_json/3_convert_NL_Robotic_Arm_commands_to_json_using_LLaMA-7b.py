import os
import sys
import json
import fire
import torch
import transformers
from peft import PeftModel
from transformers import GenerationConfig, LlamaForCausalLM, LlamaTokenizer

if torch.cuda.is_available():
    device = "cuda"
else:
    device = "cpu"

try:
    if torch.backends.mps.is_available():
        device = "mps"
except:  # noqa: E722
    pass


def main(
    load_8bit: bool = False,
    base_model: str = "yahma/llama-7b-hf",
    lora_weights: str = "",
    share_gradio: bool = False,
    input_file: str="/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/final-NL-commands/NL_commands_UAV_with_5rephrasings.txt",
    output_file: str="/media/akoubaa/new_ssd/Adel/LLM-Adapters/ROSGPT/Converted_JSON_commands_Robotic_Arm.txt"

):
    assert (
        base_model
    ), "Please specify a --base_model, e.g. --base_model='decapoda-research/llama-7b-hf'"

    # assert (
    #     json_file
    # ), "Please specify a --json_file test input file"

    tokenizer = LlamaTokenizer.from_pretrained(base_model)
    if device == "cuda":
        model = LlamaForCausalLM.from_pretrained(
            base_model,
            load_in_8bit=load_8bit,
            torch_dtype=torch.float16,
            device_map="auto",
            trust_remote_code=True,
        )
        model = PeftModel.from_pretrained(
            model,
            lora_weights,
            torch_dtype=torch.float16,
        )
    elif device == "mps":
        model = LlamaForCausalLM.from_pretrained(
            base_model,
            device_map={"": device},
            torch_dtype=torch.float16,
        )
        model = PeftModel.from_pretrained(
            model,
            lora_weights,
            device_map={"": device},
            torch_dtype=torch.float16,
        )
    else:
        model = LlamaForCausalLM.from_pretrained(
            base_model, device_map={"": device}, low_cpu_mem_usage=True
        )
        model = PeftModel.from_pretrained(
            model,
            lora_weights,
            device_map={"": device},
        )

    # unwind broken decapoda-research config
    model.config.pad_token_id = tokenizer.pad_token_id = 0  # unk
    model.config.bos_token_id = 1
    model.config.eos_token_id = 2

    if not load_8bit:
        model.half()  # seems to fix bugs for some users.

    model.eval()
    if torch.__version__ >= "2" and sys.platform != "win32":
        model = torch.compile(model)

    def evaluate(
        instruction,
        input=None,
        temperature=0.1,
        top_p=0.75,
        top_k=40,
        num_beams=4,
        max_new_tokens=128,
        **kwargs,
    ):
        prompt = generate_prompt(instruction, input)
        inputs = tokenizer(prompt, return_tensors="pt")
        input_ids = inputs["input_ids"].to(device)
        generation_config = GenerationConfig(
            temperature=temperature,
            top_p=top_p,
            top_k=top_k,
            num_beams=num_beams,
            **kwargs,
        )
        with torch.no_grad():
            generation_output = model.generate(
                input_ids=input_ids,
                generation_config=generation_config,
                return_dict_in_generate=True,
                output_scores=True,
                max_new_tokens=max_new_tokens,
            )
        s = generation_output.sequences[0]
        output = tokenizer.decode(s)
        return output.split("### Response:")[1].strip().split('\n')


    GR_instruction = """
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

    # json_file = ''
    import json

    # with open(json_file, 'r') as f:
    #     data = json.load(f)  # Load the JSON data into a Python object

    # with open(output_file, 'w') as f:
    #     i = 0
    #     for item in data:
    #         if 'input' in item:
    #             i += 1
    #             print(i)
    #             instruction = GR_instruction

    #             input_field = item['input']
    #             output_field = item['output']
    #             try:
    #                 response = evaluate(instruction, input=input_field)

    #                 f.write(
    #                     str(i) + '\n' +
    #                     'Case' + '\n' +
    #                     input_field + '\n' +
    #                     'Decision (predicted)' + '\n' +
    #                     response + '\n' +
    #                     'Decision (GT)' + '\n' +
    #                     output_field + '\n\n'
    #                 )
    #             except:
    #                 print('Exception!')

    start_from = 1
    i = 1
    with open(input_file, 'r') as input_file:
        with open(output_file, 'w') as output_file:  # 'a' mode to append in case of restarts
            for line in input_file:
                print(i)
                if i >= start_from:
                    line = line.strip()  # remove any trailing or leading whitespaces
                    if line:  # make sure line is not empty
                        try:
                            json_output = evaluate(GR_instruction, input=line)
                            print(json_output)
                            # Save the result to the output file immediately                           
                        except Exception as e:
                            print(f"Error processing line '{line}': {e}")
                            json_output=f"ERROR: {e}" + '\n'
                        
                        output_file.write(f'{line},{json_output}\n')
                i += 1


    # print("Instruction:", instruction)
    # print("Response:", evaluate(instruction))
    # print()


def generate_prompt(instruction, input=None):
    if input:
        return f"""Below is an instruction that describes a task, paired with an input that provides further context. Write a response that appropriately completes the request. 

### Instruction:
{instruction}

### Input:
{input}

### Response:
""" # noqa: E501
    else:
        return f"""Below is an instruction that describes a task. Write a response that appropriately completes the request.  

### Instruction:
{instruction}

### Response:
"""  # noqa: E501


if __name__ == "__main__":
    fire.Fire(main)

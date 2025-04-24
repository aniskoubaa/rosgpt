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
    input_file: str="/media/akoubaa/new_ssd/Adel/LLM-Adapters/ROSGPT/NL_commands_Ground_Robot_commands_with_5rephrasings.txt",
    output_file: str="/media/akoubaa/new_ssd/Adel/LLM-Adapters/ROSGPT/Converted_JSON_commands_Ground_Robot.txt"

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
        return output.split("### Response:")[1].strip()


    GR_instruction = """
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

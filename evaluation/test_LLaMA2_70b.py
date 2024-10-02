import requests

API_URL = "https://api-inference.huggingface.co/models/meta-llama/Llama-2-70b-chat-hf"
headers = {"Authorization": "***********"}

def query(payload):
  response = requests.post(API_URL, headers=headers, json=payload)
  return response.json()
  
data = query(
    {
        "inputs": "write me full email asking for  3 days vacation from my manager",
        "parameters": {"max_new_tokens": 256 , "top_k" : 40, "top_p" : 0.1 , "temperature" : 0.5 , "stream" : True }
    }
)

print(data)
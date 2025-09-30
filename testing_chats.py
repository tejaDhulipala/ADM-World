from dotenv import load_dotenv
import os
from openai import OpenAI
from jsbgym.OpenRouterAgent import OpenRouterAgent

load_dotenv()
api_key = os.getenv("OPENROUTER_API_KEY")

if not api_key:
    raise ValueError("OPENROUTER_API_KEY not found in environment variables")

model_name = OpenRouterAgent.MISTRAL

client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=api_key,
    default_headers={
        "HTTP-Referer": os.getenv("YOUR_SITE_URL", "http://localhost:5000"),
        "X-Title": os.getenv("YOUR_SITE_NAME", "Model Comparison Demo")
    }
)

conversation = []
while "quit" not in (prompt := input(f"Ask {model_name} a question (type 'quit' to exit): ").lower()):
    messages = [
        {
            "role": "system",
            "content": "You are a helpful assistant.",
            "cache_control": {"type": "ephemeral"}  # <-- this marks it for caching
        }
    ]
    conversation.append({"role": "user", "content": prompt, "cache_control": {"type": "ephemeral"}})
    messages += conversation
    response = client.chat.completions.create(
        model=model_name,
        messages=messages,
    )
    conversation.append(
        {"role": "assistant", "content": response.choices[0].message.content.strip(), "cache_control": {"type": "ephemeral"}}
    )
    print(response.choices[0].message.content.strip())

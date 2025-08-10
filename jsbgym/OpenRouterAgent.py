from dotenv import load_dotenv
import yaml
from openai import OpenAI
import os
from abc import ABC, abstractmethod

class ADMAgent(ABC):
    def __init__(self):
        """
        Base class for an Autonomous Decision Making Agent (ADM).
        This class should be extended to implement specific decision-making algorithms.
        """
        pass

    @abstractmethod
    def get_decision(self, system_prompt: dict, cur_prompt: dict) -> dict:
        """
        Generate a decision based on the provided prompts.
        """
        pass

class DemoADMAlgorithm(ADMAgent):
    def __init__(self):
        pass

    def get_decision(self, system_prompt: dict, cur_prompt: dict) -> dict:
        return {
        {
            "control interface": {
                "heading": 330,
                "altitude": 3000,
            },
            "manual": {
            }
        }
    }   

class OpenRouterAgent(ADMAgent):
    models = {
    "OpenAI GPT-3.5": "openai/gpt-3.5-turbo",
    "OpenAI GPT-4": "openai/gpt-4",
    "Anthropic Claude": "anthropic/claude-instant-v1",
    "Google PaLM 2": "google/palm-2-chat-bison", 
    "Mistral": "mistralai/mistral-7b-instruct-v0.2"
    }

    def __init__(self, model_name):
        """
        Initialize the OpenRouterAgent with a specific model name.
        """  
        self.model_name = model_name
        self.client = None

        # Load environment variables from .env file
        load_dotenv()
    
    # Initialize the OpenRouter client
    def get_client(self):
        """
        Initialize and return the OpenRouter client
        """
        api_key = os.getenv("OPENROUTER_API_KEY")

        if not api_key:
            raise ValueError("OPENROUTER_API_KEY not found in environment variables")

        client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=api_key,
            default_headers={
                "HTTP-Referer": os.getenv("YOUR_SITE_URL", "http://localhost:5000"),
                "X-Title": os.getenv("YOUR_SITE_NAME", "Model Comparison Demo")
            }
        )

        self.client = client
        return client

    def get_decision(self, system_prompt: dict, cur_prompt: dict):
        """
        Generate a response using the specified model

        Args:
            model (str): OpenRouter model identifier
            prompt (str): Text prompt to send to the model

        Returns:
            str: The model's response
        """
        client = self.client
        if not client:
            client = self.get_client()

        response = client.chat.completions.create(
            model=self.models[self.model_name],
            messages=[
                {"role": "system", "content": yaml.dump(system_prompt)},
                {"role": "user", "content": yaml.dump(cur_prompt)}
            ]
        )
        print("Response from Model:", response.choices[0].message.content)
        model_response = response.choices[0].message.content
        model_response = model_response.strip()
        model_response = model_response.split("<YAML>")[1].strip()  # Extract the YAML part
        model_response = model_response.split("</YAML>")[0].strip()
        print("Parsed YAML response:", model_response)
        return yaml.safe_load(model_response)
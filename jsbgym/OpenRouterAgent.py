from dotenv import load_dotenv
import json
from openai import OpenAI
import os
from abc import ABC, abstractmethod
import re

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

class OpenRouterAgent(ADMAgent):
    GEMINI_FLASH_2_5 = "google/gemini-2.5-flash"
    MISTRAL = "mistralai/mistral-7b-instruct:free"
    CLAUDE_SONNET_4 = "anthropic/claude-sonnet-4"
    GPT_5_MINI = "openai/gpt-5-mini"
    LLAMA_3_2_1B = "meta-llama/llama-3.2-1b-instruct"

    MAX_CALLS = 5


    def __init__(self, model):
        """
        Initialize the OpenRouterAgent with a specific model name.
        """  
        self.model_name = model
        self.client = None
        self.num_calls = 0
        self.conversation = []

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
        Generate a response using the specified model.

        Args:
            system_prompt (dict): The system prompt as a dictionary.
            cur_prompt (dict): The current prompt as a dictionary.

        Returns:
            dict: The model's parsed JSON response.
        """
        self.num_calls += 1
        client = self.client if self.client else self.get_client()

        # Convert dicts to string prompt
        system_prompt_str = OpenRouterAgent.format_system_prompt(system_prompt)
        cur_prompt_str = json.dumps(cur_prompt, indent=2, ensure_ascii=False)

        if self.num_calls == 1:
            print("=" * 20 + " System Prompt " + "=" * 20)
            print(system_prompt_str)
        print("=" * 20 + " Current Prompt " + "=" * 20)
        print(cur_prompt_str)

        messages = [
                {"role": "system", "content": system_prompt_str, "cache_control": {"type": "ephemeral"}},
            ]
        self.conversation.append({"role": "user", "content": cur_prompt_str, "cache_control": {"type": "ephemeral"}})
        messages += self.conversation[-2:]

        response = client.chat.completions.create(
            model=self.model_name,
            messages=messages
        )

        model_response = response.choices[0].message.content.strip()
        print("=" * 20 + " Model Response " + "=" * 20)
        print(model_response)
        print("=" * 20 + " End Model Response " + "=" * 20)
        model_response = model_response[model_response.index("{"):model_response.rindex("}") + 1]  # Extract JSON part
        model_response = OpenRouterAgent.remove_extraneuous(model_response)  # Remove comments

        dict_response = json.loads(model_response)
        if self.num_calls > self.MAX_CALLS:
            print(f"Warning: Maximum number of calls ({self.MAX_CALLS}) exceeded. Returning last response.")
            return False
        self.conversation.append({"role": "assistant", "content": json.dumps(dict_response), "cache_control": {"type": "ephemeral"}})
        return dict_response

    @staticmethod
    def format_system_prompt(system_prompt: dict) -> str:
        """
        Format the system prompt into a string for the model.
        
        Args:
            system_prompt (dict): The system prompt as a dictionary.
        
        Returns:
            str: Formatted system prompt string.
        """

        commands_dict = "\n".join([f""""{i}": {system_prompt["control system"]["commands"][i]}""" for i in system_prompt["control system"]["commands"]])
        manual_commands_dict = "\n".join([f""""{i}": {system_prompt["manual commands"]["commands"][i]}""" for i in system_prompt["manual commands"]["commands"]])
        return f"""{system_prompt["role"]} {system_prompt["control system"]["description"]} The control interface commands are:
        {commands_dict}
        {system_prompt["manual commands"]["description"]} The manual commands are:
        {manual_commands_dict}
        {system_prompt["output format"]}
        """

    @staticmethod
    def test_get_decision(str_input: str):
        """
        Test parsing of a model's JSON response.
        
        Args:
            str_input (str): The model's raw response string containing JSON between <JSON> tags.
        
        Returns:
            dict: Parsed JSON content.
        """
        model_response = str_input.strip()
        model_response = model_response[model_response.index("{"):model_response.rindex("}") + 1]  # Extract JSON part
        model_response = OpenRouterAgent.remove_extraneuous(model_response)  # Remove comments

        dict_response = json.loads(model_response)

        return dict_response
    
    @staticmethod
    def remove_extraneuous(str_input: str) -> str:
        while str_input.find("//") != -1:
            start = str_input.find("//")
            end = str_input.find("\n", start)   
            str_input = str_input[:start] + str_input[end:]
        
        cleaned_response = re.sub(r'\b0+(\d+)(\.\d+)?\b', r'\1\2', str_input)
        return cleaned_response
import json
import sys
from typing import List, Dict, Any, Optional
from ollama import Client  # Assumes `ollama` Python package is installed

from tiago.lib.database.database import Database
from tiago.lib.map.map import Map


class ShopAssistant:
    def __init__(self, model: str, customer_id: str, window_size=6):
        self.customer_id = customer_id
        self.llm = Client()
        self.model = model
        self.chat_history: List[Dict[str, str]] = []
        self.window_size = window_size
        self.customer_information = {}

    def add_message(self, content: str):
        self.chat_history.append({'role': 'user', 'content': content})

    def set_customer_information(self, customer_information: Dict[str, Any]):
        """
        Set the customer information for the assistant.

        Args:
            customer_information (Dict[str, Any]): The structured customer information to set.
        """
        self.customer_information = customer_information

    def answer(self, option: Optional[str] = None) -> str:
        system_instruction = {
            "role": "system",
            "content": (
                "You're a sports shop assistant. Be concise and clear. "
                "You do not do physical actions - like walking - you just provide information.\n"
                "If the user is asking for information about a product or staff that you don't have, just tell him you "
                "don't know and try to assist him otherwise."
                f"This is what you know about the customer from previous conversations with him: "
                f"{self.customer_information}\n" if self.customer_information else ""
                                                                                   "Try to not write more than 3 sentences.\n"
                                                                                   f"{option}\n" if option else ""
            )
        }
        response = self.llm.chat(model=self.model, messages=[system_instruction, *self.chat_history])
        assistant_reply = response['message']['content']
        self.chat_history.append({'role': 'assistant', 'content': assistant_reply})
        return assistant_reply

    def answer_with_results(self, results: Dict, option: Optional[str] = None) -> str:
        """
        Show the results of the query to the user.
        :param results: The results of the query to show to the user.
        Results should be a dictionary with keys 'products' and/or 'staff', each containing a list of relevant items.

        :param option: Optional additional instructions to the assistant.
        """
        system_instruction = {
            "role": "system",
            "content": (
                "You are a sports shop assistant. Your task is to extract structured information from user messages.\n"
                "You do not do physical actions - like walking - you just provide information.\n"
                "Try to help the user with the following information:\n"
                f"{json.dumps(results, indent=2)}\n"
                "If the data is not relevant, do not include it in the answer and try to help the customer in other ways.\n"
                "Try to not write more than 3 sentences.\n"
                f"{option}\n" if option else ""
            )
        }

        # Printing in stderr
        response = self.llm.chat(model=self.model, messages=[system_instruction, *self.chat_history])
        assistant_reply = response['message']['content']
        self.chat_history.append({'role': 'assistant', 'content': assistant_reply})
        return assistant_reply

    def extract_knowledge(self) -> Dict[str, any]:
        """
        Extract structured knowledge about the customer and conversation history.

        This method analyzes the chat history to identify:
        1. Customer personal information (name, preferences, needs)
        2. Key events in the conversation
        3. Products or services the customer showed interest in
        4. Any issues or complaints mentioned

        Returns:
            Dict[str, any]: JSON formatted knowledge extracted from the conversation
        """
        system_instruction_content = (
            "You're analyzing a conversation between a sports shop assistant and a customer. "
            "Extract structured information from the conversation history only about the customer ( user ) "
            "information, not about the assistant!."
            "Create a JSON with the following fields (all are optional):\n"
            "customer_info: {\n"
            "  name: Customer's name if mentioned,\n"
            "  age_category: Customer's age category if mentioned (e.g., child, adult),\n"
            "  gender: male, female or other,\n"
            "  preferences: Any product preferences mentioned (brands, styles, sports),\n"
            "  size_info: Any sizing information mentioned (shoe size, clothing size),\n"
            "  budget: Any budget constraints mentioned\n"
            "},\n"
            "products_of_interest: List of specific products the customer asked about or showed interest in,\n"
            "purchase_intent: Assessment of how likely the customer is to make a purchase (high/medium/low),\n"
            "follow_up_needed: Whether the customer needs additional assistance or information,\n"
            "conversation_summary: Brief summary of the key points of the conversation\n"
            "}\n"
            f"This is what you know about the customer from previous conversations with him, so modify it with the new one: "
            f"{self.customer_information}\n" if self.customer_information else ""
                                                                               "Extract only information that was explicitly shared. Don't make assumptions about data not provided.\n"
                                                                               "All the fields should be strings or list of strings, except follow_up_needed, which must be a boolean.\n"
                                                                               "Answer only with the JSON, no extra text!\n"
        )

        system_instruction = {
            "role": "system",
            "content": system_instruction_content
        }

        # Including the entire chat history for analysis
        result = self.llm.chat(
            model=self.model,
            messages=[system_instruction, {'role': 'user', 'content': json.dumps(self.chat_history, indent=2)}]
        )

        try:
            parsed = json.loads(result['message']['content'])
            return parsed
        except json.JSONDecodeError:
            pass

    def _detect_query(self, system_instruction_content: str) -> Dict[str, any]:
        """
        Private helper method to extract structured queries from user intent.

        Args:
            system_instruction_content (str): The system instruction content for the LLM.

        Returns:
            Dict[str, any]: The parsed JSON response or an empty dictionary if parsing fails.
        """
        system_instruction = {
            "role": "system",
            "content": system_instruction_content
        }

        # Only taking the user messages of chat history of window size
        user_messages = [message['content']
                         for message in self.chat_history if message['role'] == 'user'][-self.window_size:]

        result = self.llm.chat(
            model=self.model,
            messages=[system_instruction, {'role': 'user', 'content': '\n'.join(user_messages)}]
        )

        try:
            parsed = json.loads(result['message']['content'])
            return parsed
        except json.JSONDecodeError:
            return {}

    def get_conversation_direction(self) -> str:
        """
        Detect the direction of the conversation based on user input.

        Returns:
            str: The detected direction of the conversation. Possible values are:
            - product_info: The customer clearly asked for product information.
            - staff_info: The customer clearly asked for staff information.
            - walk_to_product: The customer clearly asked to be walked to a product.
            - walk_to_staff: The customer clearly asked to be walked to a staff member.
            - finished: It's clear the conversation is ended.
            - If nothing is relevant, returns an empty string.
        """
        system_instruction_content = (
            "You're a sports shop assistant. Be concise and clear. "
            "Answer with a JSON with only one field:\n"
            "conversation_direction ( str ),\n"
            "Possible values are:\n"
            "product_info ( the customer clearly asked for product information),\n"
            "staff_info ( the customer clearly asked for staff information),\n"
            "walk_to_product ( the customer clearly asked to be walked to a product. Not if he asks information about a product,he must clearly ask to be walked there!),\n"
            "walk_to_staff ( the customer clearly asked to be walked to a staff member. Not if he asks information about a staff member, he must clearly ask to be walked there! ),\n"
            "finished ( it's clear the conversation is ended ),\n"
            "If nothing is relevant, return an empty JSON object.\n"
            "Answer only with the JSON, no extra text!\n"
            "It's really important that you answer only if the user clearly asked for it, do not assume anything.\n"
        )

        conversation_query = self._detect_query(system_instruction_content)
        try:
            if conversation_query and "conversation_direction" in conversation_query:
                direction = conversation_query["conversation_direction"].rstrip(" ").lstrip(" ")
                return direction
        except TypeError | ValueError | KeyError | AttributeError:
            pass
        return ""

    def detect_products_query(self) -> Dict[str, any]:
        """
        Ask the LLM to extract a structured query for products from user intent.

        Returns:
            Dict[str, any]: The parsed product query or an empty dictionary if parsing fails.
        """
        system_instruction_content = (
            "You're a sports shop assistant. Be concise and clear. "
            "If the user messages indicate that he needs some products from the database, create a JSON query with the following fields ( all are optional ):\n"
            "product_category ( like shoes, balls, etc. ),\n"
            "product_name ( like Air Max, etc. ),\n"
            "product_brand ( like Nike, Adidas, etc. ),\n"
            "product_sport_category ( like football, basketball, etc. ),\n"
            "product_description ( like red shoes, etc. ),\n"
            "price_range ( like 50-100, etc. ),\n"
            "product_location_needed ( true or false),\n"
            "The fields must represent the user's intent. For example, if the user is looking for a specific product, "
            "include its name and brand.\n"
            "Do not include information that the customer did not ask for explicitly.\n"
            "If nothing is relevant, return an empty JSON object.\n"
            "All of the fields must be strings or list of strings, except product_location_needed, which must be a boolean.\n"
            "Answer only with the JSON, no extra text!\n"
        )

        products_query = self._detect_query(system_instruction_content)
        try:
            has_product_query = any(value for value in products_query.values())
        except (TypeError, ValueError, AttributeError):
            has_product_query = False
        if has_product_query:
            return products_query
        return {}

    def detect_staff_query(self) -> Dict[str, any]:
        """
        Ask the LLM to extract a structured query for staff information from user intent.

        Returns:
            Dict[str, any]: The parsed staff query or an empty dictionary if parsing fails.
        """
        system_instruction_content = (
            "You're a sports shop assistant. Be concise and clear. "
            "If the user messages indicate that he needs information about staff members, create a JSON query with the following fields ( all are optional ):\n"
            "staff_name ( like John Smith, etc. ),\n"
            "staff_role ( his rank in the store, like manager, sales associate, etc. ),\n"
            "staff_category ( what his specialties are, like footwear expert, tennis equipment specialist, etc. ),\n"
            "staff_location_needed ( true or false ),\n"
            "If nothing is relevant, return an empty JSON object.\n"
            "The fields must represent the user's intent. For example, if the user is looking for a specific staff member, "
            "include their name and role.\n"
            "Do not include information that the customer did not ask for explicitly.\n"
            "All of the fields must be strings or list of strings, except staff_location_needed, which must be a boolean.\n"
            "Answer only with the JSON, no extra text!\n"
        )

        staff_query = self._detect_query(system_instruction_content)
        try:
            has_staff_query = any(value for value in staff_query.values())
        except (TypeError, ValueError, AttributeError):
            has_staff_query = False
        if has_staff_query:
            return staff_query

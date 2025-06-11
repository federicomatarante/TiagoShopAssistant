import json
import sys
from typing import List, Dict, Any, Optional
from ollama import Client  # Assumes `ollama` Python package is installed

from data.database.database import Database
from data.interface import DataInterface
from data.map import Map


class ShopAssistant:
    def __init__(self, model: str, customer_id: str, data_interface: DataInterface, window_size=6, debug=False):
        self.customer_id = customer_id
        self.llm = Client()
        self.model = model
        self.data_interface = data_interface
        self.customer_information = data_interface.get_customer_information(customer_id)
        self.chat_history: List[Dict[str, str]] = []
        self.window_size = window_size
        self.debug = debug

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
        if self.debug:
            print("[DEBUG] Summary reply: ", result['message']['content'], file=sys.stderr)
            print("[DEBUG] Previous customer information: ", self.customer_information, file=sys.stderr)
        try:
            parsed = json.loads(result['message']['content'])
            self.data_interface.update_customer_information(self.customer_id, parsed)
            self.customer_information = self.data_interface.get_customer_information()
        except json.JSONDecodeError:
            pass
        if self.debug:
            print("[DEBUG] Customer information updated: ", self.customer_information, file=sys.stderr)

    def answer_with_results(self, results, option: Optional[str] = None) -> str:
        """
        Show the results of the query to the user.
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

    def detect_conversation_interaction(self) -> dict[str, Any]:
        """
        Detect the direction of the conversation based on user input.

        Returns:
            str: The detected direction of the conversation.
        """
        system_instruction_content = (
            "You're a sports shop assistant. Be concise and clear. "
            "Answer with a JSON with only one field:\n"
            "conversation_direction ( str ),\n"
            "Possible values are:\n"
            "product_info ( the customer clearly asked for product information),\n"
            "staff_info ( the customer clearly asked for staff information),\n"
            "product_location ( the customer clearly asked for product location),\n"
            "staff_location ( the customer clearly asked for staff location),\n"
            "walk_to_product ( the customer clearly asked to be walked to a product. Not if he asks information about a product,he must clearly ask to be walked there!),\n"
            "walk_to_staff ( the customer clearly asked to be walked to a staff member. Not if he asks information about a staff member, he must clearly ask to be walked there! ),\n"
            "finished ( it's clear the conversation is ended ),\n"
            "If nothing is relevant, return an empty JSON object.\n"
            "Answer only with the JSON, no extra text!\n"
            "It's really important that you answer only if the user clearly asked for it, do not assume anything.\n"
        )

        return self._detect_query(system_instruction_content)

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

        return self._detect_query(system_instruction_content)

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

        return self._detect_query(system_instruction_content)

    def _extract_results_from_query(self, products_query, staff_query) -> dict[str, Any]:
        """
        Extract relevant results from the query.

        Args:
            query (Dict[str, any]): The structured query to extract results from.

        Returns:
            List[Dict[str, Any
            ]]: A list of dictionaries containing the extracted results.
        """
        # Checking if there are products or staff queries
        try:
            has_product_query = any(value for value in products_query.values())
        except (TypeError, ValueError, AttributeError):
            has_product_query = False
        try:
            has_staff_query = any(value for value in staff_query.values())
        except (TypeError, ValueError, AttributeError):
            has_staff_query = False

        # Dividing the query into product and staff queries
        query = {}
        if has_product_query:
            query["product_query"] = products_query
        if has_staff_query:
            query["staff_query"] = staff_query

        # Extracting results based on the query
        results = {}
        if "product_query" in query:
            results['products'] = self.data_interface.extract_relevant_products(query["product_query"])

        if "staff_query" in query:
            results['staff'] = self.data_interface.extract_relevant_staff(query["staff_query"])

        return results

    def respond(self, user_input: str) -> dict[str, str | None | Any]:
        self.chat_history.append({'role': 'user', 'content': user_input})
        conversation_query = self.detect_conversation_interaction()

        direction = None
        try:
            if conversation_query and "conversation_direction" in conversation_query:
                direction = conversation_query["conversation_direction"].rstrip(" ").lstrip(" ")
        except TypeError | ValueError | KeyError | AttributeError:
            # If no relevant query is detected, just answer the user
            pass
        if self.debug:
            print("[DEBUG] Conversation direction extracted: ", direction, file=sys.stderr)
        if direction == "finished":
            assistant_reply = self.answer(option="Say goodbye to the customer")
            return {
                'assistant_reply': assistant_reply,
                'conversation_direction': direction
            }
        products_query = self.detect_products_query()
        staff_query = self.detect_staff_query()
        if self.debug:
            print("[DEBUG] Products query extracted: ", products_query, file=sys.stderr)
            print("[DEBUG] Staff query extracted: ", staff_query, file=sys.stderr)

        results = self._extract_results_from_query(products_query, staff_query)
        if self.debug:
            print("[DEBUG] Results extracted: ", results, file=sys.stderr)
        has_products = "products" in results
        has_staff = "staff" in results
        if direction == "walk_to_product" and has_products:
            product = results["products"][0] if len(results["products"]) > 0 else None
            if product:
                results = {'products': [product, ]}
                assistant_reply = self.answer_with_results(results,
                                                           option="Say the customer you're going to walk him to the product")
                return {
                    'assistant_reply': assistant_reply,
                    'conversation_direction': direction,
                    'product': product

                }
            else:
                assistant_reply = self.answer(results,
                                              option="Say the customer you don't have the product he asked for")
                return {
                    'assistant_reply': assistant_reply,
                    'conversation_direction': direction,
                }
        if direction == "walk_to_staff" and has_staff:
            staff = results["staff"][0] if len(results["staff"]) > 0 else None

            if staff:
                results = {'staff': [staff, ]}
                assistant_reply = self.answer_with_results(results,
                                                           option="Say the customer you're going to walk him to the staff")
                return {
                    'assistant_reply': assistant_reply,
                    'conversation_direction': direction,
                    'staff': staff
                }
            else:
                assistant_reply = self.answer(results,
                                              option="Say the customer you don't have the staff he asked for")
                return {
                    'assistant_reply': assistant_reply,
                    'conversation_direction': direction,
                }
        if has_products or has_staff:
            assistant_reply = self.answer_with_results(results)
        else:
            assistant_reply = self.answer()
        return {
            'assistant_reply': assistant_reply,
        }


if __name__ == "__main__":
    database = Database(db_path='test.sqlite')
    map = Map.from_image(image_path='res/map.png')
    interface = DataInterface(
        database=database,
        map=map,
        similarity_threshold=0.7
    )
    choice = input("What to do with the data? \n"
                   "Y) Load\n"
                   "N) Nothing\n")
    if choice.lower() == 'y':
        database.delete()
        with open("../../res/shop_inventory.json", "r", encoding="utf-8") as f:
            data = json.load(f)
        if 'staff' in data:
            for staff in data['staff']:
                interface.update_staff_information(staff)
        if 'products' in data:
            for products in data['products']:
                interface.update_product_information(products)

    assistant = ShopAssistant(model="llama3", customer_id="id1", data_interface=interface, debug=True)

    print("Welcome to the Sports Shop Assistant!")
    while True:
        user_input = input("You: ")
        if user_input.lower() in {"exit", "quit"}:
            break
        response = assistant.respond(user_input)
        print("[DEBUG] Response: ", response, file=sys.stderr)
        print(f"Assistant: {response['assistant_reply']}")

    assistant.extract_knowledge()

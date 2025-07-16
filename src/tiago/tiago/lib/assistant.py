from __future__ import annotations

import json
import re
import sys
from pprint import pprint
from typing import Any, Dict, List, Optional, Sequence, TypedDict, Union

import requests

try:
    from ollama import Client as OllamaClient
except ModuleNotFoundError:
    OllamaClient = None  # type: ignore

# Optional future use; not referenced directly right now
from tiago.lib.database.database import Database  # noqa: F401
from tiago.lib.map.map import Map  # noqa: F401


class _ChatMessage(TypedDict):
    role: str
    content: str


class ShopAssistant:
    """Conversational assistant for a sports shop.

    You can pick **Ollama** (local) or **Groq** (cloud) as the backend.

    Parameters
    ----------
    provider : {"ollama", "groq"}, default "ollama"
        Which LLM backend to use.
    model : str
        Model name recognised by the chosen provider (e.g. ``llama3`` or
        ``mixtral-8x7b`` for Groq).
    customer_id : str
        Unique identifier of the customer.
    subject_info : str
        One‑line description of the customer (e.g. "first‑time visitor").
    window_size : int, default 6
        How many most‑recent user utterances to consider when building a
        structured query.

    Environment variables (Groq only)
    ---------------------------------
    GROQ_API_KEY : str
        Your Groq secret key. If missing you can also pass it explicitly via
        :pymeth:`set_groq_api_key`.
    """

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    def __init__(
            self,
            model: str,
            customer_id: str,
            subject_info: str,
            provider: str = "ollama",
            groq_api_key: Optional[str] = None,
            window_size: int = 6,
    ) -> None:
        self.provider = provider.lower()
        if self.provider not in {"ollama", "groq"}:
            raise ValueError("provider must be 'ollama' or 'groq'")
        if self.provider == "groq" and not groq_api_key:
            raise ValueError(
                "Groq provider requires a Groq API key. "
                "Set it via the GROQ_API_KEY environment variable or use set_groq_api_key()."
            )
        self.model = model
        self.customer_id = customer_id
        self.subject_info = subject_info
        self.window_size = window_size
        self.customer_information: Dict[str, Any] = {}
        self.chat_history: List[_ChatMessage] = []

        # Provider‑specific client initialisation
        if self.provider == "ollama":
            if OllamaClient is None:
                raise ImportError("Ollama library not installed; pip install ollama")
            self._ollama = OllamaClient()
        else:  # groq
            self._groq_api_key: Optional[str] = groq_api_key
            self._groq_endpoint = "https://api.groq.com/openai/v1/chat/completions"

    # ----------------------------- Public helpers -----------------------------

    def set_groq_api_key(self, key: str) -> None:
        """Manually set (or override) the Groq API key at runtime."""
        self._groq_api_key = key

    def add_message(self, content: str) -> None:
        """Append a *user* message to the conversation."""
        self._append("user", content)

    def set_customer_information(self, customer_information: Dict[str, Any]) -> None:
        """Provide structured, previously‑known customer info."""
        self.customer_information = customer_information

    # ------------------------------------------------------------------
    # Core conversational methods
    # ------------------------------------------------------------------

    def answer(self, option: Optional[str] = None) -> str:
        """Generate an assistant reply (no external result table)."""
        sys_inst = self._make_base_system_instruction(option)
        reply = self._chat([sys_inst, *self.chat_history])
        self._append("assistant", reply)
        return reply

    def answer_with_results(self, products: Optional = None, staff: Optional = None, areas: Optional = None,
                            option: Optional[str] = None) -> str:
        """Generate an assistant reply that references *results*."""
        results = {}

        if products:
            results["products"] = products
        if staff:
            results["staff"] = staff
        if areas:
            for i, product in enumerate(results["products"]):
                product["location"] = areas[i]

        if products is not None and not products:
            assistant_action = (
                "You don't have the specific products in the shop the user is asking for. Tell him that and try to help him in other ways.\n"
            )
        elif staff is not None and not staff:
            assistant_action = (
                "You don't have the specific staff members in the shop the user is asking for. Tell him that and try to help him in other ways.\n"
            )
        else:
            assistant_action = (
                "Try to help the user with the following information:\n"
                f"{json.dumps(results, indent=2)}\n"
                "If the data is not relevant, do not include it in the answer and try to help the customer in other ways.\n"
            )

        parts = [
            "You are a sports shop assistant.",
            "You do not do physical action – like walking – unless instructed, you just provide information.",
            f"The person you're talking with is a {self.subject_info}.",
            "It's really important that you do not make up information, If the user asks for something you don't know, just say you don't know and try to help them in other ways.\n"
            f"This is what you know about the customer from previous conversations with them: {self.customer_information}",
            assistant_action,
            "Try not to write more than 3 short sentences per answer.",
        ]
        if option:
            parts.append(option)

        sys_inst: _ChatMessage = {"role": "system", "content": "\n".join(parts)}
        reply = self._chat([sys_inst, *self.chat_history])
        self._append("assistant", reply)
        return reply

    # ------------------------------------------------------------------
    # Knowledge extraction
    # ------------------------------------------------------------------

    def extract_knowledge(self) -> Dict[str, Any]:
        """Return structured JSON about the *customer* from full history."""
        parts: List[str] = [
            "You're analyzing a conversation between a sports‑shop assistant and a user. "
            "Extract structured information from the *user* messages only.",
            "Create a JSON with the following optional fields:\n"
            "customer_info: { name, age_category, gender, preferences, size_info, budget },\n"
            "products_of_interest, purchase_intent (high|medium|low), follow_up_needed (boolean), conversation_summary",
            "Extract *only* information that was explicitly shared. All scalar fields must be strings; lists where appropriate. follow_up_needed is boolean.",
        ]
        if self.customer_information:
            parts.append(
                "Previous structured information about the user (update with new insights): "
                f"{self.customer_information}"
            )
        parts.append("Answer only with ONE ( 1 ) JSON – no extra text.")

        sys_inst: _ChatMessage = {"role": "system", "content": "\n".join(parts)}
        result_raw = self._chat([sys_inst, {"role": "user", "content": json.dumps(self.chat_history, indent=2)}])
        try:
            return json.loads(result_raw)
        except json.JSONDecodeError as exc:
            print(f"[extract_knowledge] JSON decode error: {exc}", file=sys.stderr)
            return {}

    # ------------------------------------------------------------------
    # Direction & query detection helpers
    # ------------------------------------------------------------------

    def get_conversation_direction(self) -> str:
        """Return 'product_info' | 'staff_info' | 'walk_to_product' | 'walk_to_staff' | 'finished' | '' """
        sys_content = (
            "You're a sports shop assistant. Your task is to analyze the conversation and determine what the user is asking for.\n"
            " Answer with JSON with the following structure:\n"
            " { 'conversation_direction': 'product_info' | 'staff_info' | 'walk_to_product' | 'walk_to_staff' | 'finished' | '' }.\n"
            " Where: \n"
            " - 'product_info' means the user is asking about products information or location in the shop.\n"
            " - 'staff_info' means the user is asking about staff information or location in the shop.\n"
            " - 'walk_to_product' means the user wants to be walked to a product ( only if he asks it explicitly ).\n"
            " - 'walk_to_staff' means the user wants to be walked to a staff member ( only if he asks it explicitly ).\n"
            " - 'finished' means the conversation is over and no further action is needed ( only if he asks it explicitly ).\n"
            " - '' means the conversation is not about any of the above.\n"
            " Just answer with ONE ( 1 ) JSON, no extra text, with the most likely direction.\n"
        )
        result = self._detect_query(sys_content)
        return result.get("conversation_direction", "").strip() if isinstance(result, dict) else ""

    def detect_products_query(self) -> Dict[str, Any]:
        spec = (
            "You're a sports shop assistant. Your task is to analyze the conversation and determine what kind of product the user is looking for.\n"
            " Answer with JSON with the following structure:\n"
            "{\n"
            "'product_category': str,  # e.g. 'shoes', 'clothes', 'accessories'\n"
            "'product_name': str,  # e.g. 'running shoes', 'football jersey'\n"
            "'product_brand': str,  # e.g. 'Nike', 'Adidas'\n"
            "'product_sport_category': str,  # e.g. 'running', 'football'\n"
            "'product_description': str,  # e.g. 'lightweight running shoes for long distances'\n"
            "'price_range': str,  # e.g. 'low', 'medium', 'high'\n"
            "'product_location_needed': bool  # whether the user wants to be walked to the product location\n"
            "}\n."
            "In this case, all the fields are optional ( in case fill them with 'None' ), and you can return an empty JSON if nothing relevant.\n"
            "Answer only with ONE ( 1 ) JSON, no extra text.\n"
        )

        res = self._detect_query(spec)
        return res if any(getattr(res, "values", lambda: [])()) else {}

    def detect_staff_query(self) -> Dict[str, Any]:
        spec = (
            "You're a sports shop assistant. Your task is to analyze the conversation and determine what kind of staff the user may need or is looking for.\n"
            " Answer with JSON with the following structure:\n"
            "{\n"
            "'staff_name': str,  # e.g. 'John Doe'\n"
            "'staff_role': str,  # e.g. 'sales assistant', 'manager'\n"
            "'staff_category': str,  # e.g. 'shoes', 'clothes', 'accessories'\n"
            "'staff_location_needed': bool  # whether the user wants to be walked to the staff member's location\n"
            "}\n"
            "In this case, all the fields are optional ( in case fill them with 'None' ), and you can return an empty JSON if nothing relevant.\n"
            "Answer only with ONE ( 1 ) JSON, no extra text.\n"
        )
        res = self._detect_query(spec)
        return res if any(getattr(res, "values", lambda: [])()) else {}

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _append(self, role: str, content: str) -> None:
        self.chat_history.append({"role": role, "content": content})

    def _make_base_system_instruction(self, extra: Optional[str] = None) -> _ChatMessage:
        parts: List[str] = [
            "You're a sports shop assistant. Your role is to help customers or staffer in the shop."
            "Be concise and clear with the answer, like if you were having a normal conversation. "
            "You do not do physical actions - like walking – unless instructed, you just provide information.",
            f"The person you're talking with is a {self.subject_info}.\n",
            "It's really important that you do not make up information, If the user asks for something you don't know, "
            "just say you don't know and try to help them in other ways.",
        ]
        if self.customer_information:
            parts.append(
                "This is what you know about the customer from previous conversations with them: "
                f"{self.customer_information}"
            )
        parts.append("Try to not write more than 3 short sentences per answer.")
        if extra:
            parts.append(extra)
        return {"role": "system", "content": "\n".join(parts)}

    # --------------- Backend‑agnostic chat dispatcher ----------------

    def _chat(self, messages: Sequence[_ChatMessage]) -> str:

        if self.provider == "ollama":
            assert self._ollama is not None
            response = self._ollama.chat(model=self.model, messages=list(messages))
            return response["message"]["content"]
        # Groq branch
        if not self._groq_api_key:
            raise RuntimeError(
                "Groq API key not set. Use set_groq_api_key() or export GROQ_API_KEY before instantiating."
            )
        headers = {
            "Authorization": f"Bearer {self._groq_api_key}",
            "Content-Type": "application/json",
        }
        payload = {"model": self.model, "messages": list(messages)}
        resp = requests.post(self._groq_endpoint, headers=headers, json=payload, timeout=30)
        resp.raise_for_status()
        data = resp.json()
        return data["choices"][0]["message"]["content"]

    # ------------------ Narrow‑window LLM helper ------------------

    def _clean_json(self, raw: str) -> str:
        """Return a string that is much closer to valid JSON.
        Fixes performed, in order:
          1. Normalise quotes + newlines.
          2. Convert the *string* "None" to JSON null.
          3. Convert bare True/False/None to JSON literals.
          4. Remove trailing commas before } or ].
          5. Collapse repeated whitespace.
        """
        _PY_LITERALS: Dict[str, str] = {  # bare → JSON
            "True": "true",
            "False": "false",
            "None": "null",
        }

        # ① bare True / False / None *outside* quotes
        _BARE_PY_PATTERN = re.compile(r'(?<!")\b(?:True|False|None)\b(?!")')

        # ② the quoted string "None" (exactly that, nothing else)
        _QUOTED_NONE_PATTERN = re.compile(r'"\s*None\s*"')

        # ③ trailing comma right before } or ]
        _TRAILING_COMMA_PATTERN = re.compile(r',\s*([}\]])')

        # ④ runs of ≥2 whitespace chars
        _MULTI_WS_PATTERN = re.compile(r'\s{2,}')
        # 1. quick normalisation
        cleaned = raw.replace("'", '"').replace("\n", " ").strip()

        # 2. "None"  →  null
        cleaned = _QUOTED_NONE_PATTERN.sub("null", cleaned)

        # 3. True / False / None  →  true / false / null
        cleaned = _BARE_PY_PATTERN.sub(lambda m: _PY_LITERALS[m.group(0)], cleaned)

        # 4. kill dangling commas
        cleaned = _TRAILING_COMMA_PATTERN.sub(r'\1', cleaned)

        # 5. compact whitespace
        cleaned = _MULTI_WS_PATTERN.sub(" ", cleaned)

        return cleaned

    def _detect_query(self, system_instruction_content: str) -> Dict[str, Any]:
        sys_inst: _ChatMessage = {"role": "system", "content": system_instruction_content}
        last_user_msgs = [m["content"] for m in self.chat_history if m["role"] == "user"][-self.window_size:]
        joined = "\n".join(last_user_msgs)
        raw = self._chat([sys_inst, {"role": "user", "content": joined}])
        raw = self._clean_json(raw)
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            print(f"[_detect_query] JSON decode error: {exc}.\nRaw response: \"{raw}\"", file=sys.stderr)
            return {}

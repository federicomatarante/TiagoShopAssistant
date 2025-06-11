import sys
from enum import Enum
from typing import Dict, Type, Any, Optional, Tuple, List

import yaml
from ollama import chat
from pydantic import BaseModel, Field


class ClassesParser:
    def __init__(self, file: str):
        self.file = file
        self.raw_definitions = self._load_yaml()
        self.generated_classes: Dict[str, Tuple[Type[BaseModel], str]] = {}
        self._parse_classes()

    def _load_yaml(self) -> Dict[str, Any]:
        with open(self.file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _parse_classes(self):
        for class_name, class_info in self.raw_definitions.items():
            if not isinstance(class_info, dict) or 'fields' not in class_info:
                continue  # Skip non-class definitions
            fields = class_info['fields']
            class_descriptions = class_info['description']
            annotations = {}
            class_fields = {}
            for field_name, meta in fields.items():
                raw_type = meta.get('type', 'Any')
                field_description = meta.get('description', '')
                # Eval the type string (simple, not safe for arbitrary input)
                if 'domain' not in meta:
                    type_hint = eval(raw_type, {"__builtins__": {}}, self._safe_types())
                else:
                    values = [value.rstrip(" ").lstrip(" ") for value in meta['domain'].split(",")]
                    type_hint = self._create_enum(field_name.upper(), values)
                annotations[field_name] = type_hint
                class_fields[field_name] = Field(description=field_description, default=None)
            model = type(class_name, (BaseModel,), {"__annotations__": annotations, **class_fields})
            self.generated_classes[class_name] = (model, class_descriptions)

    def _safe_types(self) -> Dict[str, Any]:
        """Define a whitelist of safe types for eval."""
        return {
            "str": str,
            "int": int,
            "float": float,
            "bool": bool,
            "Any": Any,
            "None": type(None),
            "dict": dict,
            "list": list,
        }

    def _create_enum(self, name: str, values: list[str]):
        namespace = {'__qualname__': name}  # Richiesto per la corretta qualificazione dell'enum

        for value in values:
            namespace[value] = value

        return Enum(name, namespace)

    def getMessage(self) -> Type[BaseModel]:
        """Generate a Message class with all known model classes as optional fields."""
        annotations = {}
        class_fields = {}

        for class_name, (model_class, class_description) in self.generated_classes.items():
            annotations[class_name] = Optional[model_class]
            class_fields[class_name] = Field(description=class_description)

        message_class = type("Message", (BaseModel,), {"__annotations__": annotations, **class_fields})
        return message_class


class NLPExtractor:

    def __init__(self, assistant_role: str, temperature: float = 0.0):
        self.assistant_role = assistant_role
        self.temperature = temperature

    def extract(self, parser: ClassesParser, messages: List[Dict[str, str]], extra: str = None):
        message = parser.getMessage()
        messages.insert(0, {
            "role": "system",
            "content": self.assistant_role + "\n" + extra if extra else self.assistant_role
        })

        # DEBUG MESSAGES in stderr
        for msg in messages:
            print(f"DEBUG: {msg['role']}: {msg['content']}", file=sys.stderr)

        response = chat(
            messages=messages,
            model='llama3',
            format=message.model_json_schema(),
            options={'temperature': self.temperature},  # Set temperature to 0 for more deterministic output
        )
        data = message.model_validate_json(response.message.content).model_dump()
        return data

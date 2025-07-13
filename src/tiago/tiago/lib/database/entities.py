from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class Product:
    product_id: str
    name: Optional[str] = None
    price: Optional[float] = None
    description: Optional[str] = None
    category: Optional[str] = None
    brand: Optional[str] = None
    sport_category: Optional[str] = None

    embedded_name: Optional[np.ndarray] = None
    embedded_description: Optional[np.ndarray] = None
    embedded_category: Optional[np.ndarray] = None
    embedded_brand: Optional[np.ndarray] = None
    embedded_sport_category: Optional[np.ndarray] = None

    def to_json(self):
        return {
            'product_id': self.product_id,
            'name': self.name,
            'price': self.price,
            'description': self.description,
            'category': self.category,
            'brand': self.brand,
            'sport_category': self.sport_category,
        }


@dataclass
class Staff:
    staff_id: str
    name: Optional[str] = None
    categories: Optional[str] = None
    role: Optional[str] = None

    embedded_name: Optional[np.ndarray] = None
    embedded_role: Optional[np.ndarray] = None
    embedded_categories: Optional[np.ndarray] = None

    def to_json(self):
        return {
            'staff_id': self.staff_id,
            'name': self.name,
            'categories': self.categories,
            'role': self.role,
        }


@dataclass
class Customer:
    customer_id: str
    name: Optional[str] = None
    age_category: Optional[str] = None
    gender: Optional[str] = None
    preferences: Optional[str] = None
    size_information: Optional[str] = None
    budget: Optional[str] = None
    products_of_interest: Optional[str] = None
    purchase_intent: Optional[str] = None
    follow_up_needed: Optional[bool] = None
    conversation_summary: Optional[str] = None

    def to_json(self):
        return {
            'customer_id': self.customer_id,
            'name': self.name,
            'age_category': self.age_category,
            'gender': self.gender,
            'preferences': self.preferences,
            'size_information': self.size_information,
            'budget': self.budget,
            'products_of_interest': self.products_of_interest,
            'purchase_intent': self.purchase_intent,
            'follow_up_needed': self.follow_up_needed,
            'conversation_summary': self.conversation_summary,
        }

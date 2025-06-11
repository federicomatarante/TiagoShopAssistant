from abc import ABC, abstractmethod
from typing import List, Dict

import numpy as np
import torch
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity
from torch import cosine_similarity
from transformers import AutoTokenizer, AutoModel

from data.database.entities import Customer, Product, Staff


# TODO around all the code, use torch on GPU to make it faster! Optimize!

class SimilarityCalculator(ABC):

    @abstractmethod
    def similarity_score(self, emeddings_a: np.ndarray, embeddings_b: np.ndarray):
        pass


class CosineDistanceCalculator(SimilarityCalculator):

    def __init__(self, device: str = None):
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')

    def similarity_score(self, embeddings_a: np.ndarray, embeddings_b: np.ndarray):
        embeddings_a_tensor = torch.from_numpy(embeddings_a)
        embeddings_b_tensor = torch.from_numpy(embeddings_b)

        if embeddings_a_tensor.device != self.device:
            embeddings_a_tensor = embeddings_a_tensor.to(self.device)
        if embeddings_b_tensor.device != self.device:
            embeddings_b_tensor = embeddings_b_tensor.to(self.device)

        return cosine_similarity(embeddings_a_tensor.cpu(), embeddings_b_tensor.cpu()).numpy()


class EmbeddingsManager:
    def __init__(self, model_name='paraphrase-MiniLM-L6-v2', threshold: float = 0.7):
        self.model = SentenceTransformer(model_name)
        self.threshold = threshold

    def create_embeddings(self, sentences: List[str]) -> np.ndarray:
        if not sentences:
            return np.empty((0, self.model.get_sentence_embedding_dimension()))
        return self.model.encode(sentences, convert_to_numpy=True, normalize_embeddings=True)

    def compare_embeddings(self, embeddings_a: np.ndarray, embeddings_b: np.ndarray) -> np.ndarray:
        """
        Returns a binary list indicating whether each embedding in embeddings_a
        is similar to any in embeddings_b based on cosine similarity.
        """
        if embeddings_a is None or embeddings_b is None or embeddings_a.size == 0 or embeddings_b.size == 0:
            return np.array([], dtype=int)
        similarity_matrix = np.dot(embeddings_a, embeddings_b.T)  # cosine similarity
        return np.array([int(np.any(row >= self.threshold)) for row in similarity_matrix])


class ProductRecommender:
    def __init__(self, all_products, embeddings_manager: EmbeddingsManager = None):
        self.embeddings_manager = embeddings_manager or EmbeddingsManager()
        self.product_names_emb = np.stack([product.embedded_name for product in all_products])
        self.product_descriptions_emb = np.stack([product.embedded_description for product in all_products])
        self.product_categories_emb = np.stack([product.embedded_category for product in all_products])
        self.product_embedded_brands = np.stack([product.embedded_brand for product in all_products])
        self.product_embedded_sport_categories = np.stack([product.embedded_sport_category for product in all_products])

        self.all_products = all_products

    def get_products(self, product_request: Dict[str, str], max_products=5) -> List[Product]:

        embedding_map = {
            'product_name': self.product_names_emb,
            'product_brand': self.product_embedded_brands,
            'product_category': self.product_categories_emb,
            'product_sport_category': self.product_embedded_sport_categories,
            'product_description': self.product_descriptions_emb,
        }

        final_indices = []
        for key, stored_embs in embedding_map.items():
            values = product_request.get(key)
            if not values:
                continue
            if isinstance(values, str):
                values = [values, ]

            for value in values:
                if not value:
                    continue
                query_emb = self.embeddings_manager.create_embeddings([value])[0]
                mask = self.embeddings_manager.compare_embeddings(stored_embs, query_emb)
                indices = np.where(mask == 1)[0]
                final_indices.extend([index for index in indices if index not in final_indices])

        filtered_products = [self.all_products[i] for i in final_indices]
        if 'price_range' in product_request and product_request['price_range']:
            price_range = product_request['price_range']
            # If it's any collection
            if isinstance(price_range, list):
                price_range = price_range[0]
            if not isinstance(price_range, str):
                price_range = str(price_range)
            try:
                min_price, max_price = map(float, price_range.split('-'))
            except ValueError:
                min_price, max_price = 0, float('inf')
            filtered_products = [
                product for product in filtered_products
                if min_price <= product.price <= max_price
            ]
        return filtered_products[:max_products] if max_products > 0 else filtered_products


class StaffRecommender:
    def __init__(self, all_staff, embeddings_manager: EmbeddingsManager = None):
        self.embeddings_manager = embeddings_manager or EmbeddingsManager()
        self.staff_names_emb = np.stack([staff.embedded_name for staff in all_staff]) if all_staff else np.empty((0, 0))
        self.staff_categories_emb = [staff.embedded_categories for staff in all_staff]
        self.staff_roles_emb = np.stack([staff.embedded_role for staff in all_staff]) if all_staff else np.empty((0, 0))
        self.all_staff = all_staff

    def get_staff(self, staff_request: Dict[str, str], max_staff=5) -> List[Staff]:

        embedding_map = {
            'staff_name': self.staff_names_emb,
            'staff_role': self.staff_roles_emb,
        }
        final_indices = []
        for key, stored_embs in embedding_map.items():
            values = staff_request.get(key, None)
            if not values:
                continue
            if isinstance(values, str):
                values = [values, ]
            for value in values:
                if not value:
                    continue
                query_emb = self.embeddings_manager.create_embeddings([value])[0]
                mask = self.embeddings_manager.compare_embeddings(stored_embs, query_emb)
                indices = np.where(mask == 1)[0]
                final_indices.extend([index for index in indices if index not in final_indices])

        # Handling staff_categories separately due to variable-length lists
        category_values = staff_request.get('staff_category', None)
        if category_values:
            if isinstance(category_values, str):
                category_values = [category_values]
            for value in category_values:
                if not value:
                    continue
                query_emb = self.embeddings_manager.create_embeddings([value])[0]
                for idx, staff_cats in enumerate(self.staff_categories_emb):
                    # staff_cats is a list of embeddings
                    for cat_emb in staff_cats:
                        match = self.embeddings_manager.compare_embeddings(np.array([cat_emb]), query_emb)
                        if match[0] == 1 and idx not in final_indices:
                            final_indices.append(idx)
                            break  # stop checking other categories for this staff once matched

        filtered_staff = [self.all_staff[i] for i in final_indices]
        return filtered_staff[:max_staff] if len(filtered_staff) > max_staff else filtered_staff


class CustomerManager:

    def get_updated_customer(self, customer: Customer, customer_information: Dict[str, str]) -> Customer:
        """
        Updates a Customer dataclass instance with information extracted from customer_information.

        :param customer: The original Customer instance.
        :param customer_information: A dictionary in the format:
            {
                "customer_info": {
                    "name": ...,
                    "age_category": ...,
                    "gender": ...,
                    "preferences": ...,
                    "size_info": ...,
                    "budget": ...
                },
                "products_of_interest": ...,
                "purchase_intent": ...,
                "follow_up_needed": ...,
                "conversation_summary": ...
            }
        :return: Updated Customer instance.
        """
        if 'customer_info' in customer_information and isinstance(customer_information['customer_info'], dict):
            customer_info = customer_information['customer_info']
            if 'name' in customer_info:
                customer.name = customer_info['name']
            if 'age_category' in customer_info:
                customer.age_category = customer_info['age_category']
            if 'gender' in customer_info:
                customer.gender = customer_info['gender']
            if 'preferences' in customer_info:
                customer.preferences = customer_info['preferences']
            if 'size_info' in customer_info:
                customer.size_information = customer_info['size_info']
            if 'budget' in customer_info:
                customer.budget = customer_info['budget']

        if 'products_of_interest' in customer_information:
            customer.products_of_interest = customer_information['products_of_interest']
        if 'purchase_intent' in customer_information:
            customer.purchase_intent = customer_information['purchase_intent']
        if 'follow_up_needed' in customer_information:
            customer.follow_up_needed = customer_information['follow_up_needed']
        if 'conversation_summary' in customer_information:
            customer.conversation_summary = customer_information['conversation_summary']

        return customer

from typing import Dict, Any, Optional

from tiago.lib.database.entities import Customer, Product, Staff
from tiago.lib.map.geometry import Point2D
from tiago.lib.database.managers import EmbeddingsManager, CustomerManager, ProductRecommender, StaffRecommender
from tiago.lib.map.map import Map

from tiago.lib.database.database import Database


class DataInterface:
    def __init__(self,
                 database: Database, map: Map, similarity_threshold: float = 0.8):
        """
        Initializes the DataInterface with an NLP extractor and a database.

        :param extractor: The NLP extractor used to process and extract information from messages.
        :param database: The database instance used to store and retrieve data.
        """
        self.database = database
        self.map = map
        self.similarity_threshold = similarity_threshold
        self.embeddings_manager = EmbeddingsManager()

    def update_customer_information(self, customer_id: str, customer_information: Dict[str, Any]):
        """
        Updates the information related to a user by extracting relevant data from messages.

        :param customer_id: The unique identifier for the user.
        """

        if not customer_information:
            return
        customer = self.database.get_customer(customer_id)
        if customer:
            existing = True
        else:
            existing = False
            customer = Customer(customer_id)
        customer_manager = CustomerManager()
        customer = customer_manager.get_updated_customer(customer, customer_information)
        if existing:
            self.database.update_customer(customer)
        else:
            self.database.add_customer(customer)
        return customer.to_json()

    def get_customer_information(self, customer_id: str, ):
        customer = self.database.get_customer(customer_id) or Customer(customer_id)
        return customer

    def update_product_information(self, product_information: Dict[str, Any], product_id: Optional[str] = None):
        """
        Updates the information related to a product using provided fields and generates embeddings.

        If a product with the given ID exists, it is retrieved and updated.
        If it doesn't exist, a new product instance is created.

        :param product_id: Optional ID of the product to update. If None, a new product must be created with a unique ID externally.
        :param product_information: Dictionary containing the following optional keys:
            - 'name': Name of the product (str)
            - 'price': Price of the product (float)
            - 'description': Description of the product (str)
            - 'category': Category of the product (str)
        """
        product = self.database.get_product(product_id)
        if product:
            existing = True
        else:
            existing = False
            product_id = self.database.get_new_product_id
            product = Product(product_id=product_id)

        if 'name' in product_information:
            product.name = product_information['name']
            product.embedded_name = self.embeddings_manager.create_embeddings([product.name])[0]

        if 'price' in product_information:
            try:
                product.price = float(product_information['price'])
            except (ValueError, TypeError):
                pass
        if 'brand' in product_information:
            product.brand = product_information['brand']
            product.embedded_brand = self.embeddings_manager.create_embeddings([product.brand])[0]

        if 'description' in product_information:
            product.description = product_information['description']
            product.embedded_description = self.embeddings_manager.create_embeddings([product.description])[0]
        if 'sport_category' in product_information:
            product.sport_category = product_information['sport_category']
            product.embedded_sport_category = self.embeddings_manager.create_embeddings([product.sport_category])[0]

        if 'category' in product_information:
            product.category = product_information['category']
            product.embedded_category = self.embeddings_manager.create_embeddings([product.category])[0]
        if existing:
            self.database.update_product(product)
        else:
            self.database.add_product(product)

    def update_product_position(self, product_id: str, position: Point2D):
        """
        Updates the spatial position of a product in the environment.

        :param product_id: The unique identifier for the product.
        :param position: The new 2D position of the product.
        """
        self.map.observations.update_product_location(product_id, position)

    def update_staff_position(self, staff_id: str, position: Point2D):
        """
        Updates the spatial position of a staff member in the environment.

        :param staff_id: The unique identifier for the staff member.
        :param position: The new 2D position of the staff member.
        """
        self.map.observations.update_staff_location(staff_id, position)

    def update_staff_information(self, staff_information: Dict[str, Any], staff_id: Optional[str] = None):
        """
        Updates the information related to a staff member using provided fields and generates embeddings.

        If a staff member with the given ID exists, they are retrieved and updated.
        If they don't exist, a new staff instance is created.

        :param staff_id: Optional ID of the staff to update. If None, a new staff must be created with a unique ID externally.
        :param staff_information: Dictionary containing the following optional keys:
            - 'name': Name of the staff member (str)
            - 'categories': List of categories the staff specializes in (List[str])
            - 'role': Role of the staff member (str)
        """
        staff = self.database.get_staff(staff_id)
        if staff:
            existing = True
        else:
            existing = False
            staff = Staff(staff_id=staff_id)

        if 'name' in staff_information:
            staff.name = staff_information['name']
            staff.embedded_name = self.embeddings_manager.create_embeddings([staff.name])[0]

        if 'role' in staff_information:
            staff.role = staff_information['role']
            staff.embedded_role = self.embeddings_manager.create_embeddings([staff.role])[0]

        if 'categories' in staff_information:
            staff.categories = staff_information['categories']
            # Create embeddings for each category and then combine them
            if 'categories' in staff_information:
                staff.categories = staff_information['categories']
                # Create embeddings for all categories, preserving the 2D structure
                if staff.categories:
                    staff.embedded_categories = self.embeddings_manager.create_embeddings(staff.categories)
                else:
                    staff.embedded_categories = None

        if existing:
            self.database.update_staff(staff)
        else:
            self.database.add_staff(staff)

    def extract_relevant_products(self, product_query):
        products = None
        product_recommender = ProductRecommender(self.database.get_all_products(), self.embeddings_manager)
        answer = None
        if 'product_id' in product_query and product_query['product_id']:
            product = self.database.get_product(product_query['product_id'])
            if product:
                products = [product.to_json(), ]
        if not products:
            products = product_recommender.get_products(product_query)
        if 'product_location_needed' in product_query and product_query['product_location_needed']:
            areas = []
            for product in products:
                location = self.map.observations.get_product_location(product.product_id)
                area = self.map.get_area(location) if location else None
                if area:
                    areas.append(area[0])
                else:
                    areas.append("Unknown")
            areas = [areas[i] for i, product in
                     enumerate(products)]  # TODO this
            return products, areas
        else:
            return products, None

    def extract_relevant_staff(self, staff_query):
        staff = None
        answer = None
        staff_recommender = StaffRecommender(self.database.get_all_staff(), self.embeddings_manager)
        if 'staff_id' in staff_query and staff_query['staff_id']:
            staff = self.database.get_staff(staff_query['staff_id'])
            if staff:
                answer = [staff.to_json(), ]
        if not staff:
            staff = staff_recommender.get_staff(staff_query)
            if 'staff_location_needed' in staff_query and staff_query['staff_location_needed']:
                areas = []
                for staff_member in staff:
                    location = self.map.observations.get_staff_location(staff_member.staff_id)
                    area = self.map.get_area(location) if location else None
                    if area:
                        areas.append(area[0])
                    else:
                        areas.append("Unknown")
                areas = [areas[i] for
                         i, staff_member in
                         enumerate(staff)]
                return staff, areas
            else:
                return staff, None

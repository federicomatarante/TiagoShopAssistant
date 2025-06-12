import pickle
from sqlalchemy import Column, String, Float, JSON, LargeBinary, Boolean

from tiago.lib.database.entities import Customer, Staff, Product
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()


class ProductModel(Base):
    __tablename__ = 'products'

    product_id = Column(String, primary_key=True)
    name = Column(String)
    price = Column(Float)
    description = Column(String)
    category = Column(String)
    sport_category = Column(String)

    # Embeddings
    embedded_name = Column(LargeBinary)
    embedded_description = Column(LargeBinary)
    embedded_category = Column(LargeBinary)
    embedded_brand = Column(LargeBinary)
    embedded_sport_category = Column(LargeBinary)

    def to_dataclass(self) -> 'Product':
        return Product(
            product_id=self.product_id,
            name=self.name,
            price=self.price,
            description=self.description,
            category=self.category,
            embedded_name=pickle.loads(self.embedded_name) if self.embedded_name else None,
            embedded_description=pickle.loads(self.embedded_description) if self.embedded_description else None,
            embedded_category=pickle.loads(self.embedded_category) if self.embedded_category else None,
            embedded_brand=pickle.loads(self.embedded_brand) if self.embedded_brand else None,
            embedded_sport_category=pickle.loads(
                self.embedded_sport_category) if self.embedded_sport_category else None,
        )

    @classmethod
    def from_dataclass(cls, product: 'Product') -> 'ProductModel':
        return cls(
            product_id=product.product_id,
            name=product.name,
            price=product.price,
            description=product.description,
            category=product.category,
            embedded_name=pickle.dumps(product.embedded_name) if product.embedded_name is not None else None,
            embedded_brand=pickle.dumps(product.embedded_brand) if product.embedded_brand is not None else None,
            embedded_description=pickle.dumps(
                product.embedded_description) if product.embedded_description is not None else None,
            embedded_category=pickle.dumps(
                product.embedded_category) if product.embedded_category is not None else None,
            embedded_sport_category=pickle.dumps(
                product.embedded_sport_category) if product.embedded_sport_category is not None else None,
        )


class StaffModel(Base):
    __tablename__ = 'staffers'

    staff_id = Column(String, primary_key=True)
    name = Column(String, nullable=False)
    categories = Column(JSON)
    role = Column(String)

    # Embeddings
    embedded_name = Column(LargeBinary)
    embedded_role = Column(LargeBinary)
    embedded_categories = Column(LargeBinary)

    def to_dataclass(self) -> 'Staff':
        return Staff(
            staff_id=self.staff_id,
            name=self.name,
            categories=self.categories or [],
            role=self.role,
            embedded_name=pickle.loads(self.embedded_name) if self.embedded_name else None,
            embedded_role=pickle.loads(self.embedded_role) if self.embedded_role else None,
            embedded_categories=pickle.loads(self.embedded_categories) if self.embedded_categories else None,
        )

    @classmethod
    def from_dataclass(cls, staff: 'Staff') -> 'StaffModel':
        return cls(
            staff_id=staff.staff_id,
            name=staff.name,
            role=staff.role,
            categories=staff.categories,
            embedded_name=pickle.dumps(staff.embedded_name) if staff.embedded_name is not None else None,
            embedded_role=pickle.dumps(staff.embedded_role) if staff.embedded_role is not None else None,
            embedded_categories=pickle.dumps(
                staff.embedded_categories) if staff.embedded_categories is not None else None,
        )


class CustomerModel(Base):
    __tablename__ = 'customers'

    customer_id = Column(String, primary_key=True)
    name = Column(String, nullable=True)
    age_category = Column(String, nullable=True)
    gender = Column(String, nullable=True)
    preferences = Column(String, nullable=True)
    size_information = Column(String, nullable=True)
    budget = Column(String, nullable=True)
    products_of_interest = Column(String, nullable=True)
    purchase_intent = Column(String, nullable=True)
    follow_up_needed = Column(Boolean, nullable=True)
    conversation_summary = Column(String, nullable=True)

    @classmethod
    def from_dataclass(cls, customer: 'Customer') -> 'CustomerModel':
        return cls(
            customer_id=customer.customer_id,
            name=customer.name,
            age_category=customer.age_category,
            gender=customer.gender,
            preferences=customer.preferences,
            size_information=customer.size_information,
            budget=customer.budget,
            products_of_interest=customer.products_of_interest,
            purchase_intent=customer.purchase_intent,
            follow_up_needed=customer.follow_up_needed,
            conversation_summary=customer.conversation_summary,
        )

    def to_dataclass(self) -> 'Customer':
        return Customer(
            customer_id=self.customer_id,
            name=self.name,
            age_category=self.age_category,
            gender=self.gender,
            preferences=self.preferences,
            size_information=self.size_information,
            budget=self.budget,
            products_of_interest=self.products_of_interest,
            purchase_intent=self.purchase_intent,
            follow_up_needed=self.follow_up_needed,
            conversation_summary=self.conversation_summary,
        )

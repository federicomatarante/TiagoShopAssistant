import functools
import os
import time
from datetime import datetime
from typing import List, Dict, Any, Optional, TypeVar, Generic, Union

from sqlalchemy import create_engine, Column, String, Float, JSON, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

from tiago.lib.database.entities import Customer, Staff, Product
from tiago.lib.database.orm_entities import ProductModel, StaffModel, CustomerModel, Base

# Cache System Implementation
T = TypeVar('T')


class Cache(Generic[T]):
    def __init__(self, ttl: int = 300):
        """
        Initialize cache with time-to-live (TTL) in seconds
        """
        self.cache: Dict[str, Dict[Any, Any]] = {}
        self.ttl = ttl
        self.timestamps: Dict[str, Dict[Any, float]] = {}

    def get(self, key: str, subkey: Any = None) -> Optional[Union[T, Dict[Any, T]]]:
        """
        Get value from cache
        """
        if key not in self.cache:
            return None

        if subkey is not None:
            if subkey not in self.cache[key]:
                return None

            timestamp = self.timestamps[key][subkey]
            if time.time() - timestamp > self.ttl:
                # Cache expired
                del self.cache[key][subkey]
                del self.timestamps[key][subkey]
                return None

            return self.cache[key][subkey]
        else:
            # Return all non-expired items for the key
            result = {}
            for sk, value in self.cache[key].items():
                timestamp = self.timestamps[key][sk]
                if time.time() - timestamp <= self.ttl:
                    result[sk] = value
                else:
                    # Clean up expired items
                    del self.cache[key][sk]
                    del self.timestamps[key][sk]

            return result

    def set(self, key: str, subkey: Any, value: T) -> None:
        """
        Set value in cache
        """
        if key not in self.cache:
            self.cache[key] = {}
            self.timestamps[key] = {}

        self.cache[key][subkey] = value
        self.timestamps[key][subkey] = time.time()

    def invalidate(self, key: str, subkey: Any = None) -> None:
        """
        Remove a key or subkey from the cache
        """
        if key not in self.cache:
            return

        if subkey is not None:
            if subkey in self.cache[key]:
                del self.cache[key][subkey]
                del self.timestamps[key][subkey]
        else:
            del self.cache[key]
            del self.timestamps[key]

    def clear(self) -> None:
        """
        Clear the entire cache
        """
        self.cache.clear()
        self.timestamps.clear()


def cached(func):
    """
    Decorator for caching function results
    """

    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        # Create a key based on the function name and arguments
        key = func.__name__

        # Create a unique subkey based on the arguments
        arg_key = str(args) + str(sorted(kwargs.items()))

        result = self.cache.get(key, arg_key)
        if result is None:
            result = func(self, *args, **kwargs)
            self.cache.set(key, arg_key, result)

        return result

    return wrapper


class Database:
    def __init__(self, db_path: str = "store.db", cache_ttl: int = 300):
        self.engine = create_engine(f'sqlite:///{db_path}')
        Base.metadata.create_all(self.engine)
        self.db_path = db_path
        self.Session = sessionmaker(bind=self.engine)
        self.cache = Cache(ttl=cache_ttl)

    # Product CRUD operations
    def add_product(self, product: Product) -> Product:
        session = self.Session()
        try:
            product_model = ProductModel.from_dataclass(product)
            session.add(product_model)
            session.commit()
            # Invalidate cache
            self.cache.invalidate("get_all_products")
            self.cache.invalidate("get_product", product.product_id)
            return product
        finally:
            session.close()

    @cached
    def get_product(self, product_id: str) -> Optional[Product]:
        session = self.Session()
        try:
            product_model = session.query(ProductModel).filter_by(product_id=product_id).first()
            if product_model:
                return product_model.to_dataclass()
            return None
        finally:
            session.close()

    @cached
    def get_all_products(self) -> List[Product]:
        session = self.Session()
        try:
            product_models = session.query(ProductModel).all()
            return [model.to_dataclass() for model in product_models]
        finally:
            session.close()

    def update_product(self, product: Product) -> Product:
        session = self.Session()
        try:
            product_model = session.query(ProductModel).filter_by(product_id=product.product_id).first()
            if product_model:
                product_model.name = product.name
                product_model.price = product.price
                product_model.description = product.description
                product_model.category = product.category
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_products")
                self.cache.invalidate("get_product", product.product_id)
            return product
        finally:
            session.close()

    def delete_product(self, product_id: str) -> bool:
        session = self.Session()
        try:
            product_model = session.query(ProductModel).filter_by(product_id=product_id).first()
            if product_model:
                session.delete(product_model)
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_products")
                self.cache.invalidate("get_product", product_id)
                return True
            return False
        finally:
            session.close()

    # Staff CRUD operations
    def add_staff(self, staff: Staff) -> Staff:
        session = self.Session()
        try:
            staff_model = StaffModel.from_dataclass(staff)
            session.add(staff_model)
            session.commit()
            # Invalidate cache
            self.cache.invalidate("get_all_staff")
            self.cache.invalidate("get_staff", staff.staff_id)
            return staff
        finally:
            session.close()

    @cached
    def get_staff(self, staff_id: str) -> Optional[Staff]:
        session = self.Session()
        try:
            staff_model = session.query(StaffModel).filter_by(staff_id=staff_id).first()
            if staff_model:
                return staff_model.to_dataclass()
            return None
        finally:
            session.close()

    @cached
    def get_all_staff(self) -> List[Staff]:
        session = self.Session()
        try:
            staff_models = session.query(StaffModel).all()
            return [model.to_dataclass() for model in staff_models]
        finally:
            session.close()

    def update_staff(self, staff: Staff) -> Staff:
        session = self.Session()
        try:
            staff_model = session.query(StaffModel).filter_by(staff_id=staff.staff_id).first()
            if staff_model:
                staff_model.name = staff.name
                staff_model.role = staff.role

                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_staff")
                self.cache.invalidate("get_staff", staff.staff_id)
            return staff
        finally:
            session.close()

    def delete_staff(self, staff_id: str) -> bool:
        session = self.Session()
        try:
            staff_model = session.query(StaffModel).filter_by(staff_id=staff_id).first()
            if staff_model:
                session.delete(staff_model)
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_staff")
                self.cache.invalidate("get_staff", staff_id)
                return True
            return False
        finally:
            session.close()

    # Customer CRUD operations
    def add_customer(self, customer: Customer) -> Customer:
        session = self.Session()
        try:
            customer_model = CustomerModel.from_dataclass(customer)
            session.add(customer_model)
            session.commit()
            # Invalidate cache
            self.cache.invalidate("get_all_customers")
            self.cache.invalidate("get_customer", customer.customer_id)
            return customer
        finally:
            session.close()

    @cached
    def get_customer(self, customer_id: str) -> Optional[Customer]:
        session = self.Session()
        try:
            customer_model = session.query(CustomerModel).filter_by(customer_id=customer_id).first()
            if customer_model:
                return customer_model.to_dataclass()
            return None
        finally:
            session.close()

    @cached
    def get_all_customers(self) -> List[Customer]:
        session = self.Session()
        try:
            customer_models = session.query(CustomerModel).all()
            return [model.to_dataclass() for model in customer_models]
        finally:
            session.close()

    def update_customer(self, customer: Customer) -> Customer:
        session = self.Session()
        try:
            customer_model = session.query(CustomerModel).filter_by(customer_id=customer.customer_id).first()
            if customer_model:
                customer_model.name = customer.name
                customer_model.age_category = customer.age_category
                customer_model.what_he_likes = customer.what_he_likes
                product_ids = customer.what_hes_found
                product_models = session.query(ProductModel).filter(ProductModel.product_id.in_(product_ids)).all()
                customer_model.what_hes_found = product_models
                customer_model.what_he_cannot_find = customer.what_he_cannot_find
                customer_model.last_modified = datetime.now()
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_customers")
                self.cache.invalidate("get_customer", customer.customer_id)
            return customer
        finally:
            session.close()

    def delete_customer(self, customer_id: str) -> bool:
        session = self.Session()
        try:
            customer_model = session.query(CustomerModel).filter_by(customer_id=customer_id).first()
            if customer_model:
                session.delete(customer_model)
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_customers")
                self.cache.invalidate("get_customer", customer_id)
                return True
            return False
        finally:
            session.close()

    def delete_expired_items(self, timedelta: datetime):
        session = self.Session()
        try:
            current = datetime.now()
            customer_models = session.query(CustomerModel).filter_by(created_time_ge=timedelta - current)
            if customer_models:
                customer_ids = [customer_model.customer_id for customer_model in customer_models]
                session.delete(customer_models)
                session.commit()
                # Invalidate cache
                self.cache.invalidate("get_all_customers")
                for customer_id in customer_ids:
                    self.cache.invalidate("get_customer", customer_id)
                return True
            return False
        finally:
            session.close()

    def clear_cache(self) -> None:
        """
        Clear the entire cache
        """
        self.cache.clear()

    def get_new_staff_id(self) -> str:
        """
        Generate a new staff ID
        """
        session = self.Session()
        try:
            max_id = session.query(StaffModel).count()
            if max_id:
                return "S" + str(int(max_id) + 1)
            else:
                return "S1"
        finally:
            session.close()

    @property
    def get_new_product_id(self):
        """
        Generate a new product ID
        """
        session = self.Session()
        try:
            max_id = session.query(ProductModel).count()
            if max_id:
                return "P" + str(int(max_id) + 1)
            else:
                return "P1"
        finally:
            session.close()

    def delete(self):
        """
        Delete all data from the database
        """
        session = self.Session()
        try:
            session.query(ProductModel).delete()
            session.query(StaffModel).delete()
            session.query(CustomerModel).delete()
            session.commit()
        finally:
            session.close()
            self.clear_cache()

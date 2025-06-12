from tiago.lib.database.entities import Product, Staff, Customer
from tiago.msg import Product as ProductMsg
from tiago.msg import Staff as StaffMsg
from tiago.msg import Customer as CustomerMsg
from tiago.msg import ProductQuery
from tiago.msg import StaffQuery


def product_to_msg(product: Product) -> ProductMsg:
    return ProductMsg(
        product_id=product.product_id,
        name=product.name or "",
        price=product.price if product.price is not None else 0.0,
        description=product.description or "",
        category=product.category or "",
        brand=product.brand or "",
        sport_category=product.sport_category or "",
    )


def staff_to_msg(staff: Staff) -> StaffMsg:
    return StaffMsg(
        staff_id=staff.staff_id,
        name=staff.name or "",
        categories=staff.categories or [],
        role=staff.role or "",
    )


def customer_to_msg(customer: Customer) -> CustomerMsg:
    return CustomerMsg(
        customer_id=customer.customer_id,
        name=customer.name or "",
        age_category=customer.age_category or "",
        gender=customer.gender or "",
        preferences=customer.preferences or "",
        size_information=customer.size_information or "",
        budget=customer.budget or "",
        products_of_interest=customer.products_of_interest or "",
        purchase_intent=customer.purchase_intent or "",
        follow_up_needed=customer.follow_up_needed if customer.follow_up_needed is not None else False,
        conversation_summary=customer.conversation_summary or "",
    )


# ---------------------
# Reverse Conversions: msg → dataclass
# ---------------------

def msg_to_product(msg: ProductMsg) -> Product:
    return Product(
        product_id=msg.product_id,
        name=msg.name if msg.name else None,
        price=msg.price if msg.price != 0.0 else None,
        description=msg.description if msg.description else None,
        category=msg.category if msg.category else None,
        brand=msg.brand if msg.brand else None,
        sport_category=msg.sport_category if msg.sport_category else None
    )


def msg_to_staff(msg: StaffMsg) -> Staff:
    return Staff(
        staff_id=msg.staff_id,
        name=msg.name if msg.name else None,
        categories=msg.categories if msg.categories else [],
        role=msg.role if msg.role else None
    )


def msg_to_customer(msg: CustomerMsg) -> Customer:
    return Customer(
        customer_id=msg.customer_id,
        name=msg.name if msg.name else None,
        age_category=msg.age_category if msg.age_category else None,
        gender=msg.gender if msg.gender else None,
        preferences=",".join(msg.preferences) if msg.preferences else None,
        size_information=msg.size_information if msg.size_information else None,
        budget=msg.budget if msg.budget else None,
        products_of_interest=",".join(msg.products_of_interest) if msg.products_of_interest else None,
        purchase_intent=msg.purchase_intent if msg.purchase_intent else None,
        follow_up_needed=msg.follow_up_needed,
        conversation_summary=msg.conversation_summary if msg.conversation_summary else None
    )


def _clean_dict(d):
    return {
        k: v for k, v in d.items()
        if v not in (None, "", []) and not (isinstance(v, dict) and _clean_dict(v) == {})
    }


def customer_to_info(request: CustomerMsg, remove_null: bool = False) -> dict:
    """
    Converts a CustomerMsg into a structured dictionary.

    Args:
        request (CustomerMsg): The customer message to convert.
        remove_null (bool): If True, remove keys with None, empty strings, or empty lists.

    Returns:
        dict: A dictionary representation of the customer info.
    """

    customer_info = {
        'name': request.customer.name,
        'age_category': request.customer.age_category,
        'gender': request.customer.gender,
        'preferences': request.customer.preferences,
        'size_information': request.customer.size_information,
        'budget': request.customer.budget
    }

    full_info = {
        'customer_info': customer_info,
        'products_of_interest': request.customer.products_of_interest,
        'purchase_intent': request.customer.purchase_intent,
        'follow_up_needed': request.customer.follow_up_needed,
        'conversation_summary': request.customer.conversation_summary
    }
    return _clean_dict(full_info) if remove_null else full_info


def product_to_info(product_msg: ProductMsg, remove_null: bool = False) -> dict:
    """
    Converts a ProductMsg into a structured dictionary with optional fields.

    :param product_msg: ProductMsg instance received via ROS2
    :param remove_null: If True, removes keys with None, empty strings, or empty lists
    :return: Dictionary with optional product fields
    """

    product_info = {
        'name': product_msg.name,
        'price': product_msg.price,
        'description': product_msg.description,
        'category': product_msg.category,
        'brand': product_msg.brand,
        'sport_category': product_msg.sport_category
    }

    return _clean_dict(product_info) if remove_null else product_info


def staff_to_info(staff_msg: StaffMsg, remove_null: bool = False) -> dict:
    """
    Converts a StaffMsg into a structured dictionary with optional fields.

    :param staff_msg: StaffMsg instance received via ROS2
    :param remove_null: If True, removes keys with None, empty strings, or empty lists
    :return: Dictionary with optional staff fields
    """

    staff_info = {
        'name': staff_msg.name,
        'categories': list(staff_msg.categories),
        'role': staff_msg.role
    }

    return _clean_dict(staff_info) if remove_null else staff_info


############################
# Product Query Conversion
############################

def product_query_to_dict(query_msg: ProductQuery) -> dict:
    """
    Converts a ProductQuery ROS message into a dictionary suitable for product filtering.

    Args:
        query_msg (ProductQuery): ROS2 message containing product query fields.

    Returns:
        dict: Dictionary where keys are product query keys and values are strings, lists of strings, or bools.
    """

    def normalize_list_field(field):
        if not field:
            return None
        if len(field) == 1:
            return field[0]
        return list(field)

    result = {
        'product_name': normalize_list_field(query_msg.product_name),
        'product_brand': normalize_list_field(query_msg.product_brand),
        'product_category': normalize_list_field(query_msg.product_category),
        'product_sport_category': normalize_list_field(query_msg.product_sport_category),
        'product_description': normalize_list_field(query_msg.product_description),
        'price_range': query_msg.price_range.strip() if query_msg.price_range else None,
        'product_location_needed': query_msg.product_location_needed
    }

    if result['price_range'] == "":
        result['price_range'] = None

    return result


def staff_query_to_dict(query_msg: StaffQuery) -> dict:
    """
    Converts a StaffQuery ROS message into a dictionary suitable for staff filtering.

    Args:
        query_msg (StaffQuery): ROS2 message containing staff query fields.

    Returns:
        dict: Dictionary where keys are staff query keys and values are strings, lists of strings, or bools.
    """

    def normalize_list_field(field):
        if not field:
            return None
        if len(field) == 1:
            return field[0]
        return list(field)

    result = {
        'staff_name': normalize_list_field(query_msg.staff_name),
        'staff_role': normalize_list_field(query_msg.staff_role),
        'staff_category': normalize_list_field(query_msg.staff_category),
        'staff_id': query_msg.staff_id if query_msg.staff_id else None,
        'staff_location_needed': query_msg.staff_location_needed
    }

    return result

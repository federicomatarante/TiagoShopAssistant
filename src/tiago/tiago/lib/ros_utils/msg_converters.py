from tiago.lib.database.entities import Product, Staff, Customer
from tiago.msg import Product as ProductMsg
from tiago.msg import Staff as StaffMsg
from tiago.msg import Customer as CustomerMsg
from tiago.msg import ProductQuery
from tiago.msg import StaffQuery
from tiago.lib.map.geometry import Point2D # Import Point2D class
from geometry_msgs.msg import Point # Assuming Point ROS message is from geometry_msgs.msg. If tiago has its own Point.msg, adjust this import accordingly.


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
        preferences=customer.preferences or "", # Ensure preferences is a list
        size_information=customer.size_information or "",
        budget=customer.budget or "",
        products_of_interest=customer.products_of_interest or "", # Ensure products_of_interest is a list
        purchase_intent=customer.purchase_intent or "",
        follow_up_needed=customer.follow_up_needed if customer.follow_up_needed is not None else False,
        conversation_summary=customer.conversation_summary or "",
    )


def point2d_to_msg(point2d: Point2D) -> Point:
    """
    Converts a Point2D object into a geometry_msgs.msg.Point message.

    Args:
        point2d (Point2D): The Point2D object to convert.

    Returns:
        Point: A geometry_msgs.msg.Point message with x, y, and z (set to 0.0) coordinates.
    """
    point_msg = Point()
    point_msg.x = float(point2d.x)
    point_msg.y = float(point2d.y)
    point_msg.z = 0.0 # Point2D does not have a z-component, so set to 0.0
    return point_msg


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
        # Ensure preferences and products_of_interest are stored as comma-separated strings
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


def customer_to_info(customer_msg: CustomerMsg, remove_null: bool = False) -> dict: # Changed request to customer_msg
    """
    Converts a CustomerMsg into a structured dictionary.

    Args:
        customer_msg (CustomerMsg): The customer message to convert.
        remove_null (bool): If True, remove keys with None, empty strings, or empty lists.

    Returns:
        dict: A dictionary representation of the customer info.
    """

    customer_info = {
        'name': customer_msg.name,
        'age_category': customer_msg.age_category,
        'gender': customer_msg.gender,
        'preferences': customer_msg.preferences, # preferences is already a list in CustomerMsg
        'size_information': customer_msg.size_information,
        'budget': customer_msg.budget
    }

    full_info = {
        'customer_info': customer_info,
        'products_of_interest': customer_msg.products_of_interest, # products_of_interest is already a list in CustomerMsg
        'purchase_intent': customer_msg.purchase_intent,
        'follow_up_needed': customer_msg.follow_up_needed,
        'conversation_summary': customer_msg.conversation_summary
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


def dict_to_product_query(query_dict: dict) -> ProductQuery:
    """
    Converts a dictionary into a ProductQuery ROS message.

    Args:
        query_dict (dict): Dictionary containing product query fields.

    Returns:
        ProductQuery: ROS2 message suitable for product filtering.
    """

    def normalize_to_list(field):
        """Convert single values to lists, handle None values"""
        if field is None:
            return []
        if isinstance(field, str):
            return [field]
        if isinstance(field, list):
            return field
        return [str(field)]

    query_msg = ProductQuery()

    # Handle product_name
    if 'product_name' in query_dict:
        query_msg.product_name = normalize_to_list(query_dict['product_name'])

    # Handle product_brand
    if 'product_brand' in query_dict:
        query_msg.product_brand = normalize_to_list(query_dict['product_brand'])

    # Handle product_category
    if 'product_category' in query_dict:
        query_msg.product_category = normalize_to_list(query_dict['product_category'])

    # Handle product_sport_category
    if 'product_sport_category' in query_dict:
        query_msg.product_sport_category = normalize_to_list(query_dict['product_sport_category'])

    # Handle product_description
    if 'product_description' in query_dict:
        query_msg.product_description = normalize_to_list(query_dict['product_description'])

    # Handle price_range
    if 'price_range' in query_dict and query_dict['price_range'] is not None:
        query_msg.price_range = str(query_dict['price_range'])
    else:
        query_msg.price_range = ""

    # Handle product_location_needed
    if 'product_location_needed' in query_dict:
        query_msg.product_location_needed = bool(query_dict['product_location_needed'])
    else:
        query_msg.product_location_needed = False

    # Handle product_id if present
    if 'product_id' in query_dict and query_dict['product_id'] is not None:
        query_msg.product_id = str(query_dict['product_id'])
    else:
        query_msg.product_id = ""

    return query_msg


def dict_to_staff_query(query_dict: dict) -> StaffQuery:
    """
    Converts a dictionary into a StaffQuery ROS message.

    Args:
        query_dict (dict): Dictionary containing staff query fields.

    Returns:
        StaffQuery: ROS2 message suitable for staff filtering.
    """

    def normalize_to_list(field):
        """Convert single values to lists, handle None values"""
        if field is None:
            return []
        if isinstance(field, str):
            return [field]
        if isinstance(field, list):
            return field
        return [str(field)]

    query_msg = StaffQuery()

    # Handle staff_name
    if 'staff_name' in query_dict:
        query_msg.staff_name = normalize_to_list(query_dict['staff_name'])

    # Handle staff_role
    if 'staff_role' in query_dict:
        query_msg.staff_role = normalize_to_list(query_dict['staff_role'])

    # Handle staff_category
    if 'staff_category' in query_dict:
        query_msg.staff_category = normalize_to_list(query_dict['staff_category'])

    # Handle staff_id
    if 'staff_id' in query_dict and query_dict['staff_id'] is not None:
        query_msg.staff_id = str(query_dict['staff_id'])
    else:
        query_msg.staff_id = ""

    # Handle staff_location_needed
    if 'staff_location_needed' in query_dict:
        query_msg.staff_location_needed = bool(query_dict['staff_location_needed'])
    else:
        query_msg.staff_location_needed = False

    return query_msg


############################
# Enhanced Customer Info Converter
############################

def dict_to_customer_info(customer_info_dict: dict) -> CustomerMsg:
    """
    Converts a dictionary containing customer information into a CustomerMsg.

    This is useful when you have customer information as a dictionary and need
    to convert it to a ROS message format.

    Args:
        customer_info_dict (dict): Dictionary containing customer information fields.

    Returns:
        CustomerMsg: ROS2 message containing customer information.
    """

    customer_msg = CustomerMsg()

    # Handle basic customer info
    customer_msg.customer_id = customer_info_dict.get('customer_id', '')
    customer_msg.name = customer_info_dict.get('name', '')
    customer_msg.age_category = customer_info_dict.get('age_category', '')
    customer_msg.gender = customer_info_dict.get('gender', '')
    customer_msg.size_information = customer_info_dict.get('size_information', '')
    customer_msg.budget = customer_info_dict.get('budget', '')
    customer_msg.purchase_intent = customer_info_dict.get('purchase_intent', '')
    customer_msg.conversation_summary = customer_info_dict.get('conversation_summary', '')

    # Handle preferences (convert from string to list if needed)
    preferences = customer_info_dict.get('preferences', [])
    if isinstance(preferences, str):
        customer_msg.preferences = [pref.strip() for pref in preferences.split(',') if pref.strip()]
    elif isinstance(preferences, list):
        customer_msg.preferences = preferences
    else:
        customer_msg.preferences = ""

    # Handle products_of_interest (convert from string to list if needed)
    products_of_interest = customer_info_dict.get('products_of_interest', [])
    if isinstance(products_of_interest, str):
        customer_msg.products_of_interest = [prod.strip() for prod in products_of_interest.split(',') if prod.strip()]
    elif isinstance(products_of_interest, list):
        customer_msg.products_of_interest = products_of_interest
    else:
        customer_msg.products_of_interest = []

    # Handle follow_up_needed
    follow_up = customer_info_dict.get('follow_up_needed', False)
    if isinstance(follow_up, str):
        customer_msg.follow_up_needed = follow_up.lower() in ('true', '1', 'yes', 'on')
    else:
        customer_msg.follow_up_needed = bool(follow_up)

    return customer_msg
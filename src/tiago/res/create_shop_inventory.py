import csv

PRODUCTS_FILE = 'products.csv'
STAFF_FILE = 'staff.csv'


def get_products():
    """Read products from a CSV file."""
    products = []
    try:
        with open(PRODUCTS_FILE, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                products.append(row)
    except FileNotFoundError:
        print(f"Error: {PRODUCTS_FILE} not found.")
    return products


def get_staff():
    """Read staff from a CSV file."""
    staff = []
    try:
        with open(STAFF_FILE, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                staff.append(row)
    except FileNotFoundError:
        print(f"Error: {STAFF_FILE} not found.")
    return staff


def get_areas():
    return [
        {
            "name": "Tennis",
            "points": [
                {"x": -10, "y": -5},
                {"x": -6, "y": -5},
                {"x": -6, "y": 5},
                {"x": -10, "y": 5}
            ]
        },
        {
            "name": "Soccer",
            "points": [
                {"x": -6, "y": -5},
                {"x": 0, "y": -5},
                {"x": 0, "y": 5},
                {"x": -6, "y": 5}
            ]
        },
        {
            "name": "Basketball",
            "points": [
                {"x": 0, "y": -5},
                {"x": 6, "y": -5},
                {"x": 6, "y": 5},
                {"x": 0, "y": 5}
            ]
        },
        {
            "name": "Baseball",
            "points": [
                {"x": 6, "y": -5},
                {"x": 10, "y": -5},
                {"x": 10, "y": 5},
                {"x": 6, "y": 5}
            ]
        }
    ]


def create_shop_inventory_json_file(products, staff, areas):
    """Create a JSON file for the shop inventory."""
    import json
    inventory = {
        "products": products,
        "staff": staff,
        "areas": areas
    }

    with open('shop_inventory.json', 'w') as file:
        json.dump(inventory, file, indent=4)
    print("Shop inventory JSON file created successfully.")


create_shop_inventory_json_file(get_products(), get_staff(), get_areas())
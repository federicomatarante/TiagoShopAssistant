#!/usr/bin/env python3

import json
import sys
import os
import threading
from typing import Dict, Any
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# Updated imports
from tiago.lib.database.entities import Product, Staff
from tiago.lib.database.database import Database

from tiago.lib.database.managers import EmbeddingsManager


class DatabaseLoaderNode(Node):
    """
    ROS2 Node that loads database from JSON file when 'load' command is typed in terminal
    """

    def __init__(self):
        super().__init__('database_loader_node')

        # Get package share directory and set file paths
        package_share = get_package_share_directory('tiago')

        self.db_path = os.path.join(package_share, 'res', 'database.db')
        self.embedder = EmbeddingsManager()

        self.json_file_path = os.path.join(package_share, 'res', 'shop_inventory.json')

        # Create publisher for status updates
        self.status_publisher = self.create_publisher(String, 'db_loader_status', 10)

        self.get_logger().info('Database Loader Node initialized')
        self.get_logger().info(f'JSON file path: {self.json_file_path}')
        self.get_logger().info(f'Database path: {self.db_path}')

        # Start the command input thread
        self.command_thread = threading.Thread(target=self.command_input_loop, daemon=True)
        self.command_thread.start()

        print("\n" + "=" * 60)
        print("DATABASE LOADER ROS2 NODE - INTERACTIVE MODE")
        print("=" * 60)
        print("Node is running and ready for commands!")
        print(f"JSON file: {self.json_file_path}")
        print(f"Database: {self.db_path}")
        print("\nAvailable commands:")
        print("  load    - Load database from JSON file")
        print("  status  - Show current file paths")
        print("  exit    - Shutdown the node")
        print("\nType your command and press Enter:")
        print("=" * 60)

    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)

    def command_input_loop(self):
        """
        Loop to handle user input commands
        """
        while rclpy.ok():
            try:
                command = input("db_loader> ").strip().lower()

                if command == "load":
                    success, message = self.load_database()
                    if success:
                        print(f"✅ SUCCESS: {message}")
                    else:
                        print(f"❌ FAILED: {message}")

                elif command == "status":
                    print(f"📁 JSON file: {self.json_file_path}")
                    print(f"💾 Database: {self.db_path}")
                    print(f"📂 JSON exists: {Path(self.json_file_path).exists()}")
                    print(f"💽 DB exists: {Path(self.db_path).exists()}")

                elif command == "exit":
                    print("Shutting down node...")
                    rclpy.shutdown()
                    break

                elif command == "help":
                    print("\nAvailable commands:")
                    print("  load    - Load database from JSON file")
                    print("  status  - Show current file paths")
                    print("  help    - Show this help message")
                    print("  exit    - Shutdown the node")

                elif command == "":
                    continue

                else:
                    print(f"Unknown command: '{command}'. Type 'help' for available commands.")

            except EOFError:
                # Handle Ctrl+D
                print("\nShutting down node...")
                rclpy.shutdown()
                break
            except KeyboardInterrupt:
                # Handle Ctrl+C
                print("\nShutting down node...")
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"Error processing command: {e}")

    def load_data_from_json(self, file_path: str) -> Dict[str, Any]:
        """
        Load data from JSON file

        Args:
            file_path (str): Path to the JSON file

        Returns:
            Dict[str, Any]: Parsed JSON data

        Raises:
            FileNotFoundError: If the file doesn't exist
            json.JSONDecodeError: If the JSON is invalid
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            raise FileNotFoundError(f"File '{file_path}' not found.")
        except json.JSONDecodeError as e:
            raise json.JSONDecodeError(f"Invalid JSON in file '{file_path}': {e}")
        except Exception as e:
            raise Exception(f"Error reading file '{file_path}': {e}")

    def load_staff_data(self, db: Database, staff_data: list) -> int:
        """
        Load staff data into the database

        Args:
            db (Database): Database instance
            staff_data (list): List of staff dictionaries

        Returns:
            int: Number of staff members loaded
        """
        loaded_count = 0

        for staff_info in staff_data:
            try:
                # Create Staff entity
                staff = Staff(
                    staff_id=staff_info['staff_id'],
                    name=staff_info['name'],
                    role=staff_info['role'],
                    categories=staff_info['categories'],
                    embedded_name=self.embedder.create_embeddings([staff_info['name'],])[0],
                    embedded_role=self.embedder.create_embeddings([staff_info['role'],])[0],
                    embedded_categories=self.embedder.create_embeddings([staff_info['categories'],])[0],
                    # If your Staff entity has a categories field, uncomment the next line
                    # categories=staff_info.get('categories', [])
                )


                db.add_staff(staff)
                loaded_count += 1
                self.publish_status(f"Loaded staff: {staff.name} ({staff.staff_id})")
                print(f"  ✓ Loaded staff: {staff.name}")

            except KeyError as e:
                error_msg = f"Missing required field {e} in staff data: {staff_info}"
                self.get_logger().error(error_msg)
                print(f"  ✗ Error: {error_msg}")
            except Exception as e:
                error_msg = f"Error loading staff {staff_info.get('staff_id', 'unknown')}: {e}"
                self.get_logger().error(error_msg)
                print(f"  ✗ Error: {error_msg}")

        return loaded_count

    def load_product_data(self, db: Database, product_data: list) -> int:
        """
        Load product data into the database

        Args:
            db (Database): Database instance
            product_data (list): List of product dictionaries

        Returns:
            int: Number of products loaded
        """
        loaded_count = 0

        for product_info in product_data:
            try:
                # Create Product entity
                product = Product(
                    product_id=product_info['product_id'],
                    name=product_info['name'],
                    price=float(product_info['price']),
                    description=product_info['description'],
                    category=product_info['category'],
                    embedded_name=self.embedder.create_embeddings([product_info['name'],])[0],
                    embedded_description=self.embedder.create_embeddings([product_info['description'],])[0],
                    embedded_category=self.embedder.create_embeddings([product_info['category'],])[0],
                    embedded_brand=self.embedder.create_embeddings([product_info.get('brand', '')])[0],
                    embedded_sport_category=self.embedder.create_embeddings([product_info.get('sport_category', '')])[0],
                )


                db.add_product(product)
                loaded_count += 1
                self.publish_status(f"Loaded product: {product.name} ({product.product_id}) - ${product.price}")
                print(f"  ✓ Loaded product: {product.name}")

            except KeyError as e:
                error_msg = f"Missing required field {e} in product data: {product_info}"
                self.get_logger().error(error_msg)
                print(f"  ✗ Error: {error_msg}")
            except ValueError as e:
                error_msg = f"Invalid price value in product {product_info.get('product_id', 'unknown')}: {e}"
                self.get_logger().error(error_msg)
                print(f"  ✗ Error: {error_msg}")
            except Exception as e:
                error_msg = f"Error loading product {product_info.get('product_id', 'unknown')}: {e}"
                self.get_logger().error(error_msg)
                print(f"  ✗ Error: {error_msg}")

        return loaded_count


    def load_database(self):
        """
        Load database from JSON file
        """
        try:
            print("\n🔄 Starting database loading...")

            json_file_path = self.json_file_path
            db_path = self.db_path

            self.publish_status(f"Loading data from: {json_file_path}")
            self.publish_status(f"Database path: {db_path}")

            # Validate file exists
            if not Path(json_file_path).exists():
                error_message = f"JSON file not found: {json_file_path}"
                self.get_logger().error(error_message)
                return False, error_message

            print(f"📂 Reading JSON file: {json_file_path}")

            # Load JSON data
            data = self.load_data_from_json(json_file_path)

            print(f"💾 Initializing database: {db_path}")

            # Initialize database
            db = Database(db_path=db_path)
            db.delete()
            self.publish_status("Database initialized successfully.")

            # Load data
            staff_count = 0
            product_count = 0

            # Load staff data if present
            if 'staff' in data:
                print(f"👥 Loading {len(data['staff'])} staff members...")
                self.publish_status(f"Loading {len(data['staff'])} staff members...")
                staff_count = self.load_staff_data(db, data['staff'])
                self.publish_status(f"Successfully loaded {staff_count} staff members.")
                print(f"✅ Loaded {staff_count} staff members")
            else:
                print("ℹ️  No staff data found in JSON file")
                self.publish_status("No staff data found in JSON file.")

            # Load product data if present
            if 'products' in data:
                print(f"📦 Loading {len(data['products'])} products...")
                self.publish_status(f"Loading {len(data['products'])} products...")
                product_count = self.load_product_data(db, data['products'])
                self.publish_status(f"Successfully loaded {product_count} products.")
                print(f"✅ Loaded {product_count} products")
            else:
                print("ℹ️  No product data found in JSON file")
                self.publish_status("No product data found in JSON file.")
            # Load areas if present

            # Save database changes
            success_message = f"Database loading complete! Staff: {staff_count}, Products: {product_count}"
            self.publish_status(success_message)
            print(f"🎉 {success_message}\n")

            return True, success_message

        except Exception as e:
            error_message = f"Error during database loading: {str(e)}"
            self.get_logger().error(error_message)
            print(f"💥 {error_message}\n")
            return False, error_message


def main(args=None):
    """
    Main function to run the ROS2 node
    """
    rclpy.init(args=args)

    try:
        node = DatabaseLoaderNode()

        # Keep the node spinning
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nReceived Ctrl+C, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

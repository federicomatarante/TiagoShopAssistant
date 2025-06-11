import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button
from PIL import Image, ImageDraw, ImageFont
from random import randint
from tkinter import simpledialog, Tk, Listbox, Button as TkButton, Toplevel, END, SINGLE, Label, Frame
import datetime
import sys
import os


class MapDisplay:
    """
    A class for displaying and interacting with 2D occupancy grid maps.
    Handles visualization of the grid, named areas, and interactive features.
    """

    def __init__(self, map_instance):
        """
        Initialize the display with the map to be shown.

        :param map_instance: The map object containing occupancy grid and named areas
        """
        self.map = map_instance
        self.rows = len(self.map.occupancy_grid)
        self.cols = len(self.map.occupancy_grid[0]) if self.rows > 0 else 0
        self.grid_array = np.array(self.map.occupancy_grid)

        # Interactive mode state variables
        self.fig = None
        self.ax = None
        self.clicked_point = None
        self.area_vertices = []
        self.current_mode = 'info'  # Default mode: just show info
        self.last_clicked = {"x": None, "y": None}
        self.mode_text = None
        self.buttons = {}  # Store button references

        # Import needed classes here to avoid circular imports
        from src.tiago.src.map.geometry import Point2D
        from src.tiago.src.map.geometry import Polygon
        self.Point2D = Point2D
        self.Polygon = Polygon

    def display(self, interactive=True):
        """
        Displays the map either interactively or non-interactively based on parameter.

        :param interactive: Whether to enable interactive clicking, defaults to True
        """
        # Check if running in PyCharm
        in_pycharm = any('pycharm' in arg.lower() for arg in sys.argv) or 'PYCHARM_HOSTED' in os.environ

        # Force a compatible backend if in PyCharm
        if in_pycharm:
            # Try to use TkAgg backend which is more compatible
            try:
                matplotlib.use('TkAgg', force=True)
            except:
                # Fall back to a non-interactive method if TkAgg fails
                return self.display_noninteractive()

        if interactive:
            try:
                self._setup_interactive_display()
                plt.show()
            except Exception as e:
                print(f"Error displaying interactive plot: {e}")
                plt.close(self.fig)
                return self.display_noninteractive()
        else:
            return self.display_noninteractive()

    def _setup_interactive_display(self):
        """
        Sets up the interactive display with clickable map and action buttons.
        """
        # Create a figure with appropriate size for buttons at bottom
        self.fig = plt.figure(figsize=(self.cols / 4, self.rows / 4 + 1.5))  # Increased height for additional buttons

        # Create main map axis (slightly smaller to make room for buttons)
        self.ax = self.fig.add_axes([0.1, 0.25, 0.8, 0.65])  # [left, bottom, width, height]

        # Draw the base map
        self._draw_map()

        # Add buttons for different actions
        self._setup_buttons()

        # Add label for mode display
        self.mode_text = self.fig.text(0.1, 0.95, "Mode: Info - Click on map for information", fontsize=10)

        # Connect the click event
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)


        plt.tight_layout()

    def _draw_map(self):
        """
        Draws the base map with occupancy grid and named areas.
        """
        # Display occupancy grid
        self.ax.imshow(self.grid_array, cmap='binary', origin='upper',
                       extent=[0, self.cols, self.rows, 0])

        # Add named areas with transparent colors
        for name, area in self.map._named_areas.items():
            # Generate a random semi-transparent fill color
            fill_color = (randint(50, 255) / 255, randint(50, 255) / 255, randint(50, 255) / 255, 0.4)

            # Convert polygon vertices to pixel coordinates
            polygon_points = [(v.x, v.y) for v in area.vertices]
            poly = MplPolygon(polygon_points, closed=True,
                              facecolor=fill_color, edgecolor='blue', alpha=0.6)
            self.ax.add_patch(poly)

            # Add area label at centroid
            centroid_x = sum(p[0] for p in polygon_points) / len(polygon_points)
            centroid_y = sum(p[1] for p in polygon_points) / len(polygon_points)
            self.ax.text(centroid_x, centroid_y, name,
                         horizontalalignment='center', verticalalignment='center',
                         fontsize=10, color='black')

        # Draw all products
        for product_id, (point, _) in self.map.observations._last_seen_products.items():
            self.ax.plot(point.x, point.y, 'go', markersize=8)
            self.ax.text(point.x + 0.5, point.y + 0.5, f"P:{product_id}",
                         color='green', fontsize=8)

        # Draw all staff
        for staff_id, (point, _) in self.map.observations._last_seen_staff.items():
            self.ax.plot(point.x, point.y, 'bo', markersize=8)
            self.ax.text(point.x + 0.5, point.y + 0.5, f"S:{staff_id}",
                         color='blue', fontsize=8)

        # Set axis limits and labels
        self.ax.set_xlim(0, self.cols)
        self.ax.set_ylim(self.rows, 0)  # Invert y-axis to match image coordinates
        self.ax.set_title('Map Display')
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')

    def _setup_buttons(self):
        """
        Creates interactive buttons for map actions.
        """
        # Creation buttons
        add_product_ax = plt.axes([0.1, 0.15, 0.15, 0.05])  # [left, bottom, width, height]
        add_staff_ax = plt.axes([0.3, 0.15, 0.15, 0.05])
        add_area_ax = plt.axes([0.5, 0.15, 0.15, 0.05])

        # List/Manage buttons
        list_products_ax = plt.axes([0.1, 0.07, 0.15, 0.05])
        list_staff_ax = plt.axes([0.3, 0.07, 0.15, 0.05])
        list_areas_ax = plt.axes([0.5, 0.07, 0.15, 0.05])

        # Save button
        save_ax = plt.axes([0.75, 0.07, 0.15, 0.05])

        # Create buttons
        add_product_button = Button(add_product_ax, 'Add Product')
        add_staff_button = Button(add_staff_ax, 'Add Staff')
        add_area_button = Button(add_area_ax, 'Create Area')

        list_products_button = Button(list_products_ax, 'List Products')
        list_staff_button = Button(list_staff_ax, 'List Staff')
        list_areas_button = Button(list_areas_ax, 'List Areas')

        save = Button(save_ax, 'Save')

        # Store references to buttons to prevent garbage collection
        self.buttons = {
            'product': add_product_button,
            'staff': add_staff_button,
            'area': add_area_button,
            'list_products': list_products_button,
            'list_staff': list_staff_button,
            'list_areas': list_areas_button,
            'save': save
        }

        # Assign button actions
        add_product_button.on_clicked(self._add_product)
        add_staff_button.on_clicked(self._add_staff)
        add_area_button.on_clicked(self._create_area)

        list_products_button.on_clicked(self._list_products)
        list_staff_button.on_clicked(self._list_staff)
        list_areas_button.on_clicked(self._list_areas)

        save.on_clicked(self._save_mode)

    def _on_click(self, event):
        """
        Handles click events on the map based on current mode.

        :param event: The mouse click event
        """
        # Check if the click was on the main map axis, not on buttons
        if event.inaxes != self.ax:
            return  # Ignore clicks outside the map area

        if event.xdata is None or event.ydata is None:
            return

        # Calculate cell coordinates (integer grid positions)
        cell_x = int(event.xdata)
        cell_y = int(event.ydata)

        # Save the last clicked point
        self.last_clicked["x"] = cell_x
        self.last_clicked["y"] = cell_y

        # Ensure coordinates are within bounds
        if 0 <= cell_x < self.cols and 0 <= cell_y < self.rows:
            if self.current_mode == 'info':
                self._handle_info_click(cell_x, cell_y, event)
            elif self.current_mode == 'create_area':
                self._handle_area_click(cell_x, cell_y)

    def _handle_info_click(self, cell_x, cell_y, event):
        """
        Handles clicks in info mode - shows information about the clicked point.

        :param cell_x: X cell coordinate
        :param cell_y: Y cell coordinate
        :param event: The original click event with exact coordinates
        """


        # Check if point is in any named area
        point = self.Point2D(cell_x, cell_y)
        area = self.map.get_area(point)



    def _handle_area_click(self, cell_x, cell_y):
        """
        Handles clicks in area creation mode - adds vertices to the new area.

        :param cell_x: X cell coordinate
        :param cell_y: Y cell coordinate
        """
        # Add vertex to area
        point = self.Point2D(cell_x, cell_y)
        self.area_vertices.append(point)

        # Draw the vertex on the map
        self.ax.plot(cell_x, cell_y, 'ro', markersize=5)

        # Draw lines between vertices
        if len(self.area_vertices) > 1:
            for i in range(len(self.area_vertices) - 1):
                self.ax.plot([self.area_vertices[i].x, self.area_vertices[i + 1].x],
                             [self.area_vertices[i].y, self.area_vertices[i + 1].y], 'r-', linewidth=1)

        # If we have at least 3 vertices, connect back to the first
        if len(self.area_vertices) >= 3:
            self.ax.plot([self.area_vertices[-1].x, self.area_vertices[0].x],
                         [self.area_vertices[-1].y, self.area_vertices[0].y], 'r--', linewidth=1)

        self.fig.canvas.draw()

    def _add_product(self, event):
        """
        Adds a product at the last clicked location.

        :param event: The button click event
        """


        # Create a root Tk window
        root = Tk()
        root.withdraw()  # Hide the main window

        # Ask for product ID
        product_id = simpledialog.askstring("Input", "Enter Product ID:", parent=root)
        if product_id and self.last_clicked["x"] and self.last_clicked["y"]:
            # Create product observation at last clicked point
            point = self.Point2D(self.last_clicked["x"], self.last_clicked["y"])
            timestamp = datetime.datetime.now()
            self.map.observations._last_seen_products[product_id] = (point, timestamp)

            # Draw marker on map
            self.ax.plot(self.last_clicked["x"], self.last_clicked["y"], 'go', markersize=8)
            self.ax.text(self.last_clicked["x"] + 0.5, self.last_clicked["y"] + 0.5, f"P:{product_id}",
                         color='green', fontsize=8)
            self.fig.canvas.draw()

        # Destroy the root window
        root.destroy()

    def _add_staff(self, event):
        """
        Adds a staff member at the last clicked location.

        :param event: The button click event
        """

        # Create a root Tk window
        root = Tk()
        root.withdraw()  # Hide the main window

        # Ask for staff ID
        staff_id = simpledialog.askstring("Input", "Enter Staff ID:", parent=root)
        if staff_id and self.last_clicked["x"] and self.last_clicked["y"]:
            # Create staff observation at last clicked point
            point = self.Point2D(self.last_clicked["x"], self.last_clicked["y"])
            timestamp = datetime.datetime.now()
            self.map.observations._last_seen_staff[staff_id] = (point, timestamp)

            # Draw marker on map
            self.ax.plot(self.last_clicked["x"], self.last_clicked["y"], 'bo', markersize=8)
            self.ax.text(self.last_clicked["x"] + 0.5, self.last_clicked["y"] + 0.5, f"S:{staff_id}",
                         color='blue', fontsize=8)
            self.fig.canvas.draw()

        # Destroy the root window
        root.destroy()

    def _create_area(self, event):
        """
        Toggles area creation mode or completes area creation if already in that mode.

        :param event: The button click event
        """
        if self.current_mode == 'create_area':
            # We're already in area creation mode, so this is a finish operation
            if len(self.area_vertices) >= 3:
                # Create a root Tk window
                root = Tk()
                root.withdraw()  # Hide the main window

                # Ask for area name
                area_name = simpledialog.askstring("Input", "Enter Area Name:", parent=root)
                root.destroy()

                if area_name:
                    # Create area with collected vertices
                    try:
                        self.map.add_area(area_name, self.Polygon(self.area_vertices))

                        # Save for next operation
                        self.area_vertices = []

                        # Redraw entire map to show the new area
                        self.ax.clear()
                        self._draw_map()

                        # Switch back to info mode
                        self.current_mode = 'info'
                        self.mode_text.set_text("Mode: Info - Click on map for information")
                        self.fig.canvas.draw()

                    except ValueError as e:
                        print(f"Error creating area: {e}")

        else:
            # Start area creation mode
            self.current_mode = 'create_area'
            self.area_vertices = []
            self.mode_text.set_text("Mode: Creating Area - Click to add vertices, click 'Create Area' again to finish")
            self.fig.canvas.draw()


    def _list_products(self, event):
        """
        Opens a dialog showing all products with options to modify or remove them.

        :param event: The button click event
        """


        # Create dialog window using Tkinter
        root = Tk()
        root.title("Product List")
        root.geometry("400x350")

        # Create frame for content
        frame = Frame(root)
        frame.pack(padx=10, pady=10, fill='both', expand=True)

        # Add title label
        Label(frame, text="Select a product to modify or remove:", font=('Arial', 12)).pack(pady=5)

        # Create listbox with all products
        listbox = Listbox(frame, width=50, height=10, selectmode=SINGLE)
        listbox.pack(pady=10, fill='both')

        # Populate listbox
        product_ids = list(self.map.observations._last_seen_products.keys())
        for product_id in product_ids:
            point, timestamp = self.map.observations._last_seen_products[product_id]
            listbox.insert(END, f"{product_id} - Position: ({point.x}, {point.y}) - Last seen: {timestamp}")

        # Function to update product location
        def update_product():
            if not listbox.curselection():
                return

            index = listbox.curselection()[0]
            product_id = product_ids[index]

            if self.last_clicked["x"] is None or self.last_clicked["y"] is None:
                # Show error message
                error_window = Toplevel(root)
                error_window.title("Error")
                Label(error_window, text="Please click on the map first to select a new location").pack(padx=20,
                                                                                                        pady=20)
                TkButton(error_window, text="OK", command=error_window.destroy).pack(pady=10)
                return

            # Update product location
            point = self.Point2D(self.last_clicked["x"], self.last_clicked["y"])
            timestamp = datetime.datetime.now()
            self.map.observations._last_seen_products[product_id] = (point, timestamp)

            # Redraw map
            self.ax.clear()
            self._draw_map()
            self.fig.canvas.draw()

            root.destroy()

        # Function to remove product
        def remove_product():
            if not listbox.curselection():
                return

            index = listbox.curselection()[0]
            product_id = product_ids[index]

            # Remove product
            del self.map.observations._last_seen_products[product_id]

            # Redraw map
            self.ax.clear()
            self._draw_map()
            self.fig.canvas.draw()

            root.destroy()

        # Add buttons
        button_frame = Frame(frame)
        button_frame.pack(pady=10)

        TkButton(button_frame, text="Update Location", command=update_product).pack(side='left', padx=10)
        TkButton(button_frame, text="Remove", command=remove_product).pack(side='left', padx=10)
        TkButton(button_frame, text="Close", command=root.destroy).pack(side='left', padx=10)

        # Run dialog
        root.mainloop()

    def _list_staff(self, event):
        """
        Opens a dialog showing all staff with options to modify or remove them.

        :param event: The button click event
        """


        # Create dialog window using Tkinter
        root = Tk()
        root.title("Staff List")
        root.geometry("400x350")

        # Create frame for content
        frame = Frame(root)
        frame.pack(padx=10, pady=10, fill='both', expand=True)

        # Add title label
        Label(frame, text="Select a staff member to modify or remove:", font=('Arial', 12)).pack(pady=5)

        # Create listbox with all staff
        listbox = Listbox(frame, width=50, height=10, selectmode=SINGLE)
        listbox.pack(pady=10, fill='both')

        # Populate listbox
        staff_ids = list(self.map.observations._last_seen_staff.keys())
        for staff_id in staff_ids:
            point, timestamp = self.map.observations._last_seen_staff[staff_id]
            listbox.insert(END, f"{staff_id} - Position: ({point.x}, {point.y}) - Last seen: {timestamp}")

        # Function to update staff location
        def update_staff():
            if not listbox.curselection():
                return

            index = listbox.curselection()[0]
            staff_id = staff_ids[index]

            if self.last_clicked["x"] is None or self.last_clicked["y"] is None:
                # Show error message
                error_window = Toplevel(root)
                error_window.title("Error")
                Label(error_window, text="Please click on the map first to select a new location").pack(padx=20,
                                                                                                        pady=20)
                TkButton(error_window, text="OK", command=error_window.destroy).pack(pady=10)
                return

            # Update staff location
            point = self.Point2D(self.last_clicked["x"], self.last_clicked["y"])
            timestamp = datetime.datetime.now()
            self.map.observations._last_seen_staff[staff_id] = (point, timestamp)

            # Redraw map
            self.ax.clear()
            self._draw_map()
            self.fig.canvas.draw()

            root.destroy()

        # Function to remove staff
        def remove_staff():
            if not listbox.curselection():
                return

            index = listbox.curselection()[0]
            staff_id = staff_ids[index]

            # Remove staff
            del self.map.observations._last_seen_staff[staff_id]

            # Redraw map
            self.ax.clear()
            self._draw_map()
            self.fig.canvas.draw()

            root.destroy()

        # Add buttons
        button_frame = Frame(frame)
        button_frame.pack(pady=10)

        TkButton(button_frame, text="Update Location", command=update_staff).pack(side='left', padx=10)
        TkButton(button_frame, text="Remove", command=remove_staff).pack(side='left', padx=10)
        TkButton(button_frame, text="Close", command=root.destroy).pack(side='left', padx=10)

        # Run dialog
        root.mainloop()

    def _list_areas(self, event):
        """
        Opens a dialog showing all areas with options to modify or remove them.

        :param event: The button click event
        """


        # Create dialog window using Tkinter
        root = Tk()
        root.title("Area List")
        root.geometry("400x350")

        # Create frame for content
        frame = Frame(root)
        frame.pack(padx=10, pady=10, fill='both', expand=True)

        # Add title label
        Label(frame, text="Select an area to remove:", font=('Arial', 12)).pack(pady=5)

        # Create listbox with all areas
        listbox = Listbox(frame, width=50, height=10, selectmode=SINGLE)
        listbox.pack(pady=10, fill='both')

        # Populate listbox
        area_names = list(self.map._named_areas.keys())
        for area_name in area_names:
            polygon = self.map._named_areas[area_name]
            num_vertices = len(polygon.vertices)
            listbox.insert(END, f"{area_name} - {num_vertices} vertices")

        # Function to remove area
        def remove_area():
            if not listbox.curselection():
                return

            index = listbox.curselection()[0]
            area_name = area_names[index]

            # Remove area
            del self.map._named_areas[area_name]

            # Redraw map
            self.ax.clear()
            self._draw_map()
            self.fig.canvas.draw()

            root.destroy()

        # Add buttons
        button_frame = Frame(frame)
        button_frame.pack(pady=10)

        TkButton(button_frame, text="Remove", command=remove_area).pack(side='left', padx=10)
        TkButton(button_frame, text="Close", command=root.destroy).pack(side='left', padx=10)

        # Run dialog
        root.mainloop()

    def _save_mode(self, event):
        """
        Saves the current mode to info mode.

        :param event: The button click event
        """
        self.map.save()

    def display_noninteractive(self):
        """
        Fallback method to display the map non-interactively using PIL.
        """
        cell_size = 20

        # Create base image (RGB)
        base_img = Image.new("RGB", (self.cols * cell_size, self.rows * cell_size), "white")
        draw = ImageDraw.Draw(base_img)

        # Draw occupancy grid
        for row in range(self.rows):
            for col in range(self.cols):
                color = (0, 0, 0) if self.map.occupancy_grid[row][col] == 100 else (255, 255, 255)
                draw.rectangle(
                    [col * cell_size, row * cell_size, (col + 1) * cell_size, (row + 1) * cell_size],
                    fill=color,
                )

        # Create transparent overlay image (RGBA)
        overlay = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
        overlay_draw = ImageDraw.Draw(overlay)

        # Draw named areas with transparent color
        for name, area in self.map._named_areas.items():
            # Generate a random semi-transparent fill color
            fill_color = (randint(50, 255), randint(50, 255), randint(50, 255), 100)  # RGBA

            # Convert polygon vertices to pixel coordinates
            polygon_points = [(v.x * cell_size, v.y * cell_size) for v in area.vertices]
            overlay_draw.polygon(polygon_points, fill=fill_color, outline=(0, 0, 255, 200))

            # Add area label at centroid
            try:
                font = ImageFont.truetype("arial.ttf", 12)  # You can change font name and size here
            except IOError:
                font = ImageFont.load_default()
            centroid_x = sum(p[0] for p in polygon_points) // len(polygon_points)
            centroid_y = sum(p[1] for p in polygon_points) // len(polygon_points)
            overlay_draw.text((centroid_x, centroid_y), name, fill=(0, 0, 0, 255), font=font)

        # Composite the overlay on top of the base image
        combined = Image.alpha_composite(base_img.convert("RGBA"), overlay)

        combined.show()


        return combined
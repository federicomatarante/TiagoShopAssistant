from src.tiago.tiago.lib.map.map import Map

map = Map.from_image('map.png')
map.path = 'map.map'
map.display(True)
print("Map loaded and displayed.")
# Sleeping
map.save()
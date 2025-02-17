import carla
import time

import carla
import random
import time


client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

blueprint_library = world.get_blueprint_library()
# Get all vehicle blueprints
vehicle_blueprints = blueprint_library.filter('vehicle.*')

# Get all available spawn points
spawn_points = world.get_map().get_spawn_points()

number_of_vehicles = 10

# Shuffle the spawn points so we get random positions
random.shuffle(spawn_points)

for i in range(number_of_vehicles):
    bp = random.choice(vehicle_blueprints)
    spawn_point = spawn_points[i]
    
    vehicle = world.try_spawn_actor(bp, spawn_point)
    if vehicle:
        vehicle.set_autopilot(True)
        print(f"Spawned vehicle {vehicle.type_id} at {spawn_point.location}")
    else:
        print(f"Failed to spawn vehicle at {spawn_point.location}")

try:
    # Let the simulation run for a while
    time.sleep(30)
finally:
    print("Destroying vehicles...")
    for vehicle in vehicles_list:
        vehicle.destroy()
    print("Done.")




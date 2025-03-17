import carla
import time
import math
import random

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # Set a timeout for the connection

world = client.get_world()

# Get vehicle blueprints
blueprint_library = world.get_blueprint_library()
vehicle_blueprints = blueprint_library.filter("vehicle.*")

# Get spawn points
spawn_points = world.get_map().get_spawn_points()

# Store vehicles
vehicle_list = []

# Spawn vehicles
for i in range(10):
    # Create random vehicles and spawn points from blueprints
    vehicle_bp = random.choice(vehicle_blueprints)
    spawn_point = random.choice(spawn_points)

    # Spawn vehicle in random location
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    # Set it on auto pilot
    if vehicle:
        vehicle.set_autopilot(True)
        vehicle_list.append(vehicle)

# Add 1 emergency vehicle to test function
emergency_vehicle_blueprint = blueprint_library.filter("vehicle.*ambulance*")[0]
emergency_spawn_point = random.choice(spawn_points)

emergency_vehicle = world.try_spawn_actor(emergency_vehicle_blueprint, emergency_spawn_point)

if emergency_vehicle:
    emergency_vehicle.set_autopilot(True)
    vehicle_list.append(emergency_vehicle)

# Get all the traffic light
traffic_lights = world.get_actors().filter('traffic.traffic_light')

# Create distance function in 3d space
def get_distance(dist1, dist2):
    dx = dist1.x - dist2.x
    dy = dist1.y - dist2.y
    dz = dist1.z - dist2.z

    return math.sqrt(dx * dx + dy * dy + dz * dz)

# Function that gets the lights in an intersection
def get_intersection(start_light, start_light_location):
    # Finds distance between each lights
    # Create a new array
    new_array = []
    # add lights to array if distance is less than or equal to 30
    for light in traffic_lights:
        if light.id != start_light.id:
            light_location = light.get_transform().location
            distance = get_distance(start_light_location, light_location)
        
            if distance <= 30:
                new_array.append(light)
    # return array
    return new_array
         
# Manages all traffic lights next to vehicle
def traffic_light_controller(vehicle):
    # Get vehicle waypoint
    vehicle_waypoint = world.get_map().get_waypoint(vehicle.get_location())
    current = None

    # checks for closest traffic light for vehicle
    for light in traffic_lights:
        waypoints = light.get_stop_waypoints()
        for w in waypoints:
            if w.road_id == vehicle_waypoint.road_id and w.lane_id == vehicle_waypoint.lane_id:
                current = light
                break
        if current:
            break

    # If traffic light was found for vehicle
    if current:
        # Call intersection function with current light to get the lights nearby
        current_location = current.get_transform().location
        intersection = get_intersection(current, current_location)

        print(f"Ambulance {vehicle.id} approaching. Traffic light {current.id} set to GREEN.")

        # Set closest traffic light to green for 15 seconds
        current.set_state(carla.TrafficLightState.Green)
        current.freeze(True)

        # Make other traffic lights in intersection red for 15 seconds
        for other_light in intersection:
            other_light.set_state(carla.TrafficLightState.Red)
            other_light.freeze(True)
            print(f"Traffic light {other_light.id} set to RED to prioritize ambulance.")

        time.sleep(15)

        # Put them back to normal after
        current.freeze(False)
        for other_light in intersection:
            other_light.freeze(False)

        print(f"Traffic light {current.id} set back to normal. Normal operation resumed.")

# Destroy vehicles when done with simulation
try:
    while True:
        # List in case there's multiple emergency vehicles
        all_vehicles = world.get_actors().filter("vehicle.*")
        for vehicle in all_vehicles:
            if "ambulance" in vehicle.type_id.lower():
                traffic_light_controller(vehicle)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nKeyboardInterrupt caught, stopping simulation...")
finally:
    for vehicle in vehicle_list:
        if vehicle.is_alive:
            vehicle.destroy()
    print("All vehicles destroyed.")

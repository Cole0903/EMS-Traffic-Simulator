import carla
import time
import random
import math

def get_distance(location1, location2):
    """Calculate Euclidean distance between two CARLA locations."""
    return math.sqrt((location1.x - location2.x) ** 2 + (location1.y - location2.y) ** 2)

def detect_pedestrians(vehicle, pedestrians, detection_range=10.0):
    """Check if there are pedestrians within a given detection range in front of the vehicle."""
    vehicle_location = vehicle.get_location()
    vehicle_forward_vector = vehicle.get_transform().get_forward_vector()

    for pedestrian in pedestrians:
        ped_location = pedestrian.get_location()
        distance = get_distance(vehicle_location, ped_location)

        # Check if the pedestrian is within range and in front of the vehicle
        relative_vector = carla.Location(
            x=ped_location.x - vehicle_location.x,
            y=ped_location.y - vehicle_location.y
        )
        dot_product = relative_vector.x * vehicle_forward_vector.x + relative_vector.y * vehicle_forward_vector.y
        
        if distance < detection_range and dot_product > 0:
            return True  # Pedestrian detected in front of the vehicle

    return False


client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

blueprint_library = world.get_blueprint_library()


vehicle_blueprints = blueprint_library.filter('vehicle.*')

spawn_points = world.get_map().get_spawn_points()

number_of_vehicles = 10


random.shuffle(spawn_points)
vehicles_list = []

for i in range(number_of_vehicles):
    bp = random.choice(vehicle_blueprints)
    spawn_point = spawn_points[i]
    
    vehicle = world.try_spawn_actor(bp, spawn_point)
    if vehicle:
        vehicle.set_autopilot(False)  
        vehicles_list.append(vehicle)
        print(f"Spawned vehicle {vehicle.type_id} at {spawn_point.location}")
    else:
        print(f"Failed to spawn vehicle at {spawn_point.location}")


pedestrians = world.get_actors().filter('walker.pedestrian.*')

try:
    for _ in range(600):  
        for vehicle in vehicles_list:
            if detect_pedestrians(vehicle, pedestrians):
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))  
                print(f"Vehicle {vehicle.id} stopped for pedestrian")
            else:
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, brake=0.0))  
        
        time.sleep(0.05)  
finally:
    print("Destroying vehicles...")
    for vehicle in vehicles_list:
        vehicle.destroy()
    print("Done.")
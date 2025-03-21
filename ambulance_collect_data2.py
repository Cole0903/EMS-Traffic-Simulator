import glob
import os
import sys
import carla
import random
import time
import cv2
import math
import numpy as np
import csv

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent

frame_count = 0
dataset_path = "D:/dataset/"
os.makedirs(dataset_path, exist_ok=True)

# log data in csv
csv_file = open(os.path.join(dataset_path, "controls.csv"), mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["frame", "steering", "throttle", "brake"])

max_frames = 50000  # Stop at 50,000 frames

def process_image(image):
    """ Process and save image from camera """
    global frame_count
    image_data = np.frombuffer(image.raw_data, dtype=np.uint8)
    image_data = image_data.reshape((image.height, image.width, 4))[:, :, :3]  # Convert BGRA to BGR
    image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB) #convert to rgb
    img_filename = os.path.join(dataset_path, f"{frame_count}.jpg")
    cv2.imwrite(img_filename, image_data)  # Save image
    frame_count += 1


def get_clear_spawn_point(world):
    """ Get a spawn point that is clear of obstacles """
    spawn_points = world.get_map().get_spawn_points()
    for _ in range(10):
        spawn_point = random.choice(spawn_points)
        return spawn_point
    return random.choice(spawn_points)


def main():
    global frame_count
    actor_list = []
    try:
        
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
    
        
        # !!! needs synchronous mode to work, 20fps (0.05 timestep) so launch carla with -fps 20 in commandline!!!
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        world.apply_settings(settings)
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_respawn_dormant_vehicles(True)

        blueprint_library = world.get_blueprint_library()

        # spawn ambulance
        vehicle_bp = blueprint_library.find('vehicle.ambulance.ford')
        spawn_point = get_clear_spawn_point(world)
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)

        agent = BehaviorAgent(vehicle, behavior="normal")
        agent._look_ahead_steps = 5 

        # attach rgb camera (224x224)
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '224')
        camera_bp.set_attribute('image_size_y', '224')
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(camera)

        # start recording
        camera.listen(lambda image: process_image(image))
        
        print("Ambulance spawned. Driving in a loop!")

        # Run sim
        # try to implement counter to detect if vehicle stuck and teleport it later (wip)
        # last_position = vehicle.get_transform().location
        # stuck_count = 0
        
        # pick random waypoint as destination
        def get_random_destination():
            spawn_points = world.get_map().get_spawn_points()
            return random.choice(spawn_points).location
        
        destination = get_random_destination()
        agent.set_destination(destination)
        print(f"Initial destination set to: {destination}")
        
        while frame_count < max_frames:
            world.tick()
            if vehicle.is_at_traffic_light():
                traffic_light = vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    traffic_light.set_state(carla.TrafficLightState.Green)
                    traffic_light.set_green_time(4.0)
            if agent.done():
                # makes ambulance loop infinitely by finding a new destination once the current one is reached
                print("Destination reached. Setting new destination.")
                destination = get_random_destination()
                agent.set_destination(destination)
                print(f"New destination set to: {destination}")

            control = agent.run_step()
            vehicle.apply_control(control)

            # record controls data to csv
            csv_writer.writerow([frame_count, control.steer, control.throttle, control.brake])
            
            '''
                current_position = vehicle.get_transform().location
                if current_position.distance(last_position) < 0.1:  # Stuck check
                    stuck_count += 1
                else:
                    stuck_count = 0  # Reset if moving

                last_position = current_position

                # **Reset vehicle if stuck for too long**
                if stuck_count > 40:
                    print("Vehicle is stuck! Resetting position...")
                    vehicle.set_transform(get_clear_spawn_point(world))
                    stuck_count = 0
            '''

    except KeyboardInterrupt:
        print("\nStopping simulation...")

    finally:
        print("Destroying actors...")
        for actor in actor_list:
            actor.destroy()
        csv_file.close()
        world.apply_settings(settings)  # Reset to asynchronous mode

if __name__ == "__main__":
    main()
import sys
import os
import carla
import pygame
from pygame.locals import *
import numpy as np
import torch
import cv2
from ultralytics import YOLO
import csv

# Yolov12 provided by Ultralytics
'''
cff-version: 1.2.0
title: Ultralytics YOLO
message: >-
  If you use this software, please cite it using the
  metadata from this file.
type: software
authors:
  - given-names: Glenn
    family-names: Jocher
    affiliation: Ultralytics
    orcid: 'https://orcid.org/0000-0001-5950-6979'
  - family-names: Qiu
    given-names: Jing
    affiliation: Ultralytics
    orcid: 'https://orcid.org/0000-0003-3783-7069'
  - given-names: Ayush
    family-names: Chaurasia
    affiliation: Ultralytics
    orcid: 'https://orcid.org/0000-0002-7603-6750'
repository-code: 'https://github.com/ultralytics/ultralytics'
url: 'https://ultralytics.com'
license: AGPL-3.0
version: 8.0.0
date-released: '2023-01-10'
'''

carla_path = os.environ.get("CARLA_PYTHON_PATH")
if not carla_path:
    raise RuntimeError(
        "CARLA_PYTHON_PATH environment variable not set!\n"
        "Please set it to your Carla PythonAPI path (contains the 'carla' package).\n"
        "Example (Linux): export CARLA_PYTHON_PATH=~/carla/PythonAPI/carla\n"
        "Example (Windows): $env:CARLA_PYTHON_PATH='C:\\Carla\\PythonAPI\\carla'"
    )
sys.path.append(carla_path)

# load yolov12 medium classification model
model = YOLO("yolo12m.pt")
model.eval()

# set up pygame window to display inference
pygame.init()
screen = pygame.display.set_mode((640, 600))
pygame.display.set_caption("YOLOv12m object detection")

def main():
    global frame_count
    frame_count = 0
    
    csv_file = open('detection_data.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
    'frame_number', 'timestamp', 'class', 'confidence', 
    'x1', 'y1', 'x2', 'y2', 'speed_kmh', 'location_x', 
    'location_y', 'location_z', 'velocity_x', 'velocity_y', 
    'velocity_z', 'resolution'  # 16 columns total
])
    
    client = carla.Client('192.168.1.124', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(0)
    # set sync mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05 #20 fps
    world.apply_settings(settings)

    # spawn ambulance
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.ambulance.ford')
    spawn_point = world.get_map().get_spawn_points()[0]
    print(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True, tm.get_port())

    # set up rgb camera sensor
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '320')
    camera_bp.set_attribute('image_size_y', '320')

    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(carla.Location(x=1.5, z=2.0)),
        attach_to=vehicle
    )
    
    latest_surface = None

    def camera_callback(image):
        global frame_count, current_resolution
        nonlocal latest_surface
        current_resolution = 320
        if frame_count % 4 == 0:
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))[:, :, :3]
            img = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)

            results = list(model(img, imgsz=current_resolution, stream=True, conf=0.5))
            
            if results:
                result = results[0]
                vehicle_location = vehicle.get_transform().location
                vehicle_velocity = vehicle.get_velocity()
                speed_kmh = 3.6 * vehicle_velocity.length()

                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = box.conf.item()
                    cls = box.cls.item()
                    label = result.names[int(cls)]
                    
                    csv_writer.writerow([
                        image.frame,
                        image.timestamp,
                        label,
                        conf,
                        x1,  
                        y1,  
                        x2,  
                        y2,  
                        speed_kmh,
                        vehicle_location.x,
                        vehicle_location.y,
                        vehicle_location.z,
                        vehicle_velocity.x,
                        vehicle_velocity.y,
                        vehicle_velocity.z,
                        current_resolution
                    ])

                rendered_img = result.plot()
                
                rendered_img = cv2.cvtColor(rendered_img, cv2.COLOR_BGR2RGB)
                rendered_img = cv2.resize(rendered_img, (640, 600))
                
                # update pygame surface
                surface = pygame.surfarray.make_surface(rendered_img.swapaxes(0, 1))
                latest_surface = surface

        frame_count += 1

    camera.listen(camera_callback)

    try:
        clock = pygame.time.Clock()
        while True:
            # carla tick world by 1 frame
            world.tick()
            clock.tick(20)

            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    raise KeyboardInterrupt

            # update pygame display every frame
            if latest_surface:
                screen.blit(latest_surface, (0, 0))
                pygame.display.flip()

    finally:
        #  destroy objects
        csv_file.close()
        camera.destroy()
        vehicle.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        pygame.quit()

if __name__ == '__main__':
    main()

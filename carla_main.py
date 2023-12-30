import glob
import os
import sys
import carla_config
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import rosbridge

actor_list = []
roscom = rosbridge.RosCom()

class CarlaBridge:
    def __init__(self):
        
        try:
            print('Initialising Carla Setup')
            client = carla.Client('localhost', 2000)
            client.set_timeout(2.0)
            world = client.get_world()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0/FPS  # FPS = 1/0.05 = 20
            world.apply_settings(settings)
            world.tick()
            self.world = world
            time.sleep(1)
        except :
            sys.exit("Failed to Intialise the world")    

        #Initializing the car
        try:
            blueprint_library = world.get_blueprint_library()
            carbp = blueprint_library.find('vehicle.tesla.model3')
            print(carbp)
            spawn_point = random.choice(world.get_map().get_spawn_points())
            vehicle = world.spawn_actor(carbp, spawn_point)
            # vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.
            self.vehicle = vehicle

            actor_list.append(vehicle)
           
        
        except: 
            print("Failed to Init Car")

        
        try: 
             # VLP-16 LiDAR
            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('channels', str(16))
            # Set the fps of simulator same as this
            lidar_bp.set_attribute('rotation_frequency', str(FPS))
            lidar_bp.set_attribute('range', str(LIDAR_RANGE))
            lidar_bp.set_attribute('lower_fov', str(-15))
            lidar_bp.set_attribute('upper_fov', str(15))
            lidar_bp.set_attribute('points_per_second', str(300000))
            # lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
            lidar_location = carla.Location(0, 0, 1.75)
            lidar_rotation = carla.Rotation(0, 0, 0)
            lidar_transform = carla.Transform(lidar_location, lidar_rotation)
            lidar_sen = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego)
            lidar_sen.listen(
                lambda point_cloud: self.process_point_cloud(point_cloud))
            self.lidar_sen = lidar_sen
            print("finished Carla setup")
        except:
            print("Failed to Initialise the Lidar")
            pass

    def setup_ticks(self):
        for i in range(20):
            self.world.tick()
            # self.spectator.set_transform(self.dummy.get_transform())
            # self.ego.apply_control(carla.VehicleControl(
            #     throttle=0, steer=0, brake=1))

            # # Clearing Brake Control | This is Important
            # self.ego.apply_control(carla.VehicleControl(
            #     throttle=0, steer=0, brake=0))
   

    def process_point_cloud(self, point_cloud_carla):
        pcd = np.copy(np.frombuffer(point_cloud_carla.raw_data,
                                    dtype=np.dtype("f4, f4, f4, f4, u4, u4")))
        pcd = np.array(pcd.tolist())

        # The 4th column is considered as intensity in ros, hence making it one
        pcd[:, 3] = 1
        # Flipping Y  | Carla works in in LHCS
        pcd[:, 1] = -pcd[:, 1]

        pcd_xyz = pcd[:, :3]
        
        pcd_sem = pcd[:, 5].reshape(-1, 1)    # Semantic Information | Might be helpful later
        pcd_intensity = pcd[:, 4].reshape(-1, 1)

        roscom.publish_points(pcd_xyz)
    
    def run(self):
        while not rospy.is_shutdown():
            
            self.world.tick()

            time.sleep(0.02)




def main():
    try:
        carlaBridge = CarlaBridge()
        carlaBridge.run()

    finally:
            print('destroying actors')
            for actor in actor_list:
                actor.destroy()
            print('done.')


if __name__ == '__main__':
    main()    
    


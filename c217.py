import glob
import os
import sys
import time
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []


def generate_radar_blueprint(blueprint_library):
    radar_blueprint = blueprint_library.filter('sensor.other.radar')[0]
    radar_blueprint.set_attribute('horizontal_fov', str(35))
    radar_blueprint.set_attribute('vertical_fov', str(20))
    radar_blueprint.set_attribute('points_per_second', str(1500))
    radar_blueprint.set_attribute('range', str(20))
    return radar_blueprint


try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    get_blueprint_of_world = world.get_blueprint_library()
    car_model = get_blueprint_of_world.filter('model3')[0]
    spawn_point = (world.get_map().get_spawn_points()[1])
    dropped_vehicle = world.spawn_actor(car_model, spawn_point)

    simulator_camera_location_rotation = carla.Transform(spawn_point.location, spawn_point.rotation)
    simulator_camera_location_rotation.location += spawn_point.get_forward_vector() * 30
    simulator_camera_location_rotation.rotation.yaw += 180
    simulator_camera_view = world.get_spectator()
    simulator_camera_view.set_transform(simulator_camera_location_rotation)
    actor_list.append(dropped_vehicle)

    radar_sensor = generate_radar_blueprint(get_blueprint_of_world)
    sensor_radar_spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.8))
    sensor = world.spawn_actor(radar_sensor, sensor_radar_spawn_point, attach_to=dropped_vehicle)

    sensor.listen(lambda radar_data: _Radar_callback(radar_data))


    def _Radar_callback(radar_data):
        #write the code for radar sensor rotation
        radar_current_rotation = radar_data.transform.rotation
        radar_velocity_range = 7.5
        #write the code for radar sensor velocity
        debug = world.debug
        for detect in radar_data:
            radar_azimuth = math.degrees(detect.azimuth)
            radar_altitude = math.degrees(detect.altitude)

            forward_view = carla.Vector3D(x=detect.depth - 0.25)
            #write the code for yaw, pitch and roll
            new_pitch = radar_current_rotation.pitch + radar_azimuth
            new_yaw = radar_current_rotation.yaw + radar_altitude

            #write the code for carla.transform
            new_roll = radar_current_rotation.roll

            carla.Transform(carla.Location(), carla.rotation(
                pitch=new_pitch,
                yaw=new_yaw,
                roll=new_roll)).transform(forward_view)




            #write the code for draw points
            def clamp(min_v, max_v, value):
                returm max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / radar_velocity_range
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            debug.draw_point(
                radar_data.transform.location + forward_view,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g , b))



    dropped_vehicle.apply_control(carla.VehicleControl(throttle=0.5))
    time.sleep(20)
    actor_list.append(sensor)

    time.sleep(1000)
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')

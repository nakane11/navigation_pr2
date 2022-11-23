#!/usr/bin/env python

import rospy
import roslaunch
import os

class MapManager(object):
    
    def __init__(self):
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        rospy.on_shutdown(self.handler)
        self.process_gmapping = None
        self.process_saver = None
        self.process_server = None        
        self.process_tf_to_map = None
        self.process_tf_from_world = None
        self.current_floor = None

    def handler(self):
        self.launch.stop()


    def start_make_map(self, floor):
        self.stop_map_server()
        self.start_tf_publisher(floor)        
        self.start_gmapping(floor)
        self.current_floor = floor        

    def change_make_map(self, floor):
        self.start_map_saver(self.current_floor)
        while not os.path.exists('/tmp/raw_maps/{}.yaml'.format(self.current_floor)):
            continue
        rospy.loginfo('Successfully saved {}.yaml!'.format(self.current_floor))
        self.stop_map_saver()     
        self.stop_gmapping()
        self.stop_tf_publisher()
        self.start_tf_publisher(floor)
        self.start_gmapping(floor)
        self.current_floor = floor

    def stop_make_map(self):
        self.start_map_saver(self.current_floor)
        while not os.path.exists('/tmp/raw_maps/{}.yaml'.format(self.current_floor)):
            continue
        rospy.loginfo('Successfully saved {}.yaml!'.format(self.current_floor))
        self.stop_map_saver() 
        self.stop_gmapping()
        self.start_map_server(self.current_floor)

    def change_floor(self, floor):
        self.stop_map_server()
        self.start_map_server(floor)
        self.current_floor = floor


    # make /map/6f of /6f frame
    def start_gmapping(self, floor):
        package = 'gmapping'
        executable = 'slam_gmapping'
        name = 'slam_gmapping_{}'.format(floor)
        args='_odom_frame:=odom_combined _map_frame:={}'.format(floor)
        remap_args = [('scan', 'base_scan'), ('map', 'map/{}'.format(floor))]
        gmapping = roslaunch.core.Node(package, executable, args=args, remap_args=remap_args)
        self.process_gmapping = self.launch.launch(gmapping)
        print self.process_gmapping.is_alive()

    def stop_gmapping(self):
        if self.process_gmapping is not None:
            self.process_gmapping.stop()
            print self.process_gmapping.is_alive()

    # save /map/6f as 6f.yaml
    def start_map_saver(self, floor):
        package = 'map_server'
        executable = 'map_saver'
        name = 'map_saver_{}'.format(floor)
        args='-f /tmp/raw_maps/{}'.format(floor)
        remap_args = [('map', 'map/{}'.format(floor))]
        saver = roslaunch.core.Node(package, executable, name=name, args=args, remap_args=remap_args)
        self.process_saver = self.launch.launch(saver)
        print self.process_saver.is_alive()

    def stop_map_saver(self):
        if self.process_saver is not None:
            self.process_saver.stop()
            print self.process_saver.is_alive()

    # serve /map topic of /map frame from 6f.yaml
    def start_map_server(self, floor):
        package = 'map_server'
        executable = 'map_server'
        name = 'map_server_{}'.format(floor)
        args='/tmp/raw_maps/{}.yaml'.format(floor)
        server = roslaunch.core.Node(package, executable, name=name, args=args)
        self.process_server = self.launch.launch(server)
        print self.process_server.is_alive()

    def stop_map_server(self):
        if self.process_server is not None:
            self.process_server.stop()

    # publish /world->/6f and /6f->/map tf
    def start_tf_publisher(self, floor):
        package = 'tf'
        executable = 'static_transform_publisher'

        name_from_world = "world_to_{}".format(floor)
        args_from_world= "0.000   0.000   0.000 0 0 0 /world /{} 100".format(floor)
        tf_from_world = roslaunch.core.Node(package, executable, name=name_from_world, args=args_from_world)
        self.process_tf_from_world = self.launch.launch(tf_from_world)
        print self.process_tf_from_world.is_alive()
        
        name_to_map = "floor_{}_to_map".format(floor)
        args_to_map= "0.000   0.000   0.000 0 0 0 /{} /map 100".format(floor)
        tf_to_map = roslaunch.core.Node(package, executable, name=name_to_map, args=args_to_map)
        self.process_tf_to_map = self.launch.launch(tf_to_map)
        print self.process_tf_to_map.is_alive()

    def stop_tf_publisher(self):
        if self.process_tf_from_world is not None:
            self.process_tf_from_world.stop()
            print self.process_tf_from_world.is_alive()

        if self.process_tf_to_map is not None:
            self.process_tf_to_map.stop()
            print self.process_tf_to_map.is_alive()

if __name__ == '__main__':
    rospy.init_node('map_manager')
    MapManager()
    rospy.spin()




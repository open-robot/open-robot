#!/usr/bin/env python

PKG = 'map_store'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import map_store.srv

default_map = None
try:
    default_map = sys.argv[1]
except:
    print "No map name received, use the last one"

print "Waiting for /list_maps..."
rospy.wait_for_service('/list_maps')
print "Waiting for /delete_map..."
rospy.wait_for_service('/publish_map')

print "Search for maps...."
list_last_maps = rospy.ServiceProxy('/list_maps', map_store.srv.ListMaps)
publish_map = rospy.ServiceProxy('/publish_map', map_store.srv.PublishMap)
maps = []
try:
    maps = list_last_maps().map_list
except:
    print "Getting maps from the map_manager has failed"
    sys.exit(2)

print maps
if default_map:
    for i in maps:
        if (i.name == default_map):
            print "Publish map", i.map_id
            publish_map(i.map_id)
else:
    for i in maps:
        if i.name:
            print "Publish map", i.map_id
            publish_map(i.map_id)
            sys.exit(0)

print "Done"

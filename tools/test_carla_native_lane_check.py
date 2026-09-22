#!/usr/bin/env python3
"""Offline Town05 query regressions; no server connection or world ticks."""
from pathlib import Path
import unittest

import carla
from carla_native_lane_check import query_driving_lane

XODR = Path('/home/a/CARLA/0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr')
FLAGS = [
    (208.74429321289062,61.494632720947266,.0360981747508049),
    (208.83665466308594,52.69392395019531,.036086346954107285),
    (208.733642578125,60.77069854736328,.0360982),
    (209.27171325683594,4.973637104034424,.0361125),
]


class WaypointView:
    """Read-only proxy used to inject invalid adjacency metadata in tests."""
    def __init__(self, source, **overrides):
        self.source,self.overrides=source,overrides
    def __getattr__(self,key):
        return self.overrides[key] if key in self.overrides else getattr(self.source,key)


class QueryView:
    def __init__(self, native, projected):
        self.native,self.projected=native,projected
    def get_waypoint(self, position, project_to_road=True, **kwargs):
        return self.projected if project_to_road else None
    def get_waypoint_xodr(self,*args):
        return self.native.get_waypoint_xodr(*args)


@unittest.skipUnless(XODR.is_file(),'Native Town05 XODR required')
class NativeLaneQueryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native=carla.Map('Town05',XODR.read_text())

    def test_recorded_false_negatives_are_inside_exact_native_lane_widths(self):
        for xyz, expected in zip(FLAGS,(-3,-2,-2,-2)):
            with self.subTest(xyz=xyz):
                p=carla.Location(*xyz)
                self.assertIsNone(self.native.get_waypoint(p,project_to_road=False,lane_type=carla.LaneType.Driving))
                wp,evidence=query_driving_lane(self.native,p)
                self.assertIsNotNone(wp)
                self.assertEqual(wp.lane_id,expected)
                self.assertLessEqual(abs(evidence['selected']['across_m']),wp.lane_width/2+1e-6)
                self.assertLess(abs(evidence['selected']['along_m']),.05)

    def test_direct_native_hit_is_preserved_without_fallback(self):
        wp=self.native.get_waypoint_xodr(38,-3,250)
        got,evidence=query_driving_lane(self.native,wp.transform.location)
        self.assertEqual((got.road_id,got.lane_id),(38,-3))
        self.assertIsNone(evidence)

    def test_road_exterior_is_not_projected_onto_road(self):
        lane=self.native.get_waypoint_xodr(38,-3,253.54)
        t=lane.transform;r=t.get_right_vector()
        p=t.location+carla.Location(x=r.x*(lane.lane_width/2+.02),y=r.y*(lane.lane_width/2+.02),z=.036)
        wp,evidence=query_driving_lane(self.native,p)
        self.assertIsNone(wp);self.assertIsNone(evidence)

    def test_height_and_large_longitudinal_offsets_are_rejected(self):
        p=carla.Location(*FLAGS[2]);wp=self.native.get_waypoint(p)
        for delta in (carla.Location(z=1),carla.Location(y=2)):
            with self.subTest(delta=delta):
                self.assertIsNone(query_driving_lane(QueryView(self.native,wp),p+delta)[0])

    def test_opposite_direction_nonadjacent_and_disconnected_neighbor_rejected(self):
        p=carla.Location(*FLAGS[2]);wp=self.native.get_waypoint(p);neighbor=wp.get_left_lane()
        for changes in ({'lane_id':2},{'lane_id':-1},{'road_id':99999},{'section_id':99},
                        {'lane_type':carla.LaneType.Shoulder},{'is_junction':True}):
            with self.subTest(changes=changes):
                fake=WaypointView(neighbor,**changes)
                source=WaypointView(wp,get_left_lane=lambda:fake)
                self.assertIsNone(query_driving_lane(QueryView(self.native,source),p)[0])

    def test_junction_projection_has_no_fallback(self):
        p=carla.Location(*FLAGS[2]);wp=self.native.get_waypoint(p)
        self.assertIsNone(query_driving_lane(QueryView(self.native,WaypointView(wp,is_junction=True)),p)[0])

    def test_non_touching_neighbor_edges_rejected(self):
        p=carla.Location(*FLAGS[2]);wp=self.native.get_waypoint(p);neighbor=wp.get_left_lane()
        tf=neighbor.transform;tf.location.x-=.2
        fake=WaypointView(neighbor,transform=tf)
        source=WaypointView(wp,get_left_lane=lambda:fake)
        self.assertIsNone(query_driving_lane(QueryView(self.native,source),p)[0])

    def test_interior_point_cannot_use_seam_fallback(self):
        wp=self.native.get_waypoint_xodr(38,-3,253.54)
        self.assertIsNone(query_driving_lane(QueryView(self.native,wp),wp.transform.location)[0])


if __name__=='__main__':unittest.main()

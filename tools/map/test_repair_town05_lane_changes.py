#!/usr/bin/env python3
"""Safety properties for offline native lane-change tagging.

Run with the CARLA virtualenv Python. Integration cases use installed Town05
assets when available and never write to them or connect to the simulator.
"""
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from repair_town05_lane_changes import (
    OWNED_TAGS, constant_broken_both, native_marking_lane, repair,
    same_direction_adjacent, sha, tags,
)

MAP = Path('/home/a/autoware_data/maps/Town05/lanelet2_map.osm')
XODR = Path('/home/a/CARLA/0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr')
MANIFEST = MAP.with_name('carla_traffic_signals.json')


class PermissionTests(unittest.TestCase):
    def test_dashed_does_not_override_explicit_prohibition(self):
        self.assertFalse(constant_broken_both(ET.fromstring(
            '<lane><roadMark sOffset="0" type="broken" laneChange="none"/></lane>')))

    def test_short_solid_segment_not_missed_by_sampling(self):
        self.assertFalse(constant_broken_both(ET.fromstring('''<lane>
          <roadMark sOffset="0" type="broken" laneChange="both"/>
          <roadMark sOffset="12.0001" type="solid" laneChange="none"/>
          <roadMark sOffset="12.0002" type="broken" laneChange="both"/>
        </lane>''')))

    def test_asymmetric_unknown_and_missing_permissions_denied(self):
        for mark in ('increase', 'decrease', 'unknown', ''):
            self.assertFalse(constant_broken_both(ET.fromstring(
                f'<lane><roadMark sOffset="0" type="broken" laneChange="{mark}"/></lane>')))
        self.assertFalse(constant_broken_both(ET.fromstring('<lane/>')))

    def test_opposite_direction_and_disconnected_lanes_denied(self):
        self.assertTrue(same_direction_adjacent((38,0,-3), (38,0,-2)))
        self.assertTrue(same_direction_adjacent((38,0,3), (38,0,2)))
        for other in ((38,0,2), (38,0,-1), (39,0,-2), (38,1,-2)):
            self.assertFalse(same_direction_adjacent((38,0,-3), other))

    def test_shared_marking_belongs_to_inner_lane_on_both_sides(self):
        section = ET.fromstring('''<laneSection><left><lane id="2"/><lane id="3"/></left>
          <right><lane id="-2"/><lane id="-3"/></right></laneSection>''')
        self.assertEqual(native_marking_lane(section, -3, -2).get('id'), '-2')
        self.assertEqual(native_marking_lane(section, 3, 2).get('id'), '2')


@unittest.skipUnless(MAP.exists() and XODR.exists() and MANIFEST.exists(), 'Installed Town05 assets required')
class MapIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.work = Path(self.tmp.name)
        # Test initial missing-boundary state even when production has already
        # installed the repair. Remove ONLY this helper's generated tags.
        tree = ET.parse(MAP)
        for way in tree.getroot().findall('way'):
            if tags(way).get('carla:lane_marking_source') == 'carla_town05_lane_change_v1':
                for child in list(way.findall('tag')):
                    if child.get('k') in OWNED_TAGS:
                        way.remove(child)
        self.source = self.work/'source.osm'
        tree.write(self.source)

    def tearDown(self):
        self.tmp.cleanup()

    def run_repair(self, source=None, stem='candidate'):
        out, manifest = self.work/(stem+'.osm'), self.work/(stem+'.json')
        result = repair(source or self.source, XODR, out, MANIFEST, manifest)
        return out, manifest, result

    def test_changes_only_proven_tags_preserves_geometry_regulations_and_speed(self):
        tree = ET.parse(self.source)
        lane = next(r for r in tree.getroot().findall('relation') if r.get('id')=='18052')
        ET.SubElement(lane,'tag', k='speed_limit', v='30')
        tree.write(self.source)
        before = ET.parse(self.source).getroot()
        out, manifest, result = self.run_repair()
        after = ET.parse(out).getroot()
        self.assertEqual(len(before), len(after))
        for a, b in zip(before, after):
            self.assertEqual(a.tag, b.tag)
            self.assertEqual(a.attrib, b.attrib)
            if a.tag != 'way':
                self.assertEqual(ET.tostring(a), ET.tostring(b))
            else:
                self.assertEqual([n.attrib for n in a.findall('nd')], [n.attrib for n in b.findall('nd')])
                self.assertTrue(tags(a).items() <= tags(b).items())
                additions = {k:v for k,v in tags(b).items() if k not in tags(a)}
                self.assertIn(additions, ({}, OWNED_TAGS))
        boundary = next(r for r in result['boundaries'] if r['way_id']==18050)
        self.assertTrue(boundary['allowed'])
        self.assertEqual(json.loads(manifest.read_text())['map_sha256'], sha(out))
        self.assertLess(result['allowed_boundaries'], result['road_boundaries'])
        out2, _, again = self.run_repair(out, 'again')
        self.assertEqual(sha(out), sha(out2))
        self.assertEqual(again['added_boundaries'], 0)

    def test_existing_user_prohibition_preserved(self):
        tree = ET.parse(self.source)
        way = next(w for w in tree.getroot().findall('way') if w.get('id')=='18050')
        ET.SubElement(way, 'tag', k='lane_change', v='no')
        tree.write(self.source)
        out, _, result = self.run_repair()
        reason = next(r for r in result['boundaries'] if r['way_id']==18050)
        self.assertFalse(reason['allowed'])
        actual = next(w for w in ET.parse(out).getroot().findall('way') if w.get('id')=='18050')
        self.assertEqual(tags(actual)['lane_change'], 'no')


if __name__ == '__main__':
    unittest.main()

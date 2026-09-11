"""Selection and exact-file-change checks; never connects to a running vehicle."""
import tempfile
import unittest
from pathlib import Path

from save_goal_speed import make_index, patch_bytes, select_roads


def lane(rid, y, reverse=False, x0=0, x1=20):
    left = [(x0, y + 3), (x1, y + 3)]
    right = [(x0, y), (x1, y)]
    if reverse:
        left, right = right[::-1], left[::-1]
    return dict(id=rid, left=left, right=right)


class GoalSpeedTests(unittest.TestCase):
    def test_both_directions_and_all_touching_lanes(self):
        roads = [lane(i + 1, i * 3, reverse=i >= 3) for i in range(6)]
        ids, info = select_roads({'roads': roads}, {'x': 10, 'y': 1})
        self.assertEqual(ids, [1, 2, 3, 4, 5, 6])
        self.assertEqual(info['opposite_direction_lanelet_ids'], [4, 5, 6])

    def test_separate_road_and_longitudinal_successor_excluded(self):
        roads = [lane(1, 0), lane(2, 3), lane(3, 9), lane(4, 0, x0=20, x1=40)]
        ids, _ = select_roads({'roads': roads}, {'x': 10, 'y': 1})
        self.assertEqual(ids, [1, 2])

    def test_crossing_street_not_followed(self):
        roads = [lane(1, 0), lane(2, 3)]
        roads.append(dict(id=3, left=[(9, 8), (9, 30)], right=[(12, 8), (12, 30)]))
        ids, _ = select_roads({'roads': roads}, {'x': 10, 'y': 1})
        self.assertEqual(ids, [1, 2])

    def test_goal_on_shared_boundary_selects_both_directions(self):
        ids, _ = select_roads({'roads': [lane(1, 0), lane(2, 3, True)]}, {'x': 10, 'y': 3})
        self.assertEqual(ids, [1, 2])

    def test_no_distant_nearest_road_fallback(self):
        with self.assertRaises(ValueError):
            select_roads({'roads': [lane(1, 0)]}, {'x': 100, 'y': 100})

    def test_patch_preserves_every_other_byte_and_is_idempotent(self):
        source = (b'<?xml version="1.0"?>\n<osm>\n'
            b'<relation id="1"><member type="way" ref="777" role="left"/>'
            b'<tag k="type" v="lanelet"/><tag k="subtype" v="road"/>'
            b'<tag v="50 km/h" k="speed_limit"/><tag k="signal" v="42"/></relation>\n'
            b'<relation id="2"><tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation>\n'
            b'<relation id="3"><tag k="type" v="lanelet"/><tag k="subtype" v="road"/>'
            b'<tag k="speed_limit" v="80 km/h"/></relation>\n</osm>')
        output, changes = patch_bytes(source, [1, 2])
        expected = source.replace(b'v="50 km/h"', b'v="30 km/h"').replace(
            b'<relation id="2"><tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation>',
            b'<relation id="2"><tag k="type" v="lanelet"/><tag k="subtype" v="road"/>'
            b'  <tag k="speed_limit" v="30 km/h"/>\n  </relation>')
        self.assertEqual(output, expected)
        self.assertEqual(len(changes), 2)
        self.assertEqual(patch_bytes(output, [1, 2]), (output, []))

    def test_unknown_id_fails_before_write(self):
        with self.assertRaises(ValueError):
            patch_bytes(b'<osm/>', [123])

    def test_forward_references_and_reversed_osm_bounds(self):
        content = '''<osm><relation id="10"><member type="way" ref="20" role="left"/>
        <member type="way" ref="21" role="right"/><tag k="type" v="lanelet"/>
        <tag k="subtype" v="road"/></relation>
        <way id="20"><nd ref="1"/><nd ref="2"/></way>
        <way id="21"><nd ref="4"/><nd ref="3"/></way>
        <node id="1"><tag k="local_x" v="0"/><tag k="local_y" v="3"/></node>
        <node id="2"><tag k="local_x" v="20"/><tag k="local_y" v="3"/></node>
        <node id="3"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
        <node id="4"><tag k="local_x" v="20"/><tag k="local_y" v="0"/></node></osm>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.osm'
            path.write_text(content)
            index = make_index(path)
            ids, _ = select_roads(index, {'x': 10, 'y': 1})
            self.assertEqual(ids, [10])


if __name__ == '__main__':
    unittest.main()

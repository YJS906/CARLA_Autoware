"""Ownership, rollback and document validation without altering a live world."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scenario_gui_backend import ScenarioEngine, validate_scenario, atomic_json


def document(**updates):
    a = dict(id='one', kind='vehicle', mode='parked', x=0., y=0., z=.5, yaw=0., speed=0., target=None)
    a.update(updates)
    return dict(schema=1, map='Town05_Opt', actors=[a])


class ScenarioTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.e = ScenarioEngine(Path(self.tmp.name))

    def test_rejects_wrong_map_nonfinite_unknown_mode_and_missing_destination(self):
        cases = [document(x=float('nan')), document(speed=-1), document(mode='teleport'),
                 document(kind='pedestrian', mode='walk'), document(yaw=True)]
        cases.append(dict(schema=1, map='Town01', actors=[]))
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                validate_scenario(case)

    def test_round_trip_preserves_yaw_and_pedestrian_destination(self):
        d = document(kind='pedestrian', mode='walk', speed=1.2, target=[-3., 5.], yaw=-90.)
        p = Path(self.tmp.name)/'scene.json'
        atomic_json(p,d)
        self.assertEqual(validate_scenario(json.loads(p.read_text())),d)

    def test_does_not_delete_unowned_or_role_mismatched_actor(self):
        self.e.world = Mock()
        actor = Mock(type_id='vehicle.tesla.model3', attributes={'role_name':'ego_vehicle'})
        self.e.world.get_actor.return_value=actor
        self.e.owned={1:{'type':actor.type_id,'role':'scenario_gui_test','spec_id':'one'}}
        self.e.connect=Mock(return_value=self.e.world)
        self.e.stop()
        actor.destroy.assert_not_called()
        self.assertEqual(self.e.owned,{})

    def test_deletes_only_verified_owned_actor_and_retains_failed_deletion(self):
        actor=Mock(type_id='vehicle.tesla.model3',attributes={'role_name':'scenario_gui_test'})
        actor.destroy.return_value=False
        self.e.world=Mock();self.e.world.get_actor.return_value=actor
        self.e.owned={1:{'type':actor.type_id,'role':'scenario_gui_test','spec_id':'one'}}
        self.e.connect=Mock(return_value=self.e.world)
        with self.assertRaises(RuntimeError):self.e.stop()
        self.assertIn(1,self.e.owned)
        self.e.world.tick.assert_not_called()
        actor.destroy.return_value=True
        self.e.stop();self.assertEqual(self.e.owned,{})

    def test_world_change_discards_old_ids_without_destroying_new_world_actors(self):
        self.e.world_id=1
        self.e.owned={5:{'role':'scenario_gui_old','type':'vehicle.test'}}
        new=Mock(id=2);new.get_map.return_value.name='Carla/Maps/Town05_Opt'
        self.e.client=Mock();self.e.client.get_world.return_value=new
        self.e.connect()
        self.assertEqual(self.e.owned,{})
        new.get_actor.assert_not_called()
        new.tick.assert_not_called()
        new.apply_settings.assert_not_called()

    def test_second_spawn_failure_rolls_back_first_owned_spawn(self):
        world=Mock();world.get_actors.return_value.filter.return_value=[]
        actor=Mock(id=9,type_id='vehicle.tesla.model3')
        role={}
        bp=world.get_blueprint_library.return_value.find.return_value
        bp.set_attribute.side_effect=lambda k,v: role.update({k:v})
        def spawn(*_):
            actor.attributes={'role_name':role['role_name']}
            return actor
        world.try_spawn_actor.side_effect=[actor,None]
        def persist():
            if 9 in self.e.owned:actor.attributes={'role_name':self.e.owned[9]['role']}
        self.e.persist=Mock(side_effect=persist)
        self.e.world=world;self.e.connect=Mock(return_value=world)
        world.get_actor.return_value=actor;actor.destroy.return_value=True
        d=document();d['actors'].append(dict(d['actors'][0],id='two',x=8.))
        with self.assertRaises(ValueError):self.e.start(d)
        actor.destroy.assert_called_once()
        self.assertEqual(self.e.owned,{})
        world.tick.assert_not_called()

    def test_duplicate_ids_are_rejected_before_spawning(self):
        d=document();d['actors']*=2
        self.e.connect=Mock()
        with self.assertRaises(ValueError):self.e.start(d)
        self.e.connect.assert_not_called()


if __name__=='__main__':unittest.main()

"""Real message classes + plant callbacks, no ROS initialization/DDS. Not a live graph/costmap test."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from PIL import Image
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'digital_twin/ws/src/x2_twin_sim'))
from x2_twin_sim.plant import base


@pytest.fixture
def plant(tmp_path, monkeypatch):
    image = np.full((40,40),254,np.uint8)
    image[0,:]=image[-1,:]=image[:,0]=image[:,-1]=0
    Image.fromarray(image).save(tmp_path/'map.pgm')
    path=tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.25,origin=[-5.,-5.,0.],free_thresh=.196,occupied_thresh=.65)))
    pubs={}
    monkeypatch.setattr(base,'require_sim_environment',lambda:None)
    monkeypatch.setattr(base.Node,'__init__',lambda *a,**kw:None)
    monkeypatch.setattr(base.Node,'declare_parameter',lambda self,k,v:SimpleNamespace(value=str(path) if k=='map_yaml' else v))
    def publisher(self, cls, topic, qos):
        sink=SimpleNamespace(cls=cls,topic_name=topic,messages=[])
        sink.publish=sink.messages.append;pubs[topic]=sink
        return sink
    monkeypatch.setattr(base.Node,'create_publisher',publisher)
    monkeypatch.setattr(base.Node,'create_subscription',lambda *a,**kw:None)
    monkeypatch.setattr(base.Node,'create_timer',lambda *a,**kw:None)
    monkeypatch.setattr(base.Node,'get_clock',lambda self:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=123,nanosec=456))))
    monkeypatch.setattr(base.PlantNode,'current_state',lambda self:((0.,0.,0.),0.,0.))
    node=base.PlantNode('contract_test')
    return node,pubs


def points(pub):
    return np.asarray(pc2.read_points_numpy(pub.messages[-1],field_names=('x','y','z'))).reshape(-1,3)


def test_production_pointcloud_publishers_and_common_stamp(plant):
    node,pubs=plant
    node._publish_scan()
    for topic in ('/qazbot_lio/scan_obstacles','/qazbot_lio/scan_clearing'):
        assert topic in pubs, 'production PointCloud2 source has no plant publisher: '+topic
        assert pubs[topic].cls is PointCloud2
        assert pubs[topic].messages[-1].header==pubs['/qazbot_lio/scan'].messages[-1].header
        assert len(points(pubs[topic]))>0


def test_missing_scan_publishes_no_sensor_stream(plant):
    node,pubs=plant;node.scan_mode='none';node._publish_scan()
    for key in ('/qazbot_lio/scan','/qazbot_lio/scan_obstacles','/qazbot_lio/scan_clearing'):
        assert not pubs[key].messages


def test_empty_scan_cannot_produce_any_clearing_ray(plant):
    node,pubs=plant;node.scan_mode='empty';node._publish_scan()
    assert points(pubs['/qazbot_lio/scan_obstacles']).size==0
    assert points(pubs['/qazbot_lio/scan_clearing']).size==0


def test_disappearing_perception_object_requires_finite_background(plant):
    node,pubs=plant;node.ghosts=[(2.,0.,.3)];node._publish_scan()
    p=points(pubs['/qazbot_lio/scan_obstacles']);front=p[abs(p[:,1])<.02]
    assert np.min(front[:,0])<2
    node.ghosts=[];node._publish_scan()
    p=points(pubs['/qazbot_lio/scan_clearing']);front=p[abs(p[:,1])<.02]
    assert np.max(front[:,0])>4
    node.scan_mode='empty';node._publish_scan()
    assert len(points(pubs['/qazbot_lio/scan_clearing']))==0


def test_blind_sector_is_not_cleared(plant):
    node,pubs=plant;node.scan_mode='blind:-10:10';node._publish_scan()
    p=points(pubs['/qazbot_lio/scan_clearing'])
    a=np.degrees(np.arctan2(p[:,1],p[:,0]))
    assert not np.any(abs(a)<=10)

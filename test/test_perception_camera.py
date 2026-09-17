import numpy as np
import pytest
from dual_fr3_maniskill.perception_camera import decode_textures, CameraSettings


def test_opengl_optical_world_and_invalid_depth():
    color = np.array([[[1.,0.,.5,1.],[0.,1.,0.,1.]]])
    position = np.array([[[.2,.3,-2.,.5],[0.,0.,-3.,1.]]])
    model = np.eye(4)
    model[:3,3] = [1,2,3]
    rgb,depth,t = decode_textures(color,position,model)
    assert rgb[0,0].tolist() == [255,0,127]
    assert depth.dtype == np.float32 and depth[0,0] == 2
    assert np.isnan(depth[0,1])
    np.testing.assert_allclose(t@np.array([.2,-.3,2.,1]),[1.2,2.3,1.,1.])


def test_bad_view_rejected():
    with pytest.raises(ValueError):
        CameraSettings(eye=(0,0,1),target=(0,0,0),up=(0,0,1))

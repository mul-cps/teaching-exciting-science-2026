import os

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
if os.environ.get("MUJOCO_GL") != "egl":
    pytest.skip("set MUJOCO_GL=egl for the real headless renderer smoke", allow_module_level=True)

from workshop.digital_twin.ros_viewer import _render


def test_ros_twin_render_expands_effective_model_offscreen_framebuffer():
    model = mujoco.MjModel.from_xml_string("""
        <mujoco model="render_framebuffer_smoke">
          <visual><global offwidth="640" offheight="480"/></visual>
          <worldbody>
            <light pos="0 0 1" dir="0 0 -1" diffuse="1 1 1"/>
            <geom name="visible_box" type="box" pos="0 0 .15"
                  size=".1 .1 .1" rgba="1 .5 .1 1"/>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    image = _render(model, data, np.zeros(3))
    assert image.shape == (640, 800, 3)
    assert image.dtype == np.uint8
    assert np.any(image)

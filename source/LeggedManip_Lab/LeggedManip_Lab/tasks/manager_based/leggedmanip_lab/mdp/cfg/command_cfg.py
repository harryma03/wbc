# Copyright (c) 2025-2026, Junjie Zhu.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaaclab.utils import configclass

from isaaclab.envs.mdp.commands import (
    UniformPoseCommandCfg as UniformPoseCommandBaseCfg,
)
from isaaclab.envs.mdp.commands import (
    UniformVelocityCommandCfg as UniformVelocityCommandBaseCfg,
)
from ..pose_command_wbc import UniformPoseWBCCommand
from ..pose_command_b import UniformBodyPoseCommand

# -- pose
@configclass
class UniformPoseCommandCfg(UniformPoseCommandBaseCfg):

    class_type: type  = UniformBodyPoseCommand

    ranges: UniformPoseCommandBaseCfg.Ranges = None  # type: ignore
    """The initial range"""
    limit_ranges: UniformPoseCommandBaseCfg.Ranges = None  # type: ignore
    """The range limit """
    curriculum_enabled: bool = False
    root_name: str = "link0"

@configclass
class UniformPoseWBCCommandCfg(UniformPoseCommandBaseCfg):

    class_type: type  = UniformPoseWBCCommand

    ranges: UniformPoseCommandBaseCfg.Ranges = None  # type: ignore
    """The initial range"""
    limit_ranges: UniformPoseCommandBaseCfg.Ranges = None  # type: ignore
    """The range limit """
    link_name: str = "link0"
    """The name of the root body, which serves as the reference frame for the command."""
    curriculum_enabled: bool = False
    workspace_max_radius: float | None = None
    """Optional spherical reach limit measured from ``link_name`` in meters."""
    tool_frame_offset: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """Quaternion (w, x, y, z) from the look-at frame to the tool body frame.

    The look-at construction aligns its local +X axis with the target direction.
    Robots whose physical tool axis is different can supply a fixed offset here
    without changing the command convention for other assets.
    """


# -- velocity
@configclass
class UniformVelocityCommandCfg(UniformVelocityCommandBaseCfg):
    """Configuration for the uniform velocity command generator."""

    ranges: UniformVelocityCommandBaseCfg.Ranges = None  # type: ignore
    """The initial range"""

    limit_ranges: UniformVelocityCommandBaseCfg.Ranges = None  # type: ignore
    """The range limit """
    curriculum_enabled: bool = False

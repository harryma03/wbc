#!/usr/bin/env python3
"""Convert the repository's TITATIT-Piper URDF into a self-contained USD asset.

The source URDF mixes relative MuJoCo mesh paths and ROS ``package://`` paths.
This helper resolves both forms before invoking Isaac Lab's URDF converter.
Run it with the Python environment in which Isaac Sim and Isaac Lab are installed.
"""

from __future__ import annotations

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from isaaclab.app import AppLauncher


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = REPOSITORY_ROOT / "titatit_piper" / "wheeled_titatit_rl_total.urdf"
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "source"
    / "LeggedManip_Lab"
    / "LeggedManip_Lab"
    / "assets"
    / "titatit_piper"
    / "titatit_piper.usd"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Source URDF path.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Destination USD path.")
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def _prepare_urdf(source: Path, destination: Path) -> None:
    """Resolve mesh paths and align semantic body names with this task."""
    tree = ET.parse(source)
    root = tree.getroot()
    model_dir = source.parent
    mesh_dir = model_dir / "meshes"

    # Drop the massless dummy root and make the physical trunk the free base.
    # Keeping this dummy while preserving fixed joints creates a zero-mass root.
    for link in root.findall("link"):
        if link.get("name") == "base":
            root.remove(link)
    for joint in root.findall("joint"):
        if joint.get("name") == "floating_base":
            root.remove(joint)

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        if filename.startswith("package://agx_arm_description/agx_arm_urdf/piper_x/meshes/"):
            relative = filename.split("/meshes/", maxsplit=1)[1]
            resolved = mesh_dir / relative
        elif filename.startswith("../meshes_mujoco/"):
            resolved = mesh_dir / Path(filename).name
        else:
            resolved = (model_dir / filename).resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"URDF mesh does not exist: {filename} -> {resolved}")
        mesh.set("filename", resolved.as_posix())

    # The shared manipulation MDP uses these semantic names. Renaming only the
    # converter's temporary URDF preserves the user's source model unchanged.
    body_name_map = {"trunk": "base", "base_link": "link0", "gripper_base": "end_effector"}
    for link in root.findall("link"):
        name = link.get("name")
        if name in body_name_map:
            link.set("name", body_name_map[name])
    for endpoint in root.findall(".//joint/parent") + root.findall(".//joint/child"):
        name = endpoint.get("link")
        if name in body_name_map:
            endpoint.set("link", body_name_map[name])

    # The source contains an entirely empty link for the master gripper slider.
    # Give it a tiny physical/visual definition so the importer does not create
    # a dangling instanceable-visual reference or invent an arbitrary inertia.
    gripper_link = root.find("./link[@name='gripper_link']")
    if gripper_link is not None and len(gripper_link) == 0:
        inertial = ET.SubElement(gripper_link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.001"})
        ET.SubElement(
            inertial,
            "inertia",
            {"ixx": "1e-8", "ixy": "0", "ixz": "0", "iyy": "1e-8", "iyz": "0", "izz": "1e-8"},
        )
        visual = ET.SubElement(gripper_link, "visual")
        ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "sphere", {"radius": "0.0001"})
        material = ET.SubElement(visual, "material", {"name": "gripper_reference"})
        ET.SubElement(material, "color", {"rgba": "0 0 0 1"})

    tree.write(destination, encoding="utf-8", xml_declaration=True)


args_cli = _parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402


def main() -> None:
    source = args_cli.input.expanduser().resolve()
    output = args_cli.output.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"URDF does not exist: {source}")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="titatit_piper_urdf_") as temp_dir:
        prepared_urdf = Path(temp_dir) / source.name
        _prepare_urdf(source, prepared_urdf)
        converter_cfg = UrdfConverterCfg(
            asset_path=str(prepared_urdf),
            usd_dir=str(output.parent),
            usd_file_name=output.name,
            force_usd_conversion=True,
            fix_base=False,
            merge_fixed_joints=False,
            make_instanceable=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                drive_type="force",
                target_type="none",
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
            ),
            collider_type="convex_hull",
            self_collision=False,
        )
        converter = UrdfConverter(converter_cfg)
        print(f"Generated USD: {converter.usd_path}")

        # Avoid persisting a path into this temporary directory in converter
        # metadata. The generated USD itself is already self-contained.
        metadata_path = output.parent / "config.yaml"
        metadata = metadata_path.read_text(encoding="utf-8")
        metadata_path.write_text(
            metadata.replace(f"asset_path: {prepared_urdf}", "asset_path: null"), encoding="utf-8"
        )

        # Fail early if the converter produced a stage that cannot be opened.
        stage = sim_utils.open_stage(converter.usd_path)
        if stage is None:
            raise RuntimeError(f"Unable to open generated USD: {converter.usd_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

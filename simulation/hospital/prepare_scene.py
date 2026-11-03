"""Compose a sensor-bounded hospital scene without changing NVIDIA assets."""

import argparse
from pathlib import Path


def prepare_scene(asset_root: Path, output: Path, contact_fixture=False):
    import isaacsim  # Makes the shared runtime's USD bindings available.
    from pxr import Gf, Usd, UsdGeom

    reference = asset_root / "Isaac/Samples/ROS2/Scenario/multiple_robot_carter_hospital_navigation.usd"
    if not reference.is_file():
        raise FileNotFoundError(reference)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"Will not overwrite an existing scene: {output}")
    stage = Usd.Stage.CreateInMemory(load=Usd.Stage.LoadNone)
    stage.GetRootLayer().subLayerPaths = [str(reference)]
    # Select variants before loading the sensor payloads. The vendor example
    # otherwise instantiates 36 cameras and many ROS publishers for three AMRs.
    for number in (1, 2, 3):
        robot = stage.OverridePrim(f"/World/Nova_Carter_ROS_{number}")
        robot.GetVariantSets().GetVariantSet("Sensors").SetVariantSelection("None")
        robot.GetVariantSets().GetVariantSet("ROS").SetVariantSelection("Disabled")
        stage.OverridePrim(f"/World/Nova_Carter_ROS_{number}/chassis_link/sensors").SetActive(False)
    stage.Load()
    outer = stage.GetPrimAtPath("/World/GroundPlane/CollisionPlane")
    inner = stage.GetPrimAtPath("/World/hospital/GroundPlane/CollisionPlane")
    if outer and inner and outer.GetTypeName() == inner.GetTypeName() == "Plane":
        def plane(prim):
            axis = prim.GetAttribute("axis").Get() or "Z"
            normal = {"X":Gf.Vec3d(1,0,0),"Y":Gf.Vec3d(0,1,0),"Z":Gf.Vec3d(0,0,1)}[axis]
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return matrix.Transform(Gf.Vec3d(0)),matrix.TransformDir(normal).GetNormalized()
        p0,n0 = plane(outer)
        p1,n1 = plane(inner)
        if abs(Gf.Dot(n0,n1)) > .999999 and abs(Gf.Dot(p1-p0,n0)) < 1e-6:
            stage.OverridePrim(outer.GetPath()).SetActive(False)
    graph_paths = [p.GetPath() for p in stage.Traverse() if p.GetTypeName() == "OmniGraph"]
    for path in graph_paths:
        stage.OverridePrim(path).SetActive(False)
    if contact_fixture:
        for number,position in enumerate(((0,0,0),(1.4,0,0),(9.5,0,0)),1):
            robot = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Nova_Carter_ROS_{number}"))
            robot.ClearXformOpOrder()
            robot.AddTranslateOp(opSuffix="contact_test").Set(Gf.Vec3d(*position))
            robot.AddOrientOp(opSuffix="contact_test").Set(Gf.Quatf(1,0,0,0))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(1_000_000_000)
    stage.SetTimeCodesPerSecond(60)
    stage.SetFramesPerSecond(60)
    stage.GetRootLayer().Export(str(output))
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--contact-fixture", action="store_true", help="Place robots for physical contact validation only")
    arguments = parser.parse_args()
    print(prepare_scene(arguments.asset_root, arguments.output, arguments.contact_fixture))

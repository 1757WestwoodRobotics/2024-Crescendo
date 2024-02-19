from collections import deque
from math import hypot, sin
from typing import List, Tuple

# import numpy as np

from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard
from wpilib import RobotBase, Timer
from wpimath.geometry import Transform3d, Pose3d, Pose2d, Rotation2d

import constants
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d


class EstimatedPose:
    def __init__(self, pose: Pose3d, hasTargets: bool, timestamp: float):
        self.pose = pose
        self.hasTargets = hasTargets
        self.timestamp = timestamp


class VisionCamera:
    def __init__(self, camera: PhotonCamera):
        self.camera = camera
        self.name = camera.getName()

        cameraKeys = {
            "frontLeft": constants.kPhotonvisionFrontLeftCameraKey,
            # "frontRight": constants.kPhotonvisionFrontRightCameraKey,
            # "backLeft": constants.kPhotonvisionBackLeftCameraKey,
            "backRight": constants.kPhotonvisionBackRightCameraKey,
        }

        self.key = cameraKeys[self.name]

        cameraTransforms = {
            "frontLeft": constants.kRobotToFrontLeftCameraTransform,
            # frontRight": constants.kRobotToFrontRightCameraTransform,
            # "backLeft": constants.kRobotToBackLeftCameraTransform,
            "backRight": constants.kRobotToBackRightCameraTransform,
        }

        self.cameraToRobotTransform = cameraTransforms[self.name].inverse()
        self.CADToRobotTransform = # CAD inputs from Onshape

    def CADToRobotCamera(self) -> Tuple[List[Tuple[int, Transform3d]], float]:
        photonOutput = self.camera.getLatestResult()
        if photonOutput.hasTargets:
            return (
                [
                    (target.getFiducialID(), target.getBestCameraToTarget())
                    for target in photonOutput.getTargets()
                    if target.getPoseAmbiguity() < constants.kPhotonvisionAmbiguityCutoff
                ],
                self.camera.Transform = self.cameraToRobotTransform - CADToRobotTransform,
            )
        else:
            return ([], 0)


class VisionSubsystemReal(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        # self.estimatedPosition = Pose2d()

        # self.camera = PhotonCamera(constants.kPhotonvisionCameraName)

        self.cameras = [
            PhotonCamera(camera) for camera in constants.kPhotonvisionCameraArray
        ]
        self.poseList = deque([])
        # if RobotBase.isSimulation():
        #     inst = NetworkTableInstance.getDefault()
        #     inst.stopServer()
        #     inst.setServer("localhost")
        #     inst.startClient4("Robot Sim")

    def periodic(self) -> None:
        # self.estimatedPosition = self.drive.getPose()
        # self.updateAdvantagescopePose()
        visionPose = SmartDashboard.getNumberArray(
            constants.kRobotVisionPoseArrayKeys.valueKey, [0, 0, 0]
        )
        robotPose = SmartDashboard.getNumberArray(
            constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
        )
        combinedPose = pose3dFrom2d(Pose2d(visionPose[0], visionPose[1], robotPose[2]))
        for camera in self.cameras:
            photonResult = camera.getLatestResult()
            hasTargets = len(photonResult.getTargets()) > 0
            multitagresult = photonResult.multiTagResult
            bestRelativeTransform = multitagresult.estimatedPose.best

            currentCamera = VisionCamera(camera)

            if not multitagresult.estimatedPose.isPresent:
                ambiguity = 10
                for result in photonResult.targets:
                    if result.poseAmbiguity < ambiguity:
                        bestRelativeTransform = (
                            Transform3d(
                                Pose3d(),
                                constants.kApriltagPositionDict[result.fiducialId],
                            )
                            + result.bestCameraToTarget.inverse()
                        )
                        ambiguity = result.poseAmbiguity

            VisionSubsystemReal.updateAdvantagescopePose(
                Pose3d() + bestRelativeTransform,
                currentCamera.key,
                combinedPose,
                currentCamera.cameraToRobotTransform,
            )

            botPose = (
                Pose3d() + bestRelativeTransform + currentCamera.cameraToRobotTransform
            )

            self.poseList.append(
                EstimatedPose(botPose, hasTargets, photonResult.getTimestamp())
            )

    @staticmethod
    def updateAdvantagescopePose(
        cameraPose3d: Pose3d,
        cameraKey: str,
        botPose: Pose3d,
        cameraToRobotTransform: Transform3d,
    ) -> None:
        cameraPose = advantagescopeconvert.convertToSendablePoses(
            [cameraPose3d, botPose + cameraToRobotTransform.inverse()]
        )
        SmartDashboard.putNumberArray(cameraKey, cameraPose)


class CameraTargetRelation:
    def __init__(self, cameraPose: Pose3d, targetPose: Pose3d) -> None:
        self.cameraPose = cameraPose
        self.camToTarg = Transform3d(cameraPose, targetPose)
        self.camToTargDist = self.camToTarg.translation().norm()
        self.camToTargDistXY = hypot(
            self.camToTarg.translation().X(), self.camToTarg.translation().Y()
        )
        self.camToTargYaw = Rotation2d(self.camToTarg.X(), self.camToTarg.Y())
        self.camToTargPitch = Rotation2d(self.camToTargDistXY, -self.camToTarg.Z())
        self.camToTargAngle = Rotation2d(
            hypot(self.camToTargYaw.radians(), self.camToTargPitch.radians())
        )

        self.targToCam = Transform3d(targetPose, cameraPose)
        self.targToCamYaw = Rotation2d(self.targToCam.X(), self.targToCam.Y())
        self.targToCamPitch = Rotation2d(self.camToTargDistXY, -self.targToCam.Z())
        self.targToCamAngle = Rotation2d(
            hypot(self.targToCamYaw.radians(), self.targToCamPitch.radians())
        )


class SimCamera:
    def __init__(
        self,
        name: str,
        location: Transform3d,
        horizFOV: float,
        vertFOV: float,
        key: str,
    ) -> None:
        self.name = name
        self.location = location
        self.horizFOV = horizFOV
        self.vertFOV = vertFOV
        self.key = key

    def canSeeTarget(self, botPose: Pose3d, targetPose: Pose3d):
        cameraPose = botPose + self.location
        rel = CameraTargetRelation(cameraPose, targetPose)
        return (
            abs(rel.camToTargYaw.degrees()) < self.horizFOV / 2
            and abs(rel.camToTargPitch.degrees()) < self.vertFOV / 2
            and abs(rel.targToCamAngle.degrees()) < 90
        )


class VisionSubsystemSim(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.cameras = [
            SimCamera(
                name,
                location,
                constants.kCameraFOVHorizontal,
                constants.kCameraFOVVertical,
                key,
            )
            for name, location, key in zip(
                constants.kPhotonvisionCameraArray,
                constants.kCameraTransformsArray,
                constants.kPhotonvisionKeyArray,
            )
        ]
        self.poseList = []

        self.rng = RNG(constants.kSimulationVariation)

    def periodic(self) -> None:
        simPose = Pose2d(
            *SmartDashboard.getNumberArray(constants.kSimRobotPoseArrayKey, [0, 0, 0])
        )
        simPose3d = pose3dFrom2d(simPose)

        for camera in self.cameras:
            seeTag = False
            botPose = Pose3d()
            for _id, apriltag in constants.kApriltagPositionDict.items():
                if camera.canSeeTarget(simPose3d, apriltag):
                    seeTag = True
                    botPose = Pose3d(
                        simPose3d.X() + self.rng.getNormalRandom(),
                        simPose3d.Y() + self.rng.getNormalRandom(),
                        simPose3d.Z() + self.rng.getNormalRandom(),
                        simPose3d.rotation(),
                    )

            rel = CameraTargetRelation(simPose3d + camera.location, botPose)
            VisionSubsystemReal.updateAdvantagescopePose(
                botPose + camera.location, camera.key, simPose3d, rel.camToTarg
            )

            self.poseList.append(
                EstimatedPose(botPose, seeTag, Timer.getFPGATimestamp())
            )


class RNG:
    def __init__(self, stdDev: float) -> None:
        self.stdDev = stdDev
        # self.rng = np.random.normal(0, stdDev, number)
        # self.rngIdx = 0

    def getNormalRandom(self) -> float:
        return sin(1000000 * Timer.getFPGATimestamp()) * self.stdDev
        # self.rngIdx = (self.rngIdx + 1) % self.number
        # return self.rng[self.rngIdx]


class VisionSubsystem(Subsystem):
    # this is really bad
    def __init__(self) -> None:
        if RobotBase.isSimulation():
            # pylint:disable-next=non-parent-init-called
            VisionSubsystemSim.__init__(self)
        else:
            # pylint:disable-next=non-parent-init-called
            VisionSubsystemReal.__init__(self)

    def periodic(self) -> None:
        if RobotBase.isSimulation():
            VisionSubsystemSim.periodic(self)
        else:
            VisionSubsystemReal.periodic(self)
from collections import deque
from math import hypot, sin, tan, atan

# import numpy as np

from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from wpilib import SmartDashboard
from wpilib import RobotBase, Timer
from wpimath.geometry import (
    Transform3d,
    Pose3d,
    Pose2d,
    Rotation2d,
    Translation3d,
    Transform2d,
    Rotation3d,
)

import constants
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d


class EstimatedPose:
    def __init__(self, pose: Pose3d, hasTargets: bool, timestamp: float):
        self.pose = pose
        self.hasTargets = hasTargets
        self.timestamp = timestamp


class VisionCamera:  # hi its landon here
    def __init__(self, camera: PhotonCamera):
        self.camera = camera
        self.name = camera.getName()

        cameraKeys = {
            "frontLeft": constants.kPhotonvisionFrontLeftCameraKey,
            "frontRight": constants.kPhotonvisionFrontRightCameraKey,
            "backLeft": constants.kPhotonvisionBackLeftCameraKey,
            "backRight": constants.kPhotonvisionBackRightCameraKey,
        }

        self.key = cameraKeys[self.name]

        cameraTransforms = {
            "frontLeft": constants.kRobotToFrontLeftCameraTransform,
            "frontRight": constants.kRobotToFrontRightCameraTransform,
            "backLeft": constants.kRobotToBackLeftCameraTransform,
            "backRight": constants.kRobotToBackRightCameraTransform,
        }

        self.cameraToRobotTransform = cameraTransforms[self.name].inverse()


class VisionSubsystemReal(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        # self.estimatedPosition = Pose2d()

        # self.camera = PhotonCamera(constants.kPhotonvisionCameraName)

        self.cameras = [
            PhotonCamera(camera) for camera in constants.kPhotonvisionCameraArray
        ]
        self.robotToTags = []

        self.noteCamera = PhotonCamera(constants.kPhotonvisionNoteCameraKey)

        self.poseList = deque([])

        self.dRobotAngle = Rotation2d()

        SmartDashboard.putBoolean(constants.kNoteInViewKey.validKey, False)
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

        noteResult = self.noteCamera.getLatestResult()
        if noteResult.hasTargets():
            notes = noteResult.getTargets()
            notePositions = [
                (
                    Pose3d(
                        robotPose[0], robotPose[1], 0, Rotation3d(0, 0, robotPose[2])
                    )
                    + constants.kRobotToNoteCameraTransform
                    + VisionSubsystemReal.getCameraToNote(self, note)
                ).toPose2d()
                for note in notes
            ]
            SmartDashboard.putNumberArray(
                constants.kNoteInViewKey.valueKey,
                advantagescopeconvert.convertToPose2dSendable(notePositions),
            )
            closestNote = Pose2d(*robotPose).nearest(notePositions)
            intakePickupPosition = (
                Pose2d(*robotPose) + constants.kRobotToIntakePickupTransform
            )

            # angle robot needs to rotate by to pick up note by driving forward
            self.dRobotAngle = (
                Rotation2d(robotPose[2])
                + Transform2d(intakePickupPosition, closestNote).rotation()
            )

            SmartDashboard.putBoolean(constants.kNoteInViewKey.validKey, True)
        else:
            # rotate around if no note in vision
            SmartDashboard.putBoolean(constants.kNoteInViewKey.validKey, False)

        combinedPose = pose3dFrom2d(Pose2d(visionPose[0], visionPose[1], robotPose[2]))
        self.robotToTags = []
        for camera in self.cameras:
            photonResult = camera.getLatestResult()
            hasTargets = len(photonResult.getTargets()) > 0
            multitagresult = photonResult.multiTagResult
            bestRelativeTransform = multitagresult.estimatedPose.best

            currentCamera = VisionCamera(camera)

            ambiguity = 10
            if not multitagresult.estimatedPose.isPresent:
                for result in photonResult.targets:
                    # pylint:disable-next=consider-iterating-dictionary
                    if result.fiducialId in constants.kApriltagPositionDict.keys():
                        botToTagPose = (
                            currentCamera.cameraToRobotTransform.inverse()
                            + result.bestCameraToTarget
                        )
                        self.robotToTags.append(
                            (botToTagPose, result.fiducialId, result.poseAmbiguity),
                        )
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

            if ambiguity < 10:
                self.poseList.append(
                    EstimatedPose(botPose, hasTargets, photonResult.getTimestamp())
                )

        if len(self.robotToTags) > 0:
            poses, ids, ambiguitys = list(zip(*self.robotToTags))

            poses3d = advantagescopeconvert.convertToSendablePoses(poses)
            SmartDashboard.putNumberArray(constants.kRobotToTagPoseKey, poses3d)
            SmartDashboard.putNumberArray(constants.kRobotToTagIdKey, ids)
            SmartDashboard.putNumberArray(constants.kRobotToTagAmbiguityKey, ambiguitys)

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

    def getCameraToNote(self, note: PhotonTrackedTarget) -> Transform3d:
        x = constants.kRobotToNoteCameraTransform.Z() / tan(
            constants.kNoteCameraPitch - note.getPitch() * constants.kRadiansPerDegree
        )
        dist = (constants.kRobotToNoteCameraTransform.Z() ** 2 + x**2) ** 0.5
        y = dist * tan(-note.getYaw() * constants.kRadiansPerDegree)

        return Transform3d(x, y, 0, Rotation3d(0, 0, atan(y / x)))


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
        self.robotToTags = []

        self.rng = RNG(constants.kSimulationVariation)

    def periodic(self) -> None:
        simPose = Pose2d(
            *SmartDashboard.getNumberArray(constants.kSimRobotPoseArrayKey, [0, 0, 0])
        )
        simPose3d = pose3dFrom2d(simPose)

        self.robotToTags = []
        for camera in self.cameras:
            seeTag = False
            botPose = Pose3d()
            for tagId, apriltag in constants.kApriltagPositionDict.items():
                if camera.canSeeTarget(simPose3d, apriltag):
                    rngOffset = Transform3d(
                        Translation3d(
                            self.rng.getNormalRandom(),
                            self.rng.getNormalRandom(),
                            self.rng.getNormalRandom(),
                        ),
                        Rotation3d(),
                    )
                    botToTagPose = Pose3d() + Transform3d(simPose3d, apriltag)
                    botToTagPose = (
                        botToTagPose + rngOffset * botToTagPose.translation().norm()
                    )
                    self.robotToTags.append(
                        (
                            botToTagPose,
                            tagId,
                            botToTagPose.translation().norm()
                            * self.rng.getNormalRandom(),
                        ),
                    )
                    seeTag = True
                    botPose = (
                        Pose3d(
                            simPose3d.X(),
                            simPose3d.Y(),
                            simPose3d.Z(),
                            simPose3d.rotation(),
                        )
                        + rngOffset
                    )

            rel = CameraTargetRelation(simPose3d + camera.location, botPose)
            VisionSubsystemReal.updateAdvantagescopePose(
                botPose + camera.location, camera.key, simPose3d, rel.camToTarg
            )

            self.poseList.append(
                EstimatedPose(botPose, seeTag, Timer.getFPGATimestamp())
            )

        if len(self.robotToTags) > 0:
            poses, ids, ambiguitys = list(zip(*self.robotToTags))

            poses3d = advantagescopeconvert.convertToSendablePoses(poses)
            SmartDashboard.putNumberArray(constants.kRobotToTagPoseKey, poses3d)
            SmartDashboard.putNumberArray(constants.kRobotToTagIdKey, ids)
            SmartDashboard.putNumberArray(constants.kRobotToTagAmbiguityKey, ambiguitys)


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

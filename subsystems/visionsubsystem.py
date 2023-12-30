from typing import List, Tuple
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Pose2d, Transform3d, Pose3d

import constants
from subsystems.drivesubsystem import DriveSubsystem
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d


class VisionSubsystem(Subsystem):
    def __init__(self, drive: DriveSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.estimatedPosition = Pose2d()

        self.camera = PhotonCamera(constants.kPhotonvisionCameraName)

        # if RobotBase.isSimulation():
        #     inst = NetworkTableInstance.getDefault()
        #     inst.stopServer()
        #     inst.setServer("localhost")
        #     inst.startClient4("Robot Sim")

    def getCameraToTargetTransforms(
        self,
    ) -> Tuple[List[Tuple[int, Transform3d]], float]:
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        photonResult = self.camera.getLatestResult()
        targets = photonResult.getTargets()
        if len(targets) > 0:
            return (
                [
                    (target.getFiducialId(), target.getBestCameraToTarget())
                    for target in targets
                    if target.getPoseAmbiguity()
                    < constants.kPhotonvisionAmbiguityCutoff
                ],
                photonResult.getTimestamp(),
            )
        else:
            return ([], 0)

    def periodic(self) -> None:
        self.estimatedPosition = self.drive.getPose()
        self.updateAdvantagescopePose()

        photonResult = self.camera.getLatestResult()
        hasTargets = len(photonResult.getTargets()) > 0
        multitagresult = photonResult.multiTagResult

        bestRelativeTransform = multitagresult.estimatedPose.best

        botPose = (
            Pose3d()
            + bestRelativeTransform
            + constants.kLimelightRelativeToRobotTransform.inverse()
        )

        self.drive.visionEstimate = botPose.toPose2d()

        SmartDashboard.putBoolean(
            constants.kRobotVisionPoseArrayKeys.validKey, hasTargets
        )
        SmartDashboard.putNumberArray(
            constants.kRobotVisionPoseArrayKeys.valueKey,
            [
                self.drive.visionEstimate.Y(),
                self.drive.visionEstimate.rotation().radians(),
            ],
        )

    def updateAdvantagescopePose(self) -> None:
        limelightPose3d = (
            pose3dFrom2d(self.estimatedPosition)
            + constants.kLimelightRelativeToRobotTransform
        )
        limelightPose = advantagescopeconvert.convertToSendablePoses([limelightPose3d])

        SmartDashboard.putNumberArray(constants.kLimelightPoseKey, limelightPose)

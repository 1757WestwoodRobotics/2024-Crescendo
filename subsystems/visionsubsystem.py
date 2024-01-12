from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform3d, Pose3d

import constants
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d


class EstimatedPose:
    def __init__(self, pose: Pose3d, hasTargets: bool, timestamp: float):
        self.pose = pose
        self.hasTargets = hasTargets
        self.timestamp = timestamp


class VisionSubsystem(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        # self.estimatedPosition = Pose2d()

        # self.camera = PhotonCamera(constants.kPhotonvisionCameraName)

        self.cameras = [
            PhotonCamera(camera) for camera in constants.kPhotonvisionCameraArray
        ]
        self.poseList = []
        # if RobotBase.isSimulation():
        #     inst = NetworkTableInstance.getDefault()
        #     inst.stopServer()
        #     inst.setServer("localhost")
        #     inst.startClient4("Robot Sim")

    def periodic(self) -> None:
        # self.estimatedPosition = self.drive.getPose()
        # self.updateAdvantagescopePose()
        for camera in self.cameras:
            photonResult = camera.getLatestResult()
            hasTargets = len(photonResult.getTargets()) > 0
            multitagresult = photonResult.multiTagResult
            bestRelativeTransform = multitagresult.estimatedPose.best

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

            cameraTransforms = {
                "frontLeft": constants.kRobotToFrontLeftCameraTransform,
                "frontRight": constants.kRobotToFrontRightCameraTransform,
                "backLeft": constants.kRobotToBackLeftCameraTransform,
                "backRight": constants.kRobotToBackRightCameraTransform,
            }

            botPose = (
                Pose3d()
                + bestRelativeTransform
                + cameraTransforms[camera.getName()].inverse()
            )

            self.poseList.append(
                EstimatedPose(
                    botPose, hasTargets, camera.getLatestResult().getTimestamp()
                )
            )

        # SmartDashboard.putData(constants.kPhotonvisionCamerasKey, poseList)

        # self.drive.visionEstimate = botPose.toPose2d()

        # SmartDashboard.putBoolean(
        #     constants.kRobotVisionPoseArrayKeys.validKey, hasTargets
        # )
        # SmartDashboard.putNumberArray(
        #     constants.kRobotVisionPoseArrayKeys.valueKey,
        #     [
        #         self.drive.visionEstimate.X(),
        #         self.drive.visionEstimate.Y(),
        #         self.drive.visionEstimate.rotation().radians(),
        #     ],
        # )

    # move to drive subsystem
    # def updateAdvantagescopePose(self) -> None:
    #     limelightPose3d = (
    #         pose3dFrom2d(self.estimatedPosition)
    #         + constants.kLimelightRelativeToRobotTransform
    #     )
    #     limelightPose = advantagescopeconvert.convertToSendablePoses([limelightPose3d])

    #     SmartDashboard.putNumberArray(constants.kLimelightPoseKey, limelightPose)

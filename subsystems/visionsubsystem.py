from collections import deque
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
        self.poseList = deque([])
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

            self.updateAdvantagescopePose(Pose3d() + bestRelativeTransform, camera)

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
                    botPose, hasTargets, photonResult.getTimestamp()
                )
            )

    def updateAdvantagescopePose(
        self, cameraPose3d: Pose3d, camera: PhotonCamera
    ) -> None:
        cameraKeys = {
            "frontLeft": constants.kPhotonvisionFrontLeftCameraKey,
            "frontRight": constants.kPhotonvisionFrontRightCameraKey,
            "backLeft": constants.kPhotonvisionBackLeftCameraKey,
            "backRight": constants.kPhotonvisionBackRightCameraKey,
        }

        cameraPose = advantagescopeconvert.convertToSendablePoses([cameraPose3d])
        SmartDashboard.putNumberArray(cameraKeys[camera.getName()], cameraPose)

from collections import deque
from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Pose3d, Rotation3d, Pose2d

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

            self.updateAdvantagescopePose(
                Pose3d() + bestRelativeTransform, currentCamera, combinedPose
            )

            botPose = (
                Pose3d() + bestRelativeTransform + currentCamera.cameraToRobotTransform
            )

            self.poseList.append(
                EstimatedPose(botPose, hasTargets, photonResult.getTimestamp())
            )



    def updateAdvantagescopePose(
            self, cameraPose3d: Pose3d, camera: VisionCamera, botPose: Pose3d
    ) -> None:
        cameraPose = advantagescopeconvert.convertToSendablePoses(
            [cameraPose3d, botPose + camera.cameraToRobotTransform.inverse()]
        )
        SmartDashboard.putNumberArray(camera.key, cameraPose)

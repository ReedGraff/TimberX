package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.V2_5Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import kotlin.random.URandomKt;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "left", preselectTeleOp="v1Tele")
public class v1AutonLeft extends LinearOpMode {
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "backWebcam";

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // Set values to zero:
        drive.setVerticalSlide("zero", true);
        drive.setHorizontalSlide("zero", true);

        // Start position
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // Build the trajectories
        TrajectorySequence scoring = drive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(new Pose2d(-36, -15, Math.toRadians(270)))
            .addTemporalMarker(1.5, ()-> {
                // Get Color
                SleeveDetection.ParkingPosition conePos = sleeveDetection.getPosition();
                camera.stopStreaming();

                // Passing vals to telemetry
                if (conePos == SleeveDetection.ParkingPosition.LEFT) {
                    V2_5Drive.changeParking(1);
                } else if (conePos == SleeveDetection.ParkingPosition.CENTER) {
                    V2_5Drive.changeParking(2);
                } else {
                    V2_5Drive.changeParking(3);
                }
            })
            .addDisplacementMarker(() -> {
                // Where we define async peripheral movements...
                drive.setGrabber("wait");
                drive.setGrabber("release");
            })
            //.lineToLinearHeading(new Pose2d(-37, -4, Math.toRadians(20)))
            .lineToLinearHeading(new Pose2d(-37, -17, Math.toRadians(45)))
            .build();

        TrajectorySequence parking1 = drive.trajectorySequenceBuilder(scoring.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(90)))
                .build();

        TrajectorySequence parking2 = drive.trajectorySequenceBuilder(scoring.end())
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)))
                .build();

        TrajectorySequence parking3 = drive.trajectorySequenceBuilder(scoring.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(90)))
                .build();

        // Start color sensor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // Initialization ends, and the round starts
        waitForStart();
        if (isStopRequested()) return;

        // Movements
        drive.followTrajectorySequence(scoring);
        drive.stackLoop2();

        if (V2_5Drive.getParking() == 1) {
            drive.followTrajectorySequence(parking1);
        } else if (V2_5Drive.getParking() == 2) {
            drive.followTrajectorySequence(parking2);
        } else {
            drive.followTrajectorySequence(parking3);
        }

        // Telemetry
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}

/*
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );


 */
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.V2_5Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "left", preselectTeleOp="v1Tele")
public class v1AutonLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // Set values to zero:
        drive.setVerticalSlide("zero", true);
        drive.setHorizontalSlide("zero", true);

        // Start position
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Build the trajectories
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            .splineToLinearHeading(new Pose2d(-36, -15, Math.toRadians(90)), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-37, -4, Math.toRadians(20)), Math.toRadians(20))
            .addDisplacementMarker(() -> {
                // Where we define peripheral movements...
                drive.setVerticalSlide("highJunction", false);
                drive.setHorizontalSlide("leftFromLeft", false);
                drive.setGrabber("Wait");
            })
            .addTemporalMarker(1.0, 0.2, () -> {
                drive.setVerticalSlideGrabber("highJunction");
            })
            .addTemporalMarker(1.0, 0.5, () -> {
                drive.setGrabber("topStack");

                drive.setVerticalSlideGrabber("Passing");
                drive.setVerticalSlide("Passing", false);
            })
            .addTemporalMarker(1.0, 1, () -> {
                drive.setGrabber("grab");
            })
            .addTemporalMarker(1.0, 2.5, () -> {
                drive.setGrabber("topStack");
            })
            .build();

        // Initialization ends, and the round starts
        waitForStart();
        if (isStopRequested()) return;

        // Movements
        drive.followTrajectorySequence(traj1);

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
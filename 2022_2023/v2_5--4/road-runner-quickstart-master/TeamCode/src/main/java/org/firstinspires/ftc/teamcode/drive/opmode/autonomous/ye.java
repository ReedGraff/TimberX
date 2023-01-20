package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.V2_5Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ye {

    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "left", preselectTeleOp="v1Tele")
    public class ye extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            // Initialization
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            V2_5Drive drive = new V2_5Drive(hardwareMap);

            // Build the trajectories
            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(-36, -15, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-37, -4, Math.toRadians(20)), Math.toRadians(20))
                    .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
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
}

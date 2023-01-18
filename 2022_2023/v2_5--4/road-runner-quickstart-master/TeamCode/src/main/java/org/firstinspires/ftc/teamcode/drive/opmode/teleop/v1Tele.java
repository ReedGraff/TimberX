package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.V2_5Drive;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@TeleOp(group = "teleop")
public class v1Tele extends LinearOpMode {
    //private int distance = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            /*
            if (gamepad1.a) {
                distance += 50;
                drive.linearPosStop();
                drive.linearStretch(distance);
            }
            */
            /*if (gamepad1.x==true && emt == 0 && distance>=0) {
                distance = 6000;
                drive.linearStretch(distance);
                emt = 1;
                while (emt == 1){
                    if (gamepad1.x==false){
                        emt = 0;
                    }
                }
            }

             */
            /*if (gamepad1.y==true && emt == 0 && distance>0 && distance<6001) {
                distance = 0;
                drive.linearStretch(distance);
                emt = 1;
                while (emt == 1){
                    if (gamepad1.y==false){
                        emt = 0;
                    }
                }
            }

             */
            /*
            if (gamepad1.x==true) {

                drive.release();

                while (emt1 == 1){
                    if (gamepad1.left_bumper==false){
                        emt1 = 0;
                    }
                }


            }
            */
            /*
            if (gamepad1.y==true ) {

                drive.grab();

                while (emt1 == 1){
                    if (gamepad1.right_bumper==false){
                        emt1 = 0;
                    }
                }


            }
            */


        }
    }
}

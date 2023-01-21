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
    private int grabLiftCase = 0;
    private int grabCase = 0;
    private int junction = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // Set them to float
        // drive.verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // drive.horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set values to zero:
        drive.setVerticalSlide("zero", true);
        drive.setHorizontalSlide("Zero",true);
        drive.setGrabber("Zero");

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        // Define times for buttons...
        long last_a_press = System.currentTimeMillis();
        long last_b_press = System.currentTimeMillis();
        long last_y_press = System.currentTimeMillis();
        long last_x_press = System.currentTimeMillis();
        long last_left_bumper_press = System.currentTimeMillis();
        long last_right_bumper_press = System.currentTimeMillis();
        long last_left_trigger_press = System.currentTimeMillis();
        long last_right_trigger_press = System.currentTimeMillis();
        long last_dpad_up_press = System.currentTimeMillis();


        while (opModeIsActive() && !isStopRequested()) {
            long currentTime = System.currentTimeMillis();

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
            telemetry.addData("vslide", drive.verticalSlide.getCurrentPosition());
            telemetry.addData("hslide", drive.horizontalSlide.getCurrentPosition());
            telemetry.update();


            // You can make 250 equal to zero... this was just an example of how you could implement holding a button vs pressing a button...

            if (gamepad1.a) {
                if (currentTime - last_a_press > 1000) {
                    last_a_press = System.currentTimeMillis();
                    if (grabCase == 0){
                        drive.setGrabber("grab");
                        grabCase = 1;
                    }
                    else if (grabCase == 1){
                        drive.setGrabber("release");
                            grabCase = 0;

                    }

                }
            }
            if (gamepad1.b) {
                if (currentTime - last_b_press > 1000) {
                    last_b_press = System.currentTimeMillis();
                    if (grabLiftCase == 0){
                        drive.setGrabber("zero");
                        grabLiftCase = -1;
                    }
                    else if (grabLiftCase == 1){
                        drive.setGrabber("passing");
                        grabLiftCase = 0;
                    }
                    if (grabLiftCase == -1){
                        grabLiftCase = 1;
                    }
                }
            }
            if (gamepad1.y) {
                if (currentTime - last_y_press > 1000) {
                    last_y_press = System.currentTimeMillis();
                    junction = 0;
                    drive.setVerticalSlide("passing", false);
                }
            }

            /*if (gamepad1.x) {
                if (currentTime - last_x_press > 1000) {
                    last_x_press = System.currentTimeMillis();
                    switch (getParking()) {
                        case 1:
                            ; //TODO
                            break;
                        case 2:
                            ; // TODO
                            break;
                        case 3:
                            ; // TODO
                            break;
                        default:
                            break;
                    }
                }
            }

             */


            if (gamepad1.right_bumper) {
                if (currentTime - last_right_bumper_press > 500) {
                    last_right_bumper_press = System.currentTimeMillis();
                    if (junction == 2 || junction == 3) {
                        drive.setVerticalSlideGrabber("mediumJunction");
                    }
                    else if (junction == 1){
                        drive.setVerticalSlideGrabber("lowJunction");
                    }
                }
            }
            if (gamepad1.left_bumper) {
                if (currentTime - last_left_bumper_press > 500) {
                    last_left_bumper_press = System.currentTimeMillis();
                    drive.setVerticalSlideGrabber("passing");
                }
            }
            if (gamepad1.dpad_up) {
                if (currentTime - last_right_bumper_press > 500) { //TODO
                    last_right_bumper_press = System.currentTimeMillis();
                    try{
                        switch (junction) {
                            case 0:
                                drive.setVerticalSlide("lowJunction", false);
                                junction += 1;
                                break;
                            case 1:
                                drive.setVerticalSlide("mediumJunction", false);
                                junction += 1;
                                break;
                            case 2:
                                drive.setVerticalSlide("highJunction", false);
                                junction += 1;
                                break;
                            default:

                                break;
                        }
                    }catch (Exception ignored) {}

                }
            }
            if (gamepad1.dpad_down) {
                if (currentTime - last_left_bumper_press > 500) { // TODO
                    try{
                        switch (junction) {
                            case 1:
                                drive.setVerticalSlide("zero", false);
                                junction -= 1;
                                break;
                            case 2:
                                drive.setVerticalSlide("lowJunction", false);
                                junction -= 1;
                                break;
                            case 3:
                                drive.setVerticalSlide("mediumJunction", false);
                                junction -= 1;
                                break;
                            default:

                                break;
                        }
                    }catch (Exception ignored) {}
                }
            }
            if (gamepad1.dpad_left) {
                if (currentTime - last_right_bumper_press > 500) { // TODO
                    last_right_bumper_press = System.currentTimeMillis();
                    junction = 3;
                    drive.setVerticalSlide("highJunction", false);
                }
            }
            if (gamepad1.dpad_right) {
                if (currentTime - last_left_bumper_press > 500) { // TODO
                    last_left_bumper_press = System.currentTimeMillis();
                    junction = 0;
                    drive.setVerticalSlide("zero", false);
                }
            }

            if (gamepad1.left_trigger>0.8) {
                if (currentTime - last_left_trigger_press > 200) {
                    last_left_trigger_press = System.currentTimeMillis();
                    drive.setHorizontalSlide("move-100",false);
                }
            }
            if (gamepad1.right_trigger>0.8) {
                if (currentTime - last_right_trigger_press > 200) {
                    last_right_trigger_press = System.currentTimeMillis();
                    drive.setHorizontalSlide("move100",false);
                }
            }


        }
    }
}

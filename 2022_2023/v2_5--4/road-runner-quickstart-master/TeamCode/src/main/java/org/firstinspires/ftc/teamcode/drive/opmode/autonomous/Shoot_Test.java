// Plugin / Config:
// plugin Tetrix motor to plug 0 and config as DC_Motor_Example

// Package
package org.firstinspires.ftc.teamcode;

// Hardware
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

// Opmodes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "DCMotor_test")
public class Shoot_Test extends LinearOpMode {

    // Initializations
    private DcMotor DC_Motor_Example;

    @Override
    public void runOpMode() {

        // Hardware Configurations
        DC_Motor_Example = hardwareMap.get(DcMotor.class, "DC_Motor_Example");

        waitForStart();
        if (opModeIsActive()) {

            // Hardware Settings
            DC_Motor_Example.setDirection(DcMotorSimple.Direction.FORWARD);

            while (!isStopRequested()) {

                // Main Code
                DC_Motor_Example.setPower(1);

            }
        }
    }
}
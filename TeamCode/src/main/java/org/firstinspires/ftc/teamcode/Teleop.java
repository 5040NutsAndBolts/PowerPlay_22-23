package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

@TeleOp(name = "TeleopCode", group = "Teleop")
public class Teleop extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            //drive method call
            robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}

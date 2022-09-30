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
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public boolean bPressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            //drive method call
            robot.drive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

            //slowmode setup
            if(gamepad1.b && !bPressed)
            {
                slowMode = !slowMode;
                bPressed = true;
            }
            else if (!gamepad1.b)
                bPressed = false;

            if(slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;

            telemetry.addData("Slow mode", slowMode);
            telemetry.update();
        }
    }
}

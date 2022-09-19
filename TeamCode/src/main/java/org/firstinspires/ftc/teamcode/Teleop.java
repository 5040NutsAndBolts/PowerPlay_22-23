package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

@TeleOp(name="TeleopCode",group="Teleop")
public class Teleop extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware robot = new Hardware(hardwareMap);
        robot.wheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.wheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive())
        {
            robot.wheelA.setPower(gamepad1.right_trigger);
            robot.wheelB.setPower(gamepad1.right_trigger);
        }
    }
}

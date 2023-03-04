package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "A Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public boolean slightSlow = false;
    public double driveSpeed = 1;
    public double speedNerf;
    public double driveSpeed2 = 1;
    public boolean slowdownOverride = false;
    public boolean cruiseControl = false;

    public boolean y1Pressed = false;
    public boolean b1Pressed = false;
    public boolean a1Pressed = false;
    public boolean y2Pressed = false;
    public boolean bumper2Pressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);
        robot.slideMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //the motors need to be the opposite of what they should be in teleop for autos to work
        //you should never reverse motors here unless you really have to
        robot.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("init done");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        //teleop loop
        while (opModeIsActive())
        {
            //robot oriented drive method call
            if (rDrive)
                robot.robotODrive(gamepad1.left_stick_y * driveSpeed * speedNerf * driveSpeed2,
                        gamepad1.left_stick_x * driveSpeed * speedNerf * driveSpeed2,
                        gamepad1.right_stick_x * driveSpeed * speedNerf * driveSpeed2);

            //field oriented drive method call
            if (!rDrive)
                robot.fieldODrive(gamepad1.left_stick_y * driveSpeed * speedNerf * driveSpeed2,
                        -gamepad1.left_stick_x * driveSpeed * speedNerf * driveSpeed2,
                        gamepad1.right_stick_x * driveSpeed * speedNerf * driveSpeed2,
                        gamepad1.right_stick_button);

            //drive mode toggles
            if (gamepad1.right_bumper)
                rDrive = true;
            if (gamepad1.left_bumper)
                rDrive = false;

            //slowmode setup
            if (gamepad1.b && !b1Pressed)
            {
                slowMode = !slowMode;
                b1Pressed = true;
            }
            else if (!gamepad1.b)
                b1Pressed = false;

            if (slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;

            //slight slowdown
            if (gamepad1.y && !y1Pressed)
            {
                slightSlow = !slightSlow;
                y1Pressed = true;
            }
            else if (!gamepad1.y)
                y1Pressed = false;

            if (slightSlow)
                driveSpeed2 = .9;
            else
                driveSpeed2 = 1;

            //slowdown override
            if(gamepad1.a && !a1Pressed && !gamepad1.start)
            {
                slowdownOverride = !slowdownOverride;
                a1Pressed = true;
            }
            else if (!gamepad1.a)
                a1Pressed = false;

            //slows down the drivetrain when the slides are up
            if(!slowdownOverride)
            {
                if(robot.slideMotorA.getCurrentPosition() <= 900)
                    speedNerf = ((1000 - robot.slideMotorA.getCurrentPosition()) / 3500.0) + .715;
                else if(robot.slideMotorA.getCurrentPosition() <= 1700)
                    speedNerf = ((1800 - robot.slideMotorA.getCurrentPosition()) / 2500.0) + 0.39;
                else if(robot.slideMotorA.getCurrentPosition() <= 2400)
                    speedNerf = ((2500 - robot.slideMotorA.getCurrentPosition()) / 3500.0) + 0.17;
                else
                    speedNerf = ((2700 - robot.slideMotorA.getCurrentPosition()) / 3600.0) + 0.12;
            }
            else
                speedNerf = 1;

            //wheel intake portion
           if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
            {
                if(gamepad2.right_trigger == 0) //intake opening
                {
                    robot.lWheel.setPower(-gamepad2.left_trigger);
                    robot.rWheel.setPower(gamepad2.left_trigger);
                }
                else //intake closing
                {
                    robot.lWheel.setPower(gamepad2.right_trigger);
                    robot.rWheel.setPower(-gamepad2.right_trigger);
                }
            }
           else
            {
                if(gamepad1.right_trigger == 0) //bottom driver intake opening
                {
                    robot.lWheel.setPower(-gamepad1.left_trigger);
                    robot.rWheel.setPower(gamepad1.left_trigger);
                }
                else //bottom driving intake closing
                {
                    robot.lWheel.setPower(gamepad1.right_trigger);
                    robot.rWheel.setPower(-gamepad1.right_trigger);
                }
            }

            //sets transfer override
            if(gamepad2.y && !y2Pressed)
            {
                robot.transferOverride = !robot.transferOverride;
                y2Pressed = true;
            }
            else if(!gamepad2.y)
                y2Pressed = false;

            //transfer level setup
            if(gamepad2.right_bumper && !bumper2Pressed && robot.transferLevel < 3)
            {
                robot.transferLevel++;
                bumper2Pressed = true;
            }
            else if(gamepad2.left_bumper && !bumper2Pressed && robot.transferLevel > 0)
            {
                robot.transferLevel--;
                bumper2Pressed = true;
            }
            else if(!gamepad2.left_bumper && !gamepad2.right_bumper)
                bumper2Pressed = false;

            if(gamepad2.b && !gamepad2.start)
                cruiseControl = true;
            if(gamepad2.left_stick_y != 0)
                cruiseControl = false;

            //transfer mech calls
            if (robot.transferOverride)
            {
                if(!cruiseControl)
                {
                    robot.slideMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    //keeps slides from moving to far to prevent damage
                    if(robot.slideMotorA.getCurrentPosition() < 2600 && gamepad2.left_stick_y < 0)
                    {
                        robot.slideMotorA.setPower(-gamepad2.left_stick_y);
                        robot.slideMotorB.setPower(-gamepad2.left_stick_y);
                    }
                    else if(robot.limitSwitch.getState() && gamepad2.left_stick_y > 0)
                    {
                        robot.slideMotorA.setPower(-gamepad2.left_stick_y * 0.70);
                        robot.slideMotorB.setPower(-gamepad2.left_stick_y * 0.70);
                    }
                    else
                    {
                        robot.slideMotorA.setPower(0);
                        robot.slideMotorB.setPower(0);
                    }
                }
                else
                {
                    robot.slideMotorB.setPower(0);
                    robot.slideMotorA.setTargetPosition(600);
                    if(Math.abs(robot.slideMotorA.getCurrentPosition() - 650) > 75)
                    {
                        robot.slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        robot.slideMotorA.setPower(1);
                    }
                    else
                    {
                        robot.slideMotorA.setPower(0);
                        robot.slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    robot.slideMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
                robot.transfer();

            if(!robot.limitSwitch.getState())
            {
                robot.slideMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("50% Slow Mode", slowMode);
            telemetry.addData("10% Slow Mode", slightSlow);
            telemetry.addData("Slide Slow Down", speedNerf);
            telemetry.addData("Slowdown Override", slowdownOverride);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.addData("Current Drive Speed", driveSpeed * driveSpeed2 * speedNerf);
            telemetry.addLine();
            telemetry.addData("Transfer Level", robot.transferLevel);
            telemetry.addData("Transfer Override", robot.transferOverride);
            telemetry.addData("Slides Resting Position", cruiseControl);
            telemetry.addLine();
            telemetry.addData("Slide Position", robot.slideMotorA.getCurrentPosition());
            telemetry.addData("Limit Switch", robot.limitSwitch.getState());
            telemetry.update();
        }
    }
}

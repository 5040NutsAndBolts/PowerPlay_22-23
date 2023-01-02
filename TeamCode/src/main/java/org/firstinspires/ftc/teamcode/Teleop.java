package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;
    public double speedNerf;
    public boolean slowdownOverride = false;

    public boolean b1Pressed = false;
    public boolean a1Pressed = false;
    public boolean y2Pressed = false;
    public boolean bumper2Pressed = false;
    public boolean slideReset = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Odometry robot = new Odometry(hardwareMap);
        robot.slideMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.resetOdometry(0,0,0);

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
                robot.robotODrive(gamepad1.left_stick_y * driveSpeed * speedNerf,
                        gamepad1.left_stick_x * driveSpeed * speedNerf,
                        gamepad1.right_stick_x * driveSpeed * speedNerf);

            //field oriented drive method call
            if (!rDrive)
                robot.fieldODrive(gamepad1.left_stick_y * driveSpeed * speedNerf,
                        -gamepad1.left_stick_x * driveSpeed * speedNerf,
                        gamepad1.right_stick_x * driveSpeed * speedNerf,
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

            //slowdown override
            if(gamepad1.a && !a1Pressed)
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
                    speedNerf = 1.0;
                else if(robot.slideMotorA.getCurrentPosition() <= 1700)
                    speedNerf = ((2600 - robot.slideMotorA.getCurrentPosition()) / 3200.0) + 0.47;
                else if(robot.slideMotorA.getCurrentPosition() <= 2400)
                    speedNerf = ((3300 - robot.slideMotorA.getCurrentPosition()) / 2000.0) - 0.05;
                else
                    speedNerf = ((3000 - robot.slideMotorA.getCurrentPosition()) / 2000.0) + 0.1;
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

            //transfer mech calls
            if (robot.transferOverride)
            {
                //robot.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //keeps slides from moving to far to prevent damage
                //if slides are changed these lines might cause problems
                if(robot.slideMotorA.getCurrentPosition() < 2800 && gamepad2.left_stick_y < 0)
                {
                    robot.slideMotorA.setPower(-gamepad2.left_stick_y);
                    robot.slideMotorB.setPower(-gamepad2.left_stick_y);
                }
                else if(!robot.limitSwitch.getState() && gamepad2.left_stick_y > 0)
                {
                    robot.slideMotorA.setPower(-gamepad2.left_stick_y * 0.50);
                    robot.slideMotorB.setPower(-gamepad2.left_stick_y * 0.50);
                }
                else
                {
                    robot.slideMotorA.setPower(0);
                    robot.slideMotorB.setPower(0);
                }
            }
            else
                robot.transfer();

            if(robot.limitSwitch.getState())
            {
                robot.slideMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //odo method call, delete after testing
            robot.updatePositionRoadRunner();

            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Slowdown Override", slowdownOverride);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.addLine();
            telemetry.addData("Transfer Level", robot.transferLevel);
            telemetry.addData("Transfer Override", robot.transferOverride);
            telemetry.addLine();
            telemetry.addData("Slide Position", robot.slideMotorA.getCurrentPosition());
            telemetry.addData("Slow Down", speedNerf);
            telemetry.addData("Limit Switch", robot.limitSwitch.getState());
            telemetry.update();

            dashboardTelemetry.addData("left", robot.leftEncoderPos);
            dashboardTelemetry.addData("right", robot.rightEncoderPos);
            dashboardTelemetry.addData("center", robot.centerEncoderPos);
            dashboardTelemetry.addData("x", robot.x);
            dashboardTelemetry.addData("y", robot.y);
            dashboardTelemetry.addData("theta", robot.theta);
            dashboardTelemetry.update();
        }
    }
}
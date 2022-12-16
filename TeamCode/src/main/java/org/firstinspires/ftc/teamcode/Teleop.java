package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;
    public double speedNerf;

    public boolean b1Pressed = false;
    public boolean y2Pressed = false;
    public boolean bumper2Pressed = false;
    public boolean slideReset = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Odometry robot = new Odometry(hardwareMap);
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.resetOdometry(0,0,0);

        telemetry.addLine("init done");
        telemetry.update();

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

            //makes robot slow down when the slides are up
            if(robot.slideMotor.getCurrentPosition() <= 333)
                speedNerf = 1.0;
            else if(robot.slideMotor.getCurrentPosition() <= 1000)
                speedNerf = ((1666 - robot.slideMotor.getCurrentPosition()) / 1333.0);
            else if(robot.slideMotor.getCurrentPosition() <= 1333)
                speedNerf = ((1666 - robot.slideMotor.getCurrentPosition()) / 1333.0);
            else
                speedNerf = ((4000 - robot.slideMotor.getCurrentPosition()) / 13333.0) + .05;

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
                if(robot.slideMotor.getCurrentPosition() < 2000 && gamepad2.left_stick_y < 0)
                  robot.slideMotor.setPower(-gamepad2.left_stick_y);
                else if(!robot.limitSwitch.getState() && gamepad2.left_stick_y > 0)
                    robot.slideMotor.setPower(-gamepad2.left_stick_y);
                else
                    robot.slideMotor.setPower(0);
            }
            else
                robot.transfer();

            if(robot.limitSwitch.getState())
            {
                robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //odo method call, delete after testing
            robot.updatePositionRoadRunner();

            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.addLine();
            telemetry.addData("Transfer Level", robot.transferLevel);
            telemetry.addData("Transfer Override", robot.transferOverride);
            telemetry.addLine();
            telemetry.addData("Slide Position", robot.slideMotor.getCurrentPosition());
            telemetry.addData("Slow Down", speedNerf);
            telemetry.addData("Limit Switch", robot.limitSwitch.getState());
            telemetry.addLine();
            telemetry.addData("left", robot.leftEncoderPos);
            telemetry.addData("right", robot.rightEncoderPos);
            telemetry.addData("center", robot.centerEncoderPos);
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
        }
    }
}
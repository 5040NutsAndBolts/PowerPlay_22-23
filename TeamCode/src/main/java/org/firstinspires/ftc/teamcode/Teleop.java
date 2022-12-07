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

    public boolean b1Pressed = false;
    public boolean y2Pressed = false;
    public boolean bumper2Pressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initializes robot object
        Hardware robot = new Hardware(hardwareMap);
        robot.transferSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        //teleop loop
        while (opModeIsActive())
        {
            //robot oriented drive method call
            if (rDrive)
                robot.robotODrive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

            //field oriented drive method call
            if (!rDrive)
                robot.fieldODrive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.right_stick_button);

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
            } else if (!gamepad1.b)
                b1Pressed = false;

            if (slowMode)
                driveSpeed = .5;
            else
                driveSpeed = 1;

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
            //comment this out if the slides get restrung
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
                robot.transferSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //keeps slides from moving to far to prevent damage
                //if you have to restring the slides remove the if statements and don't use encoder positions
                if(robot.transferSlide.getCurrentPosition() < 4150 && gamepad2.left_stick_y < 0)
                  robot.transferSlide.setPower(-gamepad2.left_stick_y);
                else if(robot.transferSlide.getCurrentPosition() > 200 && gamepad2.left_stick_y > 0)
                    robot.transferSlide.setPower(-gamepad2.left_stick_y);
                else
                    robot.transferSlide.setPower(0);
            }
            else
                robot.transfer();

            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Robot Drive", rDrive);
            telemetry.addLine();
            telemetry.addData("Transfer Level", robot.transferLevel);
            telemetry.addData("Transfer Override", robot.transferOverride);
            telemetry.addLine();
            telemetry.addData("Slide Motor Position", robot.transferSlide.getCurrentPosition());
            telemetry.addData("Slide Target Position", robot.transferSlide.getTargetPosition());
            telemetry.addData("Slide Motor Power", robot.transferSlide.getPower());
            telemetry.update();
        }
    }
}

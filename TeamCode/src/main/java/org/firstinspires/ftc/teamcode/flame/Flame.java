package org.firstinspires.ftc.teamcode.flame;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Flame", group = "Teleop")
public class Flame extends OpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public boolean b1Pressed = false;

    public BNO055IMU imu;
    public double adjust;

    @Override
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");
    }

    //robot-oriented drive method
    public void robotODrive(double forward, double sideways, double rotation)
    {
        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1)
        {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }

    //field oriented drive
    public void fieldODrive(double forward, double sideways, double rotation, boolean reset)
    {
        double P = Math.hypot(sideways, forward);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double robotAngle = Math.atan2(forward, sideways);

        if (reset)
            adjust = angles.firstAngle;

        double v5 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v6 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;
        double v7 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v8 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;

        frontLeft.setPower(v5);
        frontRight.setPower(v6);
        backLeft.setPower(v7);
        backRight.setPower(v8);
    }

    @Override
    public void loop()
    {
        //robot oriented drive method call
        if (rDrive)
            robotODrive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

        //field oriented drive method call
        if (!rDrive)
            fieldODrive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.right_stick_button);

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
    }
}
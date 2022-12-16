package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class Hardware
{
    //drive motor declaration
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx slideMotor;
    public CRServo lWheel;
    public CRServo rWheel;

    public BNO055IMU imu;
    public double adjust;

    public DigitalChannel limitSwitch;

    //tracking variables
    public int transferLevel = 0;
    public boolean transferOverride = true;

    //helper class variables
    public double x = 0, y = 0, theta = 0;
    public static LinearOpMode currentOpMode;

    public DcMotorEx leftOdom, rightOdom, centerOdom;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    public static final double ODOM_TICKS_PER_IN = 1945.083708;
    public static double trackwidth = 10.56198075;

    //constructor method
    public Hardware(HardwareMap hardwareMap)
    {
        //drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        slideMotor = hardwareMap.get(DcMotorEx.class, "Slide Motor");

        lWheel = hardwareMap.crservo.get("Left Wheel");
        rWheel = hardwareMap.crservo.get("Right Wheel");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "Limit Switch");

        //odom
        leftOdom = hardwareMap.get(DcMotorEx.class, "Front Left");
        rightOdom = hardwareMap.get(DcMotorEx.class, "Front Right");
        centerOdom = hardwareMap.get(DcMotorEx.class, "Back Right");
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

    //method to run slides
    public void transfer()
    {
        if ((transferLevel == 0))
        {
            if (slideMotor.getCurrentPosition() < 100)
            {
                slideMotor.setTargetPosition(0);
                slideMotor.setPower(0);
                if (slideMotor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else
            {
                slideMotor.setTargetPosition(0);
                slideMotor.setPower(1);
            }
        }
        else if (transferLevel == 1)
        {
            if (slideMotor.getCurrentPosition() < 1000 || slideMotor.getCurrentPosition() > 1050)
            {
                slideMotor.setTargetPosition(1000);
                slideMotor.setPower(1);
            }
            else
            {
                slideMotor.setPower(0);
                if (slideMotor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else if (transferLevel == 2)
        {
            if (slideMotor.getCurrentPosition() < 1350 || slideMotor.getCurrentPosition() > 1450)
            {
                slideMotor.setPower(1);
                slideMotor.setTargetPosition(1400);
            }
            else
            {
                slideMotor.setPower(0);
                if (slideMotor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else
        {
            if (slideMotor.getCurrentPosition() > 1900)
            {
                slideMotor.setTargetPosition(1950);
                slideMotor.setPower(0);
                if (slideMotor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else
            {
                slideMotor.setTargetPosition(1950);
                slideMotor.setPower(1);
            }
        }
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

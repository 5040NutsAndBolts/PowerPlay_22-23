package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
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

import static java.lang.Math.abs;

import java.util.ArrayList;
import java.util.List;

//class header, MechanumDrive extend is needed for RoadRunner
public class Hardware extends MecanumDrive
{
    //drive motor declaration
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx slideMotorA;
    public DcMotorEx slideMotorB;
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
        super(0.0199,0.0055,0.0,10.56198075, 11.97, 1);

        //drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        slideMotorA = hardwareMap.get(DcMotorEx.class, "Slide Motor A");
        slideMotorB = hardwareMap.get(DcMotorEx.class, "Slide Motor B");

        lWheel = hardwareMap.crservo.get("Left Wheel");
        rWheel = hardwareMap.crservo.get("Right Wheel");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorA.setDirection(DcMotorSimple.Direction.REVERSE);

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
            if (slideMotorA.getCurrentPosition() < 100)
            {
                slideMotorA.setTargetPosition(0);
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else
            {
                slideMotorA.setTargetPosition(0);
                slideMotorA.setPower(1);
            }
        }
        else if (transferLevel == 1)
        {
            if (slideMotorA.getCurrentPosition() < 1000 || slideMotorA.getCurrentPosition() > 1050)
            {
                slideMotorA.setTargetPosition(1000);
                slideMotorA.setPower(1);
            }
            else
            {
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else if (transferLevel == 2)
        {
            if (slideMotorA.getCurrentPosition() < 1350 || slideMotorA.getCurrentPosition() > 1450)
            {
                slideMotorA.setPower(1);
                slideMotorA.setTargetPosition(1400);
            }
            else
            {
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else
        {
            if (slideMotorA.getCurrentPosition() > 1900)
            {
                slideMotorA.setTargetPosition(1950);
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else
            {
                slideMotorA.setTargetPosition(1950);
                slideMotorA.setPower(1);
            }
        }
        slideMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //all methods below this have to be here to avoid errors and should be ignored
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3)
    {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        return wheelPositions;
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}

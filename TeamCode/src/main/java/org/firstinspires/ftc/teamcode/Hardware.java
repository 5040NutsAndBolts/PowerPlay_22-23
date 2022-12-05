package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Math.abs;

public class Hardware
{
    //drive motor declaration
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx transferSlide;
    public CRServo lWheel;
    public CRServo rWheel;

    public BNO055IMU imu;
    public double adjust;

    //tracking variables
    public int transferLevel = 0;
    public boolean transferOverride = true;

    //helper class variables
    public double x = 0, y = 0, theta = 0;
    public static LinearOpMode currentOpMode;

    //odo stuff
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;

    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    public static final double ODOM_TICKS_PER_IN = 1898.130719;
    public static double trackwidth = 10.39701829;

    //constructor method
    public Hardware(HardwareMap hardwareMap)
    {
        //drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        transferSlide = hardwareMap.get(DcMotorEx.class, "Slide Motor");

        lWheel = hardwareMap.crservo.get("Left Wheel");
        rWheel = hardwareMap.crservo.get("Right Wheel");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        transferSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //odo
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        //remeber to change these when I get the odo pods on
        leftOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Front Left");
        rightOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Front Right");
        centerOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Back Left");
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
            if (transferSlide.getCurrentPosition() < 100)
            {
                transferSlide.setTargetPosition(0);
                transferSlide.setPower(0);
                if (transferSlide.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
                    transferSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else
            {
                transferSlide.setTargetPosition(0);
                transferSlide.setPower(1);
            }
        }
        else if (transferLevel == 1)
        {
            if (transferSlide.getCurrentPosition() < 1950 || transferSlide.getCurrentPosition() > 2050)
            {
                transferSlide.setTargetPosition(2000);
                transferSlide.setPower(1);
            }
            else
            {
                transferSlide.setPower(0);
                if (transferSlide.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    transferSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else if (transferLevel == 2)
        {
            if (transferSlide.getCurrentPosition() < 3450 || transferSlide.getCurrentPosition() > 3550)
            {
                transferSlide.setPower(1);
                transferSlide.setTargetPosition(3500);
            }
            else
            {
                transferSlide.setPower(0);
                if (transferSlide.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    transferSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        else
        {
            if (transferSlide.getCurrentPosition() > 4700)
            {
                transferSlide.setTargetPosition(4750);
                transferSlide.setPower(0);
                if (transferSlide.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                    transferSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else
            {
                transferSlide.setTargetPosition(4750);
                transferSlide.setPower(1);
            }
        }
        transferSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

package org.firstinspires.ftc.teamcode.flame;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FlameHardware
{
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    ColorSensor leftLineSensor, centerLineSensor, rightLineSensor;

    public BNO055IMU imu;
    public double adjust;

    public double x = 0, y = 0, theta = 0;
    public static LinearOpMode currentOpMode;

    public DcMotorEx leftOdom, rightOdom, centerOdom;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    public static final double ODOM_TICKS_PER_IN = 1945.083708;
    public static double trackwidth = 10.56198075;

    //failsafe stuff
    boolean set = false;
    boolean ready = false;
    int[] leftReadings = new int[3];
    int[] rightReadings = new int[3];
    int[] centerReadings = new int[3];

    public FlameHardware(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //color sensors
        leftLineSensor = hardwareMap.colorSensor.get("Left Line Sensor");
        centerLineSensor = hardwareMap.colorSensor.get("Center Line Sensor");
        rightLineSensor = hardwareMap.colorSensor.get("Right Line Sensor");

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

    public boolean failSafe(ElapsedTime matchTime)
    {
        if(matchTime.seconds() % 3 < .1 && matchTime.seconds() > .5)
        {
            leftReadings[2] = leftOdom.getCurrentPosition();
            rightReadings[2] = rightOdom.getCurrentPosition();
            centerReadings[2] = centerOdom.getCurrentPosition();

            ready = true;
        }
        else if(matchTime.seconds() % 1 < .1 && !set)
        {
            leftReadings[0] = leftOdom.getCurrentPosition();
            rightReadings[0] = rightOdom.getCurrentPosition();
            centerReadings[0] = centerOdom.getCurrentPosition();

            set = true;
        }
        else if(matchTime.seconds() % 1 < .1 && set)
        {
            leftReadings[1] = leftOdom.getCurrentPosition();
            rightReadings[1] = rightOdom.getCurrentPosition();
            centerReadings[1] = centerOdom.getCurrentPosition();
        }

        if(ready)
        {
            if(Math.abs(leftReadings[0] - leftReadings[1]) < 100 &&
                    Math.abs(leftReadings[1] - leftReadings[2]) < 100)
            {
                return true;
            }
            if(Math.abs(rightReadings[0] - rightReadings[1]) < 100 &&
                    Math.abs(rightReadings[1] - rightReadings[2]) < 100)
            {
                return true;
            }

            ready = false;
            set = false;
        }

        return false;
    }
}

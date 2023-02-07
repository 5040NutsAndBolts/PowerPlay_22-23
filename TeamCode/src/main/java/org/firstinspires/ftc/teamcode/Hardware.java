package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //used in our odo but not RoadRunner classes
    public static final double ODOM_TICKS_PER_IN = 1831.471685;
    public static double trackwidth = 10.976;

    //failsafe stuff
    public boolean set = false;
    public boolean ready = false;
    public int[] leftReadings = new int[3];
    public int[] rightReadings = new int[3];
    public int[] centerReadings = new int[3];

    //constructor method
    public Hardware(HardwareMap hardwareMap)
    {
        super(0.0185,0.005,0.0,10.976, 12.57, 1.017);

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
        if(slideMotorA.getVelocity() > 400)
            slideMotorB.setPower(1);
        else if(slideMotorA.getVelocity() < -400)
            slideMotorB.setPower(-1);
        else
            slideMotorB.setPower(0);

        if ((transferLevel == 0))
        {
            if (slideMotorA.getCurrentPosition() < 100)
            {
                slideMotorA.setTargetPosition(0);
                slideMotorA.setPower(0);
                slideMotorB.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
                {
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            else
            {
                slideMotorA.setTargetPosition(0);
                slideMotorA.setPower(1);
            }
        }
        else if (transferLevel == 1)
        {

            if (slideMotorA.getCurrentPosition() < 1050 || slideMotorA.getCurrentPosition() > 1150)
            {
                slideMotorA.setTargetPosition(1100);
                slideMotorA.setPower(1);
            }
            else
            {
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                {
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
        else if (transferLevel == 2)
        {
            if (slideMotorA.getCurrentPosition() < 1650 || slideMotorA.getCurrentPosition() > 1750)
            {
                slideMotorA.setPower(1);
                slideMotorA.setTargetPosition(1700);
            }
            else
            {
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                {
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
        else
        {
            if (slideMotorA.getCurrentPosition() > 2500)
            {
                slideMotorA.setTargetPosition(2550);
                slideMotorA.setPower(0);
                if (slideMotorA.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
                {
                    slideMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slideMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else
            {
                slideMotorA.setTargetPosition(2550);
                slideMotorA.setPower(1);
            }
        }
        slideMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //auto method for intaking and depositing
    public void depositCone()
    {
        ElapsedTime t = new ElapsedTime();
        t.startTime();

        while(t.seconds() < 0.5)
        {
            rWheel.setPower(1);
            lWheel.setPower(-1);
        }
        rWheel.setPower(0);
        lWheel.setPower(0);
    }

    public void intakeCone()
    {
        ElapsedTime t = new ElapsedTime();
        t.startTime();

        while(t.seconds() < 0.5)
        {
            rWheel.setPower(-1);
            lWheel.setPower(1);
        }
        rWheel.setPower(0);
        lWheel.setPower(0);
    }

    public void runCounterSpin(boolean spin)
    {
        if(spin)
        {
            rWheel.setPower(-0.15);
            lWheel.setPower(0.15);
        }
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
            if(Math.abs(leftReadings[0] - leftReadings[1]) < 50 &&
                    Math.abs(leftReadings[1] - leftReadings[2]) < 50)
            {
                return true;
            }
            if(Math.abs(rightReadings[0] - rightReadings[1]) < 50 &&
                    Math.abs(rightReadings[1] - rightReadings[2]) < 50)
            {
                return true;
            }

            ready = false;
            set = false;
        }

        return false;
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

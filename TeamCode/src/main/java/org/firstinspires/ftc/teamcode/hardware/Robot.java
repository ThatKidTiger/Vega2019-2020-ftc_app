package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    /* Public OpMode members. */
    public DcMotor  backLeft; // hub 3 port 0
    public DcMotor  backRight; // hub 3 port 1
    public DcMotor  frontLeft; // hub 3 port 3
    public DcMotor  frontRight; //hub 3 port 2
    public DcMotor latch; //hub 2 port 0
    public DcMotor lift; //hub 2 port 1
    public DcMotor gripper; //hub 2 port 2
    public BNO055IMU imu;
    public ColorSensor colLeft; //hub 2 port 0
    public ColorSensor colRight; //hub 2 port 2
    public DistanceSensor distance; //hub 2 port 1
    public DistanceSensor topdistance; //hub 3 port 0

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public Robot() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");

        // Set all drive motors to run without encoders.
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse left drive motors
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        latch = hwMap.get(DcMotor.class, "latch");
        lift = hwMap.get(DcMotor.class, "lift");

        /*lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);*/

        gripper = hwMap.get(DcMotor.class, "gripper");

        //define sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        colLeft = hwMap.get(ColorSensor.class, "colLeft");
        colRight = hwMap.get(ColorSensor.class, "colRight");
        distance = hwMap.get(DistanceSensor.class, "distance");
        topdistance = hwMap.get(DistanceSensor.class, "topdistance");

        //Configure IMU_CONSTANTS Sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "IMUCalibration.json";

        imu.initialize(parameters);

        // Set all motors to zero power
        backLeft.setPower(0); //hub 1 0
        backRight.setPower(0); //hub 1 2
        frontLeft.setPower(0); //hub 1 1
        frontRight.setPower(0); //hub 1 3
    }
}

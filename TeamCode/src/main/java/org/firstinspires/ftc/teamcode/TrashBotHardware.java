package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Team 8018
 * Theoretically Impossible
 * Robot Hardware
 */

public class TrashBotHardware {
    // Public OpMode members
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor midMotor = null;
    public DcMotor clawLift = null;
    public Servo glyphTopRight = null;
    public Servo glyphBottomRight = null;
    public Servo glyphLeft = null;

    // local (private) OpMode members
    HardwareMap hwMap = null;

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize Motors
        leftMotor = hwMap.get(DcMotor.class, "leftWheel");
        rightMotor = hwMap.get(DcMotor.class, "rightWheel");
        midMotor = hwMap.get(DcMotor.class, "midWheel");
        clawLift = hwMap.get(DcMotor.class, "clawLift");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        midMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        midMotor.setPower(0);
        clawLift.setPower(0);

        // Set all motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize all installed servos
        glyphTopRight = hwMap.get(Servo.class, "glyphTopRight");
        glyphLeft = hwMap.get(Servo.class, "glyphLeft");
        glyphBottomRight = hwMap.get(Servo.class, "glyphBottomRight");

        //Define and initialize the REV color sensor
    }
}


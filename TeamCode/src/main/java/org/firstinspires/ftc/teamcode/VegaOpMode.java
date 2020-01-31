package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="VegaOpMode", group="VegaBot")
public class VegaOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    VegaHardware robot = new VegaHardware();

    Orientation lastAngles = new Orientation();
    double relativeAngle, globalAngle;

    //controls driver 2 strafe toggle
    boolean twoStrafe = false;
    boolean xChange = false;
    boolean open = true;
    boolean aChange = false;
    boolean foundation = false;
    boolean yChange = false;
    boolean downChange = false;
    boolean upChange = false;
    int liftTarget = 0;

    MiniPID liftPID = new MiniPID(0.002, 0, 0.0);

    @Override
    public void init() {
        runtime.startTime();
        robot.init(hardwareMap);
        robot.imu.readCalibrationData();
        telemetry.addData("Status", "Initialized");

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        runtime.reset();

        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double strafe = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        if (gamepad2.x && !xChange) {
            xChange = true;
            twoStrafe = !twoStrafe;
        } else if (!gamepad2.x) {
            xChange = false;
        }

        if (twoStrafe) {
            //player two slow strafe mode
            double strafeTwo = gamepad2.right_stick_x * 0.4;
            robot.backLeft.setPower(-strafeTwo);
            robot.backRight.setPower(strafeTwo);
            robot.frontLeft.setPower(strafeTwo);
            robot.frontRight.setPower(-strafeTwo);
        } else if (foundation) {
            //player one slow drive foundation
            if(Math.abs(gamepad1.right_stick_x) < 0.1) {
                leftPower *= 0.35;
                rightPower *= 0.35;
                robot.backLeft.setPower(leftPower);
                robot.backRight.setPower(rightPower);
                robot.frontLeft.setPower(leftPower);
                robot.frontRight.setPower(rightPower);
            }
            else {
                double strafeSlow = gamepad1.right_stick_x * 0.4;
                robot.backLeft.setPower(-strafeSlow);
                robot.backRight.setPower(strafeSlow);
                robot.frontLeft.setPower(strafeSlow);
                robot.frontRight.setPower(-strafeSlow);
            }
        } else if (Math.abs(strafe) < 0.1) {
            //arcade drive
            robot.backLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.frontRight.setPower(rightPower);
        } else {
            if (Math.abs(rightY) < .15) {
                //strafe if gamepad1 right stick is in the horizontal zone
                robot.backLeft.setPower(-strafe);
                robot.backRight.setPower(strafe);
                robot.frontLeft.setPower(strafe);
                robot.frontRight.setPower(-strafe);
            } else {
                //diagonal strafe if gamepad1 right stick is in the diagonal zones
                rightPower = strafe + rightY;
                leftPower = rightY - strafe;

                double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
                if (max > 1) {
                    rightPower /= max;
                    leftPower /= max;
                }
                robot.backLeft.setPower(leftPower);
                robot.backRight.setPower(rightPower);
                robot.frontLeft.setPower(rightPower);
                robot.frontRight.setPower(leftPower);
            }
        }

        if(gamepad2.right_trigger > 0) {
            liftTarget -= Math.round(gamepad2.right_trigger * 7);
        }
        else if(gamepad2.left_trigger > 0) {
            liftTarget += Math.round(gamepad2.right_trigger * 7);
        }

        if(gamepad2.dpad_up && !upChange) {
            upChange = true;
            liftTarget -= 582;
        }
        else if(!gamepad2.dpad_up) {
            upChange = false;
        }

        if(gamepad2.dpad_down && !downChange) {
            downChange = true;
            liftTarget += 582;
        }
        else if(!gamepad2.dpad_down) {
            downChange = false;
        }

        if(gamepad2.right_bumper) {
            robot.gripper.setPower(-0.30);
        }
        else if(gamepad2.left_bumper) {
            robot.gripper.setPower(0.30);
        }
        else {
            robot.gripper.setPower(0);
        }

        robot.lift.setTargetPosition(liftTarget);
        /*if(Math.abs(robot.lift.getTargetPosition() - robot.lift.getCurrentPosition()) > 5) {
            double liftpower = liftPID.getOutput(robot.lift.getCurrentPosition(), liftTarget);
            if(Math.abs(liftpower) < 0.25) {
                liftpower *= 0.25/Math.abs(liftpower);
            }
            robot.lift.setPower(liftpower);
        }
        else {
            robot.lift.setPower(0);
        }*/

        robot.lift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        robot.latch.setPower(0.3 * (gamepad1.right_trigger - gamepad1.left_trigger));

        if(gamepad2.a && !aChange) {
            aChange = true;
            open = !open;
        }
        else if(!gamepad2.a) {
            aChange = false;
        }

        if(!open) {
            robot.gripper.setPower(-0.1);
            telemetry.addLine("Closed");
        }
        else {
            telemetry.addLine("Open");
        }

        if (gamepad1.y && !yChange) {
            yChange = true;
            foundation = !foundation;
        }
        else if(!gamepad2.y) {
            yChange = false;
        }

        telemetry.addData("Time: ", runtime.milliseconds());
        telemetry.addData("IMU Calib", robot.imu.getCalibrationStatus().toString());
        telemetry.addData("Distance(cm): ", "%f", robot.distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Top Distance(cm): ", "%f", robot.topdistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Angle: ", getOrientation());
        telemetry.addData("Left R,G,B,A: ", "%d, %d, %d, %d", robot.colLeft.red(), robot.colLeft.green(), robot.colLeft.blue(), robot.colLeft.alpha());
        telemetry.addData("Right R,G,B,A: ", "%d, %d, %d, %d", robot.colRight.red(), robot.colRight.green(), robot.colRight.blue(), robot.colRight.alpha());
        telemetry.addData("Latch: ", "%d %d", robot.latch.getCurrentPosition(), robot.latch.getTargetPosition());
        telemetry.addData("Lift: ","%d, %d" , robot.lift.getCurrentPosition(), robot.lift.getTargetPosition());
        telemetry.addData("Lift Power: ", gamepad2.right_trigger - gamepad2.left_trigger);
    }

    @Override
    public void stop() {
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        relativeAngle = 0;
        globalAngle = lastAngles.firstAngle;
    }

    private double getOrientation() {
        //retrieve current imu orientation information
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //calculate change from the last recorded position
        double deltaAngle = angles.firstAngle - globalAngle;

        //because IMU returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        lastAngles = angles;
        return deltaAngle;
    }

    private double inchestoTicks(double TICKS_PER_REV, double DIAM, double inches) {
        return (inches / DIAM * Math.PI) * TICKS_PER_REV;
    }
}
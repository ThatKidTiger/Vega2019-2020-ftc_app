package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name="VegaOpMode", group="VegaBot")
public class VegaOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /*
    Todo: Integrate all subsystems into the robot class
     */
    //Robot robot = new Robot();
	private GamepadController controllers;
    private MecanumDrive drive = new MecanumDrive();

    Orientation lastAngles = new Orientation();
    double relativeAngle, globalAngle;

    //boolean downChange = false;
    //boolean upChange = false;
    int liftTarget = 0;

    int opened;
    int closed;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    MiniPID liftPID = new MiniPID(0.002, 0, 0.0);

    @Override
    public void init() {
        runtime.startTime();
        //robot.init(hardwareMap);
        //robot.imu.readCalibrationData();

        drive.init(hardwareMap);

        controllers = new GamepadController(gamepad1, gamepad2);

        int opened = hardwareMap.appContext.getResources().getIdentifier("open", "raw", hardwareMap.appContext.getPackageName());
        int closed = hardwareMap.appContext.getResources().getIdentifier("closed", "raw", hardwareMap.appContext.getPackageName());
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        runtime.reset();

        double[] powers = controllers.getDrivePowers();

        drive.setMotorPowers(powers);

        /*
        Todo: add remaining subsystem
         */
        //robot.gripper.setPower(-gamepad2.left_stick_y * 0.25);

        /*
        if(gamepad2.a) {
            rotate(IMU_CONSTANTS.angle);
        }

        if(gamepad2.x) {
            moveToTop(DIST_CONSTANTS.power, DIST_CONSTANTS.dist);
        }
        */

        /*if(gamepad2.right_trigger > 0) {
            liftTarget -= Math.round(gamepad2.right_trigger * 7);
        }
        else if(gamepad2.left_trigger > 0) {
            liftTarget += Math.round(gamepad2.right_trigger * 7);
        }*/

        /*if(gamepad2.dpad_up && !upChange) {
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
        }*/

        //robot.lift.setTargetPosition(liftTarget);
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

        /*
        robot.lift.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.8);

        robot.latch.setPower(0.3 * (gamepad1.right_trigger - gamepad1.left_trigger));
         */

        /*if(gamepad2.a && !aChange) {
            aChange = true;
            if(open) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, closed);
            }
            else {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, opened);
            }
            open = !open;
        }
        else if(!gamepad2.a) {
            aChange = false;
        }

        if(!open) {
            robot.gripper.setPower(-0.2);
        }*/

        packet.put("Time: ", runtime.milliseconds());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("Distance(cm): ", robot.distance.getDistance(DistanceUnit.CM));
        //packet.put("Top Distance(cm): ", robot.topdistance.getDistance(DistanceUnit.CM));
        //packet.put("Angle: ", getOrientation());
        //telemetry.addData("Left R,G,B,A: ", "%d %d %d %d", robot.colLeft.red(), robot.colLeft.green(), robot.colLeft.blue(), robot.colLeft.alpha());
        //telemetry.addData("Right R,G,B,A: ", "%d, %d, %d, %d", robot.colRight.red(), robot.colRight.green(), robot.colRight.blue(), robot.colRight.alpha());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }

    /*
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

        //because IMU_CONSTANTS returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        lastAngles = angles;
        return deltaAngle;
    }

    public double getAngle() {
        //retrieve current imu orientation information
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //calculate change from the last recorded position
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //because IMU_CONSTANTS returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        deltaAngle = limitAxes(deltaAngle);

        //change the overall deviation from the initial orientation
        relativeAngle += deltaAngle;

        //set the lastAngle to current angle
        lastAngles = angles;

        return relativeAngle;
    }

    private double limitAxes(double orientation) {
        if (orientation < -180)
            orientation += 360;
        else if (orientation > 180)
            orientation -= 360;
        return orientation;
    }

    private void rotate(double degrees) {
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        runtime.reset();
        resetAngle();

        MiniPID PD = new MiniPID(IMU_CONSTANTS.ROT_PID.p, IMU_CONSTANTS.ROT_PID.i, IMU_CONSTANTS.ROT_PID.d);
        sleep(250);

        double lastangle = 0;
        double angle = getAngle();
        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - angle) > 0.1 || Math.abs(degrees - lastangle) > 0.1) {
            lastangle = angle;
            angle = getAngle();
            temppower = PD.getOutput(angle, degrees);
            packet.put("Orientation", angle);

            if(Math.abs(temppower) < IMU_CONSTANTS.ROT_MIN) {
                temppower *= IMU_CONSTANTS.ROT_MIN/Math.abs(temppower);
            }

            if(angle == 0) {
                temppower = Math.signum(degrees) * IMU_CONSTANTS.ROT_MIN;
            }

            leftPower = -temppower;
            rightPower = temppower;

            robot.frontRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    private void moveToTop(double power, double dist) {
        //Accurate once it moves within 120 centimeters of the object it is approaching
        double leftPower, rightPower, temppower;

        //reset reference angle and PID controllers
        resetAngle();
        runtime.reset();
        MiniPID PD = new MiniPID(DIST_CONSTANTS.DIST_PID.p , DIST_CONSTANTS.DIST_PID.i, DIST_CONSTANTS.DIST_PID.d);

        //continues to move until the distance sensor returns it is within a margin of error
        double current = robot.topdistance.getDistance(DistanceUnit.CM);
        double lastdist = current;
        while(Math.abs(dist - robot.topdistance.getDistance(DistanceUnit.CM)) > 0.1 || Math.abs(dist - lastdist) > 0.1) {
            lastdist = current;
            current = robot.topdistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(Math.abs(temppower) < DIST_CONSTANTS.DIST_MIN) {
                temppower *= (DIST_CONSTANTS.DIST_MIN/temppower);
            }

            leftPower = -temppower;
            rightPower = -temppower;

            while(robot.topdistance.getDistance(DistanceUnit.CM) > 120){
                robot.frontRight.setPower(power);
                robot.frontLeft.setPower(power);
                robot.backRight.setPower(power);
                robot.backLeft.setPower(power);
            }
            robot.frontRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }
    */
}
package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Latch;
import org.firstinspires.ftc.teamcode.subsystems.LiftIntake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.DIST_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.DIST_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.DIST_PID;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.STRAIGHT_PID;

/*
Todo: implement color sensor methods
Todo: Integrate all subsystems into the robot class
Todo: Run all systems as state machines, with update commands as transition states
Todo: pass the dashboard object reference to robot, and then retrieve the various subsystem updates
 */

public class Robot {
    public static final String TAG = "Robot";
    ElapsedTime runtime = new ElapsedTime();

    ArrayList<Subsystem> subsystems = new ArrayList();
    private IMU imu = new IMU();
    private MecanumDrive drive = new MecanumDrive();
    private Latch latch = new Latch();
    private LiftIntake liftIntake = new LiftIntake();
    public ColorSensor colLeft;
    public ColorSensor colRight;
    public DistanceSensor distance;
    public DistanceSensor topDistance;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    /* Constructor */
    public Robot() {
        subsystems.add(drive);
        subsystems.add(imu);
        subsystems.add(latch);
        subsystems.add(liftIntake);
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        Map<String, Object> updates = new HashMap<>();
        for(Subsystem subsystem : subsystems) {
            Map<String, Object> updatePacket = subsystem.update();
            if(updatePacket != null) {
                updates.putAll(updatePacket);
            }
        }
        packet.put("Distance", distance.getDistance(DistanceUnit.CM));
        packet.put("Top Distance", topDistance.getDistance(DistanceUnit.CM));
        packet.putAll(updates);
        dashboard.sendTelemetryPacket(packet);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        imu.init(hwMap);
        drive.init(hwMap);
        latch.init(hwMap);
        liftIntake.init(hwMap);
        colLeft = hwMap.get(ColorSensor.class, "colLeft");
        colRight = hwMap.get(ColorSensor.class, "colRight");
        distance = hwMap.get(DistanceSensor.class, "distance");
        topDistance = hwMap.get(DistanceSensor.class, "topDistance");
        runtime.startTime();
    }

    //region Auton methods
    public void forward(double power) {
        Log.d(TAG, String.format("Moving " + (power > 0 ? "forward" : "backward") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, power, power, power};
        drive.setMotorPowers(powers);
    }

    public void strafe(double power) {
        Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, -power, power, -power};
        drive.setMotorPowers(powers);
    }

    public void rotateByPower(double power) {
        Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, power, - power, - power};
        drive.setMotorPowers(powers);
    }

    //uses PID controller to precisely turn a certain number of degrees
    public void rotateByAngle(double degrees) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();
        MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
        double leftPower, rightPower, temppower;

        imu.resetAngularDistance();
        sleep(250);

        double lastAngle = 0;
        double angle = imu.getAngularDistance();

        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - angle) > 0.1 || Math.abs(degrees - lastAngle) > 0.1) {
            packet.put("Reference", degrees);
            update();
            lastAngle = angle;
            angle = imu.getAngularDistance();
            temppower = imuPID.getOutput(angle, degrees);

            if(Math.abs(temppower) < ROT_MIN) {
                temppower *= ROT_MIN/Math.abs(temppower);
            } else if(Math.abs(temppower) > ROT_MAX) {
                temppower *= ROT_MAX/Math.abs(temppower);
            }

            if(angle == 0) {
                temppower = Math.signum(degrees) * ROT_MIN;
            }

            leftPower = -temppower;
            rightPower = temppower;

            double[] powers = {leftPower, leftPower, rightPower, rightPower};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Rotation: " + runtime.milliseconds());
    }

    public void stop() {
        double[] powers = {0, 0, 0, 0};
        drive.setMotorPowers(powers);
    }

    public void latchDown(){
        runtime.reset();
        while (runtime.milliseconds() < 400){
            latch.setPower(.8);
        }
        latch.setPower(0);
    }

    public void latchUp(){
        runtime.reset();
        while (runtime.milliseconds() < 400){
            latch.setPower(-.8);
        }
        latch.setPower(0);
    }

    public void grab() {
        runtime.reset();
        while (runtime.milliseconds() < 500){
            liftIntake.setGripperPower(-0.2);
        }
        liftIntake.setGripperPower(0);
    }

    public void release() {
        runtime.reset();
        while (runtime.milliseconds() < 500){
            liftIntake.setGripperPower(0.2);
        }
        liftIntake.setGripperPower(0);
    }

    public void moveToTopDistance(double dist) {
        //Accurate once it moves within 120 centimeters of the object it is approaching
        double leftPower, rightPower, temppower, adjustment;

        //reset reference angle and PID controllers
        runtime.reset();
        imu.resetOrientation();
        MiniPID distancePID = new MiniPID(DIST_PID.p, DIST_PID.i, DIST_PID.d);
        MiniPID straightPID = new MiniPID(STRAIGHT_PID.p, STRAIGHT_PID.i, STRAIGHT_PID.d);

        while(topDistance.getDistance(DistanceUnit.CM) > 120){
            drive.setMotorPowers(new double[] {DIST_MAX, DIST_MAX, DIST_MAX, DIST_MAX});
        }

        double lastDistance = 0;
        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - topDistance.getDistance(DistanceUnit.CM)) > 0.5 || Math.abs(dist - lastDistance) > 0.5 || runtime.seconds() > 4) {
            packet.put("Reference", dist);
            update();
            double current = topDistance.getDistance(DistanceUnit.CM);
            temppower = distancePID.getOutput(current, dist);
            adjustment = straightPID.getOutput(imu.getOrientation(), 0);

            leftPower = -temppower - (Math.abs(temppower) * adjustment);
            rightPower = -temppower + (Math.abs(temppower) * adjustment);

            double[] powers = {leftPower, leftPower, rightPower, rightPower};
            powers = limitPowers(powers, DIST_MIN, DIST_MAX);

            drive.setMotorPowers(powers);
            lastDistance = current;
        }
        stop();
    }

    //Todo: are left and right powers necessary, or can negative temppower just be used
    public void moveToDistance(double dist) {
        //Accurate once it moves within 120 centimeters of the object it is approaching
        double leftPower, rightPower, temppower, adjustment;

        //reset reference angle and PID controllers
        runtime.reset();
        MiniPID distancePID = new MiniPID(DIST_PID.p, DIST_PID.i, DIST_PID.d);
        MiniPID straightPID = new MiniPID(STRAIGHT_PID.p, STRAIGHT_PID.i, STRAIGHT_PID.d);

        while(distance.getDistance(DistanceUnit.CM) > 120){
            drive.setMotorPowers(new double[] {DIST_MAX, DIST_MAX, DIST_MAX, DIST_MAX});
        }

        double lastDistance = 0;
        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - distance.getDistance(DistanceUnit.CM)) > 0.5 || Math.abs(dist - lastDistance) > 0.5 || runtime.seconds() > 4) {
            packet.put("Reference", dist);
            update();
            double current = distance.getDistance(DistanceUnit.CM);
            temppower = distancePID.getOutput(current, dist);
            adjustment = straightPID.getOutput(imu.getOrientation(), 0);

            leftPower = -temppower - (Math.abs(temppower) * adjustment);
            rightPower = -temppower + (Math.abs(temppower) * adjustment);

            double[] powers = {leftPower, leftPower, rightPower, rightPower};
            powers = limitPowers(powers, DIST_MIN, DIST_MAX);

            drive.setMotorPowers(powers);
            lastDistance = current;
        }
        stop();
    }

    private double[] limitPowers(double[] powers, double min, double max) {
        double maxPower = max(powers);
        double minPower = min(powers);
        if(minPower < min) {
            for(int i = 0; i < powers.length; i++) {
                powers[i] *= min/Math.abs(minPower);
            }
        }

        if(maxPower > max) {
            for(int i = 0; i < powers.length; i++) {
                powers[i] *= max/Math.abs(maxPower);
            }
        }

        return powers;
    }

    private double max(double[] array) {
        double max = 0;
        for(double number : array) {
            if(Math.abs(number) > max) {
                max = Math.abs(number);
            }
        }
        return max;
    }

    private double min(double[] array) {
        double min = Integer.MAX_VALUE;
        for(double number : array) {
            if(Math.abs(number) < min) {
                min = Math.abs(number);
            }
        }
        return min;
    }
    //endregion
}

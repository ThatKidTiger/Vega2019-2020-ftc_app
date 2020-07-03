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
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.DIST_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.DIST_PID;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.DIST_CONSTANTS.power;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;

/*
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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    /* Constructor */
    public Robot() {
        subsystems.add(drive);
        subsystems.add(imu);
        subsystems.add(latch);
        subsystems.add(liftIntake);
    }

    public void update() {
        Map<String, Object> updates = new HashMap<>();
        for(Subsystem subsystem : subsystems) {
            Map<String, Object> updatePacket = subsystem.update();
            updates.putAll(updatePacket);
        }
        packet.putAll(updates);
        dashboard.sendTelemetryPacket(packet);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        imu.init(hwMap);
        drive.init(hwMap);
        latch.init(hwMap);
        liftIntake.init(hwMap);
        runtime.startTime();
        colLeft = hwMap.get(ColorSensor.class, "colLeft");
        colRight = hwMap.get(ColorSensor.class, "colRight");
        distance = hwMap.get(DistanceSensor.class, "distance");
        topDistance = hwMap.get(DistanceSensor.class, "topDistance");
    }

    //region Auton methods
    public void forward(double power) {
        Log.d(TAG, String.format("Moving " + (power > 0 ? "forward" : "backward") + " at power %f.2", Math.abs(power)));
        power = -power;
        double[] powers = {power, power, power, power};
        drive.setMotorPowers(powers);
    }

    public void strafe(double power) {
        Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
        power = -power;
        double[] powers = {power, -power, power, -power};
        drive.setMotorPowers(powers);
    }

    public void rotateByPower(double power) {
        Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
        power = -power;
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

            leftPower = temppower;
            rightPower = -temppower;

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
        double leftPower, rightPower, temppower;

        //reset reference angle and PID controllers
        runtime.reset();
        MiniPID PD = new MiniPID(DIST_PID.p, DIST_PID.i, DIST_PID.d);

        while(topDistance.getDistance(DistanceUnit.CM) > 120){
            drive.setMotorPowers(new double[] {1, 1, 1, 1});
        }

        double lastDistance = 0;
        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - topDistance.getDistance(DistanceUnit.CM)) > 0.1 || Math.abs(dist - lastDistance) > 0.1 || runtime.seconds() > 4) {
            double current = topDistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            if(Math.abs(temppower) < DIST_MIN) {
                temppower *= (DIST_MIN/Math.abs(temppower));
            }

            if (Math.abs(temppower) > power) {
                temppower *= (power/Math.abs(temppower));
            }

            leftPower = -temppower;
            rightPower = -temppower;

            drive.setMotorPowers(new double[] {leftPower, leftPower, rightPower, rightPower});
            lastDistance = current;
        }
        stop();
    }

    //Todo: are left and right powers necessary, or can negative temppower just be used
    public void moveToDistance(double dist) {
        //Accurate once it moves within 120 centimeters of the object it is approaching
        double leftPower, rightPower, temppower;

        //reset reference angle and PID controllers
        runtime.reset();
        MiniPID PD = new MiniPID(DIST_PID.p, DIST_PID.i, DIST_PID.d);

        while(distance.getDistance(DistanceUnit.CM) > 120){
            drive.setMotorPowers(new double[] {1, 1, 1, 1});
        }

        double lastDistance = 0;
        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - distance.getDistance(DistanceUnit.CM)) > 0.1 || Math.abs(dist - lastDistance) > 0.1 || runtime.seconds() > 4) {
            double current = distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            if(Math.abs(temppower) < DIST_MIN) {
                temppower *= (DIST_MIN/Math.abs(temppower));
            }

            if (Math.abs(temppower) > power) {
                temppower *= (power/Math.abs(temppower));
            }

            leftPower = -temppower;
            rightPower = -temppower;

            drive.setMotorPowers(new double[] {leftPower, leftPower, rightPower, rightPower});
            lastDistance = current;
        }
        stop();
    }
    //endregion
}

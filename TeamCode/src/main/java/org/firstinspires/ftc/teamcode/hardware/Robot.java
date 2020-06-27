package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/*
Todo: check if initializing the hardware map is necessary, or if object references
Todo: can be individually fetched by each subsystem.
Todo: pass the dashboard object reference to robot, and then retrieve the various subsystem updates
 */

public class Robot {
    ArrayList<Subsystem> subsystems = new ArrayList();
    public IMU imu = new IMU();
    public MecanumDrive drive = new MecanumDrive();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    /* Public OpMode members. */
    public DcMotor latch; //hub 2 port 0
    public DcMotor lift; //hub 2 port 1
    public DcMotor gripper; //hub 2 port 2
    public ColorSensor colLeft; //hub 2 port 0
    public ColorSensor colRight; //hub 2 port 2
    public DistanceSensor distance; //hub 2 port 1
    public DistanceSensor topdistance; //hub 3 port 0

    /* Constructor */
    public Robot() {

    }

    //Todo: Multithreading for telemetry updates
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();
        Map<String, Object> updates = new HashMap<>();
        Map<String, Object> imuUpdates = imu.update();
        Map<String, Object> driveUpdates = drive.update();
        updates.putAll(imuUpdates);
        updates.putAll(driveUpdates);
        packet.putAll(updates);
        dashboard.sendTelemetryPacket(packet);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        imu.init(hwMap);
        drive.init(hwMap);

        latch = hwMap.get(DcMotor.class, "latch");
        lift = hwMap.get(DcMotor.class, "lift");

        /*lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);*/

        gripper = hwMap.get(DcMotor.class, "gripper");

        //define sensors
        colLeft = hwMap.get(ColorSensor.class, "colLeft");
        colRight = hwMap.get(ColorSensor.class, "colRight");
        distance = hwMap.get(DistanceSensor.class, "distance");
        topdistance = hwMap.get(DistanceSensor.class, "topdistance");
    }
}

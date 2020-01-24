/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="VegaScrimAuton", group="queenYash")
public class VegaScrimAuton extends LinearOpMode {

    /* Declare OpMode members. */
    VegaHardware  robot   = new VegaHardware();

    private ElapsedTime runtime = new ElapsedTime(0);

    public static final double NV_40_COUNTS = 1120;
    public static final double NV_40_DIAM = 10.16;

    //region intrinsic orientation variables
    private Orientation lastAngles = new Orientation();
    private double relativeAngle, globalAngle, initialAngle;
    //endregion

    private MiniPID rotPID = new MiniPID(0.05, 0.0056, 0.0055);

    private MiniPID PD = new MiniPID(0.032, 0 , 0.01);


    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.imu.readCalibrationData();

        while (!isStarted()) {

        }
        initialAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        moveTo(0.8, 6);
        grab();
        moveDistance(0.6, -10);
        rotate(-90, 0.6);
        moveTo(0.8, 20);
        rotate(90, 0.6);
        moveTime(1, 0.6, 400);
        robot.gripper.setTargetPosition(0);
        rotate(90, 0.6);
        moveTime(1, 0.8, 600);

    }

    private void resetAngle() {
        //retrieve current orientation
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //reset globalAngle and overall relative angle
        relativeAngle = 0;
        globalAngle = lastAngles.firstAngle;
    }

    private double getOrientation() {
        //retrieve current imu orientation information
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //calculate change from the the global reference angle
        double deltaAngle = angles.firstAngle - globalAngle;

        //because IMU returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //set lastAngle to the current angle
        lastAngles = angles;

        //return orientation compared to the global reference
        return deltaAngle;
    }

    public double getAngle() {
        //retrieve current imu orientation information
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //calculate change from the last recorded position
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //because IMU returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //change the overall deviation from the initial orientation
        relativeAngle += deltaAngle;

        //set the lastAngle to current angle
        lastAngles = angles;

        return relativeAngle;
    }

    private double getAbsolute() {
        //retrieve current imu orientation information
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //retrieve deviation relative to the initial final reference angle
        double deltaAngle = initialAngle - lastAngles.firstAngle;

        //because IMU returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //return orientation relative to the initial reference angle
        return deltaAngle;
    }

    private void rotate(double degrees, double power) {
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        PD.reset();

        //ramps down to zero, starting from 20 degrees
        PD.setP(Math.abs(power/(degrees/3)));
        PD.setD(0);
        sleep(500);

        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - getOrientation()) > 0.1 && opModeIsActive()) {
            double orientation = getOrientation();
            temppower = PD.getOutput(orientation, degrees);

            if(Math.abs(orientation) < Math.abs(degrees/3)) {
                temppower *= Math.abs(orientation/(degrees/3));
            }

            telemetry.addData("temppower: ", temppower);

            //ensures the powers are within the lower power limit
            if(Math.abs(temppower) < 0.1) {
                temppower = Math.signum(temppower) * (0.1);
            }

            leftPower = -temppower;
            rightPower = temppower;

            //caps the power at the inputed power
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

            if (max > power)
            {
                leftPower *= (power/max);
                rightPower *= (power/max);
            }

            robot.frontRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);

            telemetry.addData("Motor Powers: ", "%f %f", leftPower, rightPower);
            telemetry.addData("difference: ", Math.abs(degrees - getOrientation()));
            telemetry.update();
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    private void moveDistance(double power, double change) {
        double leftPower, rightPower, temppower, dist;
        //reset reference angle and PID controllers
        resetAngle();
        rotPID.reset();
        PD.reset();

        //ramps down to zero, starting from 30 cm
        PD.setP(Math.abs(power/(change/3)));
        PD.setD(0.01);

        //calculates the goal distance that should be returned
        dist = robot.distance.getDistance(DistanceUnit.CM) - change;
        //continues to drive until the distance sensor reports it's within a margin of error
        while(Math.abs(dist - robot.distance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(Math.abs(dist + change - current) < Math.abs(change/3)) {
                temppower *= Math.abs((dist + change - current)/(change/3));
            }

            leftPower = -temppower;
            rightPower = -temppower;

            //caps the motor powers at a minimum
            if(Math.abs(temppower) < 0.1) {
                leftPower *= (0.1/temppower);
                rightPower *= (0.1/temppower);
            }

            //caps the motor powers at a maximum
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power)
            {
                leftPower *= (power/max);
                rightPower *= (power/max);
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

    private void moveToTop(double power, double dist) {
        //Accurate once it moves within 30 centimeters of the object it is approaching
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        runtime.reset();
        rotPID.reset();
        PD.reset();
        //ramps down to zero starting from 30 cm
        PD.setP(Math.abs(0.01));
        PD.setD(0);

        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - robot.topdistance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.topdistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            leftPower = -temppower;
            rightPower = -temppower;

            double min = Math.min(Math.abs(leftPower), Math.abs(rightPower));

            if(min < 0.1) {
                leftPower *= (0.1/min);
                rightPower *= (0.1/min);
            }

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power)
            {
                leftPower *= (power/max);
                rightPower *= (power/max);
            }

            while(!isStopRequested() && robot.topdistance.getDistance(DistanceUnit.CM) > 125){
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

    private void moveTo(double power, double dist) {
        //Accurate once it moves within 30 centimeters of the object it is approaching
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        runtime.reset();
        rotPID.reset();
        PD.reset();
        //ramps down to zero starting from 30 cm
        PD.setP(Math.abs(0.01));
        PD.setD(0);

        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - robot.distance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            leftPower = -temppower;
            rightPower = -temppower;

            double min = Math.min(Math.abs(leftPower), Math.abs(rightPower));

            if(min < 0.1) {
                leftPower *= (0.1/min);
                rightPower *= (0.1/min);
            }

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power)
            {
                leftPower *= (power/max);
                rightPower *= (power/max);
            }

            while(!isStopRequested() && robot.distance.getDistance(DistanceUnit.CM) > 13){
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

    private void moveTime(int direction, double power, double time) {
        double leftPower, rightPower, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();

        while(runtime.seconds() < time && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);

            leftPower = (power - (adjustment * direction)) * direction;
            rightPower = (power + (adjustment * direction)) * direction;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power)
            {
                leftPower *= (power/max);
                rightPower *= (power/max);
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

    private void strafeTime(int direction, double power, double time) {
        double FR, FL, BR, BL, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();
        while(runtime.seconds() < time && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);
            telemetry.addData("adjustment", adjustment);

            FR = -(power - adjustment * direction) * direction;
            FL = (power - adjustment * direction) * direction;
            BR = (power + adjustment * direction) * direction;
            BL = -(power + adjustment * direction) * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power/max);
            FL *= (power/max);
            BR *= (power/max);
            BL *= (power/max);

            telemetry.addData("Motor Powers: ", "%f, %f, %f, %f", FL, BL, FR, FL);
            telemetry.update();

            robot.frontRight.setPower(FR);
            robot.frontLeft.setPower(FL);
            robot.backRight.setPower(BR);
            robot.backLeft.setPower(BL);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    private void strafeCol(int direction, double power) {
        double FR, FL, BR, BL, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();

        while(!checkCol().equals("Yellow") && opModeIsActive()) {

            FR = -power * direction;
            FL = power * direction;
            BR = power * direction;
            BL = -power * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power/max);
            FL *= (power/max);
            BR *= (power/max);
            BL *= (power/max);

            telemetry.update();

            robot.frontRight.setPower(FR);
            robot.frontLeft.setPower(FL);
            robot.backRight.setPower(BR);
            robot.backLeft.setPower(BL);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        sleep(400);

        while(!checkCol().equals("Black") && opModeIsActive()) {

            FR = power * direction;
            FL = -power * direction;
            BR = -power * direction;
            BL = power * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power/max);
            FL *= (power/max);
            BR *= (power/max);
            BL *= (power/max);

            telemetry.addData("Motor Powers: ", "%f, %f, %f, %f", FL, BL, FR, FL);
            telemetry.update();

            robot.frontRight.setPower(FR);
            robot.frontLeft.setPower(FL);
            robot.backRight.setPower(BR);
            robot.backLeft.setPower(BL);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        sleep(400);

    }

    private String checkCol() {
        if((robot.colLeft.red() < 70)) {
            return "Left";
        }
        else if((robot.colRight.red() < 70)) {
            return "Right";
        }
        else return "Yellow";
    }

    private void grab() {
        robot.gripper.setTargetPosition(280);
        robot.lift.setTargetPosition(200);
    }

    private double ticksToCM(double TICKS_PER_REV, double DIAM, double ticks) {
        return DIAM * Math.PI * (ticks / TICKS_PER_REV);
    }
}
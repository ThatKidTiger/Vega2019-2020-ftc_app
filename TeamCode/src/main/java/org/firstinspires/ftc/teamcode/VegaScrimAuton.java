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

@Autonomous(name = "VegaScrimAuton", group = "queenYash")
public class VegaScrimAuton extends LinearOpMode {

    /* Declare OpMode members. */
    VegaHardware robot = new VegaHardware();

    ElapsedTime runtime = new ElapsedTime(0);

    //region intrinsic orientation variables
    Orientation lastAngles = new Orientation();
    double relativeAngle, globalAngle, initialAngle;
    //endregion

    MiniPID rotPID = new MiniPID(0.05, 0.0056, 0.0055);

    MiniPID PD = new MiniPID(0.032, 0, 0.01);


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

        strafeTo(0.2, 10);
        moveTo(0.6, 6);
        moveDistance(0.6, -20);
        //strafeCol(-1, 0.6);
        grab();


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

    public double getAbsolute() {
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

    public void rotate(double degrees, double power) {
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        PD.reset();

        //ramps down to zero, starting from 20 degrees
        PD.setP(Math.abs(power / (degrees / 3)));
        PD.setD(0);
        sleep(500);

        //rotates until the imu returns that the robot is within a margin of error
        while (Math.abs(degrees - getOrientation()) > 0.1 && opModeIsActive()) {
            double orientation = getOrientation();
            temppower = PD.getOutput(orientation, degrees);

            if (Math.abs(orientation) < Math.abs(degrees / 3)) {
                temppower *= Math.abs(orientation / (degrees / 3));
            }

            telemetry.addData("temppower: ", temppower);

            //ensures the powers are within the lower power limit
            if (Math.abs(temppower) < 0.1) {
                temppower = Math.signum(temppower) * (0.1);
            }

            leftPower = -temppower;
            rightPower = temppower;

            //caps the power at the inputed power
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

            if (max > power) {
                leftPower *= (power / max);
                rightPower *= (power / max);
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
        PD.setP(Math.abs(power / (change / 3)));
        PD.setD(0.01);

        //calculates the goal distance that should be returned
        dist = robot.distance.getDistance(DistanceUnit.CM) - change;
        //continues to drive until the distance sensor reports it's within a margin of error
        while (Math.abs(dist - robot.distance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if (Math.abs(dist + change - current) < Math.abs(change / 3)) {
                temppower *= Math.abs((dist + change - current) / (change / 3));
            }

            leftPower = -temppower;
            rightPower = -temppower;

            //caps the motor powers at a minimum
            if (Math.abs(temppower) < 0.1) {
                leftPower *= (0.1 / temppower);
                rightPower *= (0.1 / temppower);
            }

            //caps the motor powers at a maximum
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power) {
                leftPower *= (power / max);
                rightPower *= (power / max);
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
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        rotPID.reset();
        PD.reset();
        //ramps down to zero starting from 30 cm
        PD.setP(Math.abs(power) / 30);
        PD.setD(0.01);

        double initial = robot.distance.getDistance(DistanceUnit.CM);

        //continues to move until the distance sensor returns it is within a margin of error
        while (Math.abs(dist - robot.distance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if (Math.abs(initial - current) < Math.abs((initial - dist) / 3)) {
                temppower *= Math.abs((initial - current) / ((initial - dist) / 3));
            }

            leftPower = -temppower;
            rightPower = -temppower;

            double min = Math.min(Math.abs(leftPower), Math.abs(rightPower));

            if (min < 0.1) {
                leftPower *= (0.1 / min);
                rightPower *= (0.1 / min);
            }

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power) {
                leftPower *= (power / max);
                rightPower *= (power / max);
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

    public void moveTime(int direction, double power, double time) {
        double leftPower, rightPower, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();

        while (runtime.seconds() < time && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);

            leftPower = (power - (adjustment * direction)) * direction;
            rightPower = (power + (adjustment * direction)) * direction;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > power) {
                leftPower *= (power / max);
                rightPower *= (power / max);
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

    private void strafeDistance(double power, double change) {
        double FR, FL, BR, BL, adjustment, temppower;
        int direction;
        resetAngle();
        rotPID.reset();
        runtime.reset();
        PD.setP(Math.abs(power / 20));
        PD.setD(0.025);

        double dist = robot.sideDistance.getDistance(DistanceUnit.CM) - change;

        while (Math.abs(dist - robot.sideDistance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.sideDistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);
            adjustment = rotPID.getOutput(getOrientation(), 0);

            if (Math.abs(dist + change - current) < Math.abs(change / 3)) {
                temppower *= Math.abs((dist + change - current) / (change / 3));
            }

            telemetry.addData("adjustment", adjustment);
            telemetry.addData("orientation: ", getOrientation());

            double difference = dist - robot.sideDistance.getDistance(DistanceUnit.CM);


            //prevents infinite shake at the end of settling
            if (Math.abs(difference) < 8) {
                adjustment = 0;
            }

            if (difference > 0) {
                direction = 1;
            } else {
                direction = -1;
            }

            FR = (temppower - adjustment * direction);
            FL = -(temppower - adjustment * direction);
            BR = -(temppower + adjustment * direction);
            BL = (temppower + adjustment * direction);

            double minf = Math.min(Math.abs(FR), Math.abs(FL));
            double minb = Math.min(Math.abs(BR), Math.abs(BL));
            double min = Math.min(minf, minb);

            if (min < 0.1) {
                FR *= (0.08 / min);
                FL *= (0.08 / min);
                BR *= (0.08 / min);
                BL *= (0.08 / min);
            }


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

    private void strafeTo(double power, double dist) {
        double FR, FL, BR, BL, adjustment, temppower;
        int direction;
        resetAngle();
        rotPID.reset();
        runtime.reset();
        PD.setP(Math.abs(power / 20));
        PD.setD(0.025);

        double initial = robot.sideDistance.getDistance(DistanceUnit.CM);
        while (Math.abs(dist - robot.sideDistance.getDistance(DistanceUnit.CM)) > 0.25 && opModeIsActive()) {
            double current = robot.sideDistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);
            adjustment = rotPID.getOutput(getOrientation(), 0);
            telemetry.addData("adjustment", adjustment);
            telemetry.addData("orientation: ", getOrientation());
            telemetry.addData("Position: ", initial - current);
            telemetry.addData("To Travel: ", initial - dist);
            telemetry.addData("Power: ", temppower * ((initial - current) / (initial - dist)));

            if (Math.abs(initial - current) < Math.abs((initial - dist) / 3)) {
                temppower *= Math.abs((initial - current) / ((initial - dist) / 3));
            }

            double difference = dist - current;

            //prevents infinite shake at the end of settling
            if (Math.abs(difference) < 8) {
                adjustment = 0;
            }

            if (difference > 0) {
                direction = 1;
            } else {
                direction = -1;
            }

            FR = (temppower - adjustment * direction);
            FL = -(temppower - adjustment * direction);
            BR = -(temppower + adjustment * direction);
            BL = (temppower + adjustment * direction);

            double minf = Math.min(Math.abs(FR), Math.abs(FL));
            double minb = Math.min(Math.abs(BR), Math.abs(BL));
            double min = Math.min(minf, minb);

            //continue decreasing minimum power if needed
            if (min < 0.08) {
                FR *= (0.08 / min);
                FL *= (0.08 / min);
                BR *= (0.08 / min);
                BL *= (0.08 / min);
            }

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

    public void strafeTime(int direction, double power, double time) {
        double FR, FL, BR, BL, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();
        while (runtime.seconds() < time && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);
            telemetry.addData("adjustment", adjustment);

            FR = -(power - adjustment * direction) * direction;
            FL = (power - adjustment * direction) * direction;
            BR = (power + adjustment * direction) * direction;
            BL = -(power + adjustment * direction) * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power / max);
            FL *= (power / max);
            BR *= (power / max);
            BL *= (power / max);

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

    public void strafeCol(int direction, double power) {
        double FR, FL, BR, BL, adjustment;
        runtime.reset();
        resetAngle();
        rotPID.reset();

        while (!checkCol().equals("Yellow") && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);
            telemetry.addData("adjustment", adjustment);

            FR = -(power - adjustment * direction) * direction;
            FL = (power - adjustment * direction) * direction;
            BR = (power + adjustment * direction) * direction;
            BL = -(power + adjustment * direction) * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power / max);
            FL *= (power / max);
            BR *= (power / max);
            BL *= (power / max);

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

        while (!checkCol().equals("Black") && opModeIsActive()) {
            adjustment = rotPID.getOutput(getOrientation(), 0);
            telemetry.addData("adjustment", adjustment);

            FR = (power + adjustment * direction) * direction;
            FL = -(power + adjustment * direction) * direction;
            BR = -(power - adjustment * direction) * direction;
            BL = (power - adjustment * direction) * direction;

            double maxf = Math.max(Math.abs(FR), Math.abs(FL));
            double maxb = Math.max(Math.abs(BR), Math.abs(BL));
            double max = Math.max(maxf, maxb);

            FR *= (power / max);
            FL *= (power / max);
            BR *= (power / max);
            BL *= (power / max);

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

        strafeDistance(0.6, 5);
    }

    public String checkCol() {
        String colString = "";
        if ((robot.colSen.green() < 450)) {
            return "Black";
        } else {
            return "Yellow";
        }
    }

    public void grab() {
        robot.horizontalArm.setPower(-0.7);
        sleep(2000);
        robot.horizontalArm.setPower(0);
        robot.liftMech.setPower(-0.5);
        sleep(2300);
        robot.liftMech.setPower(0);
        robot.gripper.setPosition(0);
        sleep(500);
        robot.liftMech.setPower(0.5);
        sleep(2200);
        robot.liftMech.setPower(0);
    }
}
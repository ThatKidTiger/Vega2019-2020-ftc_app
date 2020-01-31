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

@Autonomous(name="Red Side Skystones", group="queenYash")
public class SkystoneRedAuton extends LinearOpMode {

    /* Declare OpMode members. */
    VegaHardware  robot   = new VegaHardware();

    ElapsedTime runtime = new ElapsedTime(0);

    //region intrinsic orientation variables
    private Orientation lastAngles = new Orientation();
    private double relativeAngle, globalAngle, initialAngle;
    //endregion

    int pos = 0;
    @Override
    public void runOpMode() {
        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        while (!isStarted()) {}

        runtime.reset();
        runtime.startTime();
        moveTo(.3, 13);
        sleep(250);

        strafeCol(-1, 0.3);
        grab();
        moveTime(-1, 0.5, 200);
        rotate(-90, 0.8);
        moveToTop(0.8, 120);
        release();
        rotate(180, 0.8);
        moveToTop(0.8, 15);
        rotate(-90, 0.8);
        strafeCol(1, 0.3);
        grab();
        moveTime(-1, 0.5, 200);
        rotate(-90, 0.8);
        moveToTop(0.8, 120);
        release();
        moveTime(-1, 0.5, 1000);
    }

    private void grab() {
        runtime.reset();
        while (!isStopRequested() && runtime.milliseconds() < 500){
            robot.latch.setPower(-0.2);
        }
        robot.latch.setPower(0);
    }

    private void release() {
        runtime.reset();
        while (!isStopRequested() && runtime.milliseconds() < 500){
            robot.latch.setPower(0.2);
        }
        robot.latch.setPower(0);
    }

    public void unlatch() {
        runtime.reset();
        while (!isStopRequested() && runtime.milliseconds() < 400){
            robot.latch.setPower(-.8);
        }
        robot.latch.setPower(0);

    }

    public void latch(){
        runtime.reset();
        while (!isStopRequested() && runtime.milliseconds() < 400){
            robot.latch.setPower(.8);
        }
        robot.latch.setPower(0);
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
        deltaAngle = limitAxes(deltaAngle);

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
        deltaAngle = limitAxes(deltaAngle);

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
        double deltaAngle = initialAngle - angles.firstAngle;

        //because IMU returns rotation on a set of axes -180 to 180 adjust angle change to be the most logical direction
        deltaAngle = limitAxes(deltaAngle);

        //return orientation relative to the initial reference angle
        return deltaAngle;
    }

    private void rotate(double degrees, double power) {
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        runtime.reset();
        resetAngle();

        MiniPID PD = new MiniPID(0.02, 0, 0.005);
        sleep(250);

        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - getOrientation()) > 0.1 && opModeIsActive() && Math.abs(degrees - getOrientation()) < 200 && runtime.seconds() < 4) {
            double orientation = getOrientation();
            temppower = PD.getOutput(orientation, degrees);

            if(Math.abs(orientation) < Math.abs(degrees/3)) {
                temppower *= Math.abs(orientation/(degrees/3));
            }

            //telemetry.addData("temppower: ", temppower);

            //ensures the powers are within the lower power limit
            if(Math.abs(temppower) < 0.15) {
                temppower = Math.signum(temppower) * 0.15;
            }

            if(Math.abs(temppower) > power) {
                temppower *= (power/temppower);
            }

            if(orientation == 0) {
                temppower = Math.signum(degrees) * 0.15;
            }

            leftPower = -temppower;
            rightPower = temppower;

            robot.frontRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);

            //telemetry.addData("Motor Powers: ", "%f %f", leftPower, rightPower);
            //telemetry.addData("difference: ", Math.abs(degrees - getOrientation()));
            //telemetry.update();
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    private void moveTopDistance(double power, double change) {
        double leftPower, rightPower, temppower, dist;

        //reset reference angle and PID controllers
        resetAngle();
        MiniPID PD = new MiniPID(0.02, 0, 0.005);

        //calculates the goal distance that should be returned
        dist = robot.topdistance.getDistance(DistanceUnit.CM) - change;

        //continues to drive until the distance sensor reports it's within a margin of error
        while(Math.abs(dist - robot.topdistance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive()) {
            double current = robot.topdistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            //caps the motor powers at a minimum
            if(Math.abs(temppower) < 0.15) {
                temppower *= (0.15/temppower);
            }

            //caps the motor powers at a maximum
            if (Math.abs(temppower) > power) {
                temppower *= (power/temppower);
            }

            leftPower = -temppower;
            rightPower = -temppower;

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
        MiniPID PD = new MiniPID(0.02, 0, 0.004);

        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - robot.topdistance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive() && runtime.seconds() < 4) {
            double current = robot.topdistance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            if(Math.abs(temppower) < 0.15) {
                temppower *= (0.15/temppower);
            }

            if (Math.abs(temppower) > power) {
                temppower *= (power/temppower);
            }

            leftPower = -temppower;
            rightPower = -temppower;

            while(!isStopRequested() && robot.topdistance.getDistance(DistanceUnit.CM) > 120){
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
        //Accurate once it moves within 13 centimeters of the object it is approaching
        double leftPower, rightPower, temppower;
        //reset reference angle and PID controllers
        resetAngle();
        runtime.reset();

        MiniPID PD = new MiniPID(0.02, 0, 0.004);

        //continues to move until the distance sensor returns it is within a margin of error
        while(Math.abs(dist - robot.distance.getDistance(DistanceUnit.CM)) > 0.1 && opModeIsActive() && runtime.seconds() < 4) {
            double current = robot.distance.getDistance(DistanceUnit.CM);
            temppower = PD.getOutput(current, dist);

            if(runtime.seconds() < 1) {
                temppower *= power * runtime.seconds();
            }

            if(Math.abs(temppower) < 0.15) {
                temppower *= (0.15/temppower);
            }

            if (Math.abs(temppower) > power) {
                temppower *= (power/temppower);
            }

            leftPower = -temppower;
            rightPower = -temppower;

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

        while(runtime.milliseconds() < time && opModeIsActive()) {

            leftPower = power * direction;
            rightPower = power * direction;

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

        while(runtime.milliseconds() < time && opModeIsActive()) {
            FR = -power * direction;
            FL = power * direction;
            BR = power * direction;
            BL = -power * direction;

            //telemetry.addData("Motor Powers: ", "%f, %f, %f, %f", FL, BL, FR, FL);
            //telemetry.update();

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

        while(!checkCol() && opModeIsActive() && !isStopRequested()) {

            FR = -power * direction;
            FL = power * direction;
            BR = power * direction;
            BL = -power * direction;

            //telemetry.addData("Motor Powers: ", "%f, %f, %f, %f", FL, BL, FR, FL);
            //telemetry.update();

            robot.frontRight.setPower(FR);
            robot.frontLeft.setPower(FL);
            robot.backRight.setPower(BR);
            robot.backLeft.setPower(BL);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        sleep(250);
    }

    private boolean checkCol() {
        boolean i = false;
        if((robot.colLeft.red() < 70) && robot.colRight.red() < 70) {
            i = true;
        }
        return i;
    }

    private double limitAxes(double orientation) {
        if (orientation < -180)
            orientation += 360;
        else if (orientation > 180)
            orientation -= 360;
        return orientation;
    }
}
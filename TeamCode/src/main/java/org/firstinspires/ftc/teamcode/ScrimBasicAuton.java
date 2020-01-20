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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@Autonomous(name = "ScrimBasicAuton", group = "queenYash")
public class ScrimBasicAuton extends LinearOpMode {

    /* Declare OpMode members. */
    VegaHardware robot = new VegaHardware();

    ElapsedTime runtime = new ElapsedTime(0);

    //region intrinsic orientation variables
    Orientation lastAngles = new Orientation();
    double relativeAngle, adjustment;
    //endregion

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        while (!isStarted()) {

        }
        forward(.4, 1000);
        strafeCol(-1, 500);
        grab();

        /*
        moveTime(1, 1, 2.5);
        strafeCol(1, 0.8);
        grab();
        strafeTime(1, 1, 5);
        robot.gripper.setPosition(180);
        strafeTime(-1, 1, 3);
        */
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        relativeAngle = 0;
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

        relativeAngle += deltaAngle;

        lastAngles = angles;

        return relativeAngle;
    }

    //proportional power decrease to eliminate overshoot on rotations and movements
    public double PControl(double power, double goal, double current, double threshold, double minimum) {
        double ratio, temppower;
        double difference = goal - current;
        if (Math.abs(difference) < threshold * Math.signum(goal)) {
            ratio = difference / threshold;
            temppower = power * ratio;
        } else temppower = power;

        temppower = Range.clip(temppower, minimum, 1);
        return temppower;
    }

    public void rotate(double degrees, double power) {
        double leftPower, rightPower, temppower, ratio;

        if (degrees < 0) {
            degrees += 1.5;
        } else if (degrees > 0) {
            degrees -= 1.5;
        } else return;

        degrees *= -1;

        temppower = power;
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
                leftPower = power;
                rightPower = -power;

                robot.backLeft.setPower(leftPower);
                robot.backRight.setPower(rightPower);
                robot.frontLeft.setPower(leftPower);
                robot.frontRight.setPower(rightPower);
            }

            while (getAngle() > degrees) {
                temppower = PControl(power, degrees, getAngle(), 25, 0.1);

                leftPower = temppower;
                rightPower = -temppower;

                robot.backLeft.setPower(leftPower);
                robot.backRight.setPower(rightPower);
                robot.frontLeft.setPower(leftPower);
                robot.frontRight.setPower(rightPower);
            }
        } else {
            // left turn.
            while (getAngle() < degrees) {
                temppower = PControl(power, degrees, getAngle(), 25, 0.1);

                leftPower = -temppower;
                rightPower = temppower;

                robot.backLeft.setPower(leftPower);
                robot.backRight.setPower(rightPower);
                robot.frontLeft.setPower(leftPower);
                robot.frontRight.setPower(rightPower);
            }
        }
        // turn the motors off.
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        // wait for rotation to stop.
        sleep(200);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void moveTime(int direction, double power, double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            power = PControl(power, time, runtime.seconds(), 1, 0.1);
            robot.frontRight.setPower(power * direction);
            robot.frontLeft.setPower(power * direction);
            robot.backRight.setPower(power * direction);
            robot.backLeft.setPower(power * direction);
        }
    }

    public void strafeTime(int direction, double power, double time) {
        double leftPower, rightPower;
        double correctFactor = 0.05;
        runtime.reset();
        double startAngle = getAngle();
        while (runtime.seconds() < time) {
            power = PControl(power, time, runtime.seconds(), 1, 0.1);

            double angleDiff = startAngle - getAngle();

            if (angleDiff < -2) {
                leftPower = power - (Math.abs(correctFactor * angleDiff));
                rightPower = power;
            } else if (angleDiff > 2) {
                leftPower = power;
                rightPower = power - (Math.abs(correctFactor * angleDiff));
            } else {
                leftPower = power;
                rightPower = power;
            }

            robot.frontRight.setPower(-rightPower * direction);
            robot.frontLeft.setPower(leftPower * direction);
            robot.backRight.setPower(rightPower * direction);
            robot.backLeft.setPower(-leftPower * direction);
        }
    }

    public void strafeCol(int direction, double power) {
        runtime.reset();
        while (robot.colSen.green() > 300) {
            robot.frontRight.setPower(-power * direction);
            robot.frontLeft.setPower(power * direction);
            robot.backRight.setPower(power * direction);
            robot.backLeft.setPower(-power * direction);
        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }


    public void forward(double power, double time) {
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        while (runtime.seconds() < time) {

        }
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    public void grab() {
        robot.horizontalArm.setPower(-1);
        sleep(700);
        robot.liftMech.setPower(-1);
        robot.gripper.setPosition(0);
        sleep(2500);
        robot.liftMech.setPower(1);
        sleep(2000);
        robot.liftMech.setPower(0);
    }
}
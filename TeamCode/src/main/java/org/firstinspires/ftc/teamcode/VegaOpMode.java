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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;

@TeleOp(name = "VegaOpMode", group = "VegaBot")
public class VegaOpMode extends OpMode {
    VegaHardware robot = new VegaHardware();
    Orientation lastAngles = new Orientation();
    double relativeAngle, globalAngle;
    //controls driver 2 strafe toggle
    boolean twoStrafe = false;
    boolean xChange = false;
    boolean open = true;
    boolean aChange = false;
    boolean foundation = false;
    boolean bChange = false;
    MiniPID P = new MiniPID(0.032, 0, 0);
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        runtime.startTime();
        robot.init(hardwareMap);
        robot.imu.readCalibrationData();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double strafe = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y;

        double tangent = rightY / strafe;

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
            //player one slow drive foundation mode
            leftPower *= 0.35;
            rightPower *= 0.35;
            robot.backLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.frontRight.setPower(rightPower);
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

        robot.horizontalArm.setPower(0.8 * (gamepad2.left_trigger - gamepad2.right_trigger));

        robot.latch.setPower(0.6 * (gamepad1.right_trigger - gamepad1.left_trigger));

        if (gamepad2.a && !aChange) {
            aChange = true;
            open = !open;
        } else if (!gamepad2.a) {
            aChange = false;
        }

        if (open) {
            robot.gripper.setPosition(180);
        } else {
            robot.gripper.setPosition(0);
        }

        if (gamepad1.b && !bChange) {
            bChange = true;
            foundation = !foundation;
        } else if (!gamepad2.b) {
            bChange = false;
        }

        if (gamepad2.dpad_up) {
            robot.liftMech.setPower(.5);
        } else if (gamepad2.dpad_down) {
            robot.liftMech.setPower(-.5);
        } else robot.liftMech.setPower(0);

        telemetry.addData("IMU Calib", robot.imu.getCalibrationStatus().toString());
        telemetry.addData("Distance(cm): ", "%f", robot.distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Side Distance: ", robot.sideDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Angle: ", getOrientation());
        telemetry.addData("RawDistance:", robot.blockDist.getDistance(DistanceUnit.CM));
        telemetry.addData("R,G,B: ", "%d %d %d  ", robot.colSen.red(), robot.colSen.green(), robot.colSen.blue());
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

    public void rotate(double degrees, double power) {
        double leftPower, rightPower, temppower;
        resetAngle();
        P.reset();
        P.setP(Math.abs(power) / 20);
        sleep(500);

        while (Math.abs(degrees - getOrientation()) > 1) {
            temppower = P.getOutput(getOrientation(), degrees);

            telemetry.addData("temppower: ", temppower);

            if (Math.abs(temppower) < 0.15) {
                temppower = Math.signum(temppower) * (0.15);
            }

            leftPower = -temppower;
            rightPower = temppower;

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
}
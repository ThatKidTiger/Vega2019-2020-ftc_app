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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "VegaFunctionAuton", group = "queenYash")
public class VegaFunctionAuton extends LinearOpMode {

    //regioncamera settings and vuforia key
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AT1SAAP/////AAABmS5DAlYFM06JmM1uCmlZz5EIowbjSKgCuoX9ig4xQs+h7mVVB5XG62XUY6wFAtMeQ66jNYsyC0gdpGmGmp3QqsEEKQnpJ49z6WHEXDY/rk1dh3FXUWXP23gLhV3FNHdG+YkidBsJq2+l1HD9COUnZ1kb73L9xH1ipOrMPjbZkXTUocWCH73boyPC7lJuw/OUv+5+jff18jw8ff0RlGuj2kjz0C9sq6FcgyODLW8EeAQqZLZP2oc3EK5FyWF3y3eNidRzvHVWohwmvvNRiw/qCBv3NLlX7DJnw3N0twB/6CH6GnB/khcGJObSrZ1YPr10ekYNZqe3k5xud7Y3GiX4PJeC3C1WFsTeRFo+kaq3bSdu\n";
    //region target measurement constants
    //Get conversions, because FIRST US uses imperial units
    private static final float mmPerInch = 25.4f;
    //endregion
    // the height of the center of the target image above the floor
    private static final float mmTargetHeight = (6) * mmPerInch;
    //Size of Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    //Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    //endregion
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
    /* Declare OpMode members. */
    VegaHardware robot = new VegaHardware();
    //endregion
    ElapsedTime runtime = new ElapsedTime(0);
    //region intrinsic orientation variables
    Orientation lastAngles = new Orientation();
    double relativeAngle, adjustment;
    //region robot measurement constants
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    //endregion
    String targetname;
    volatile float relX;
    //endregion
    volatile float relY;
    volatile float relZ;
    volatile float relRotX;
    volatile float relRotY;
    volatile float relRotZ;
    volatile float robotX;
    volatile float robotY;
    volatile float robotZ;
    volatile float robotrotX;
    volatile float robotrotY;
    volatile float robotrotZ;
    //region phone relative to robot constants
    private float phoneXRotate = 0;
    private float phoneYRotate = -90;
    private float phoneZRotate = 0;
    private VuforiaLocalizer vuforia = null;
    private ExecutorService vuforiaUpdateExecutor;
    //region vuforia output fields
    private boolean targetVisible = false;
    private OpenGLMatrix lastLocation = null;
    //endregion

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //enter vuforia parameters
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;

        //Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //load the actual set of images
        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("Skystone");

        //region give all of the trackables a name within our code
        VuforiaTrackable skyStone = targets.get(0);
        skyStone.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targets.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targets.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targets.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targets.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targets.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targets.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targets.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targets.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targets.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targets.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targets.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targets.get(12);
        rear2.setName("Rear Perimeter 2");
        //endregion


        //region set known image positions
        //Set the position of the bridge support targets with relation to origin (center of field)
        skyStone.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //endregion

        //collecting all of the VuforiaTrackable objects together
        allTrackables.addAll(targets);

        /*
         Create a transformation matrix describing where the phone is on the robot.

         NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
         Lock it into Portrait for these numbers to work.

         Info:  The coordinate frame for the robot looks the same as the field.
         The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         Z is UP on the robot.  This equates to a bearing angle of Zero degrees.

         The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         pointing to the LEFT side of the Robot.
         The two examples below assume that the camera is facing forward out the front of the robot.
        */
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targets.activate();

        while (opModeIsActive()) {

        }

        // Disable Tracking when we are done;
        targets.deactivate();
        if (isStopRequested()) {
            vuforiaUpdateExecutor.shutdown();
        }
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

    private OpenGLMatrix vuforiaOutput() {
        OpenGLMatrix trackablelocation = null;

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                trackablelocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();

        return trackablelocation;
    }

    public void strafe(double current, double goal, double power) {
        double timesatisfied = 0;
        double starttime = 0;
        while (timesatisfied < 2) {
            starttime = runtime.seconds();
            while (!(current > goal - 2 || current < goal + 2)) {
                telemetry.addLine("strafing");
                PControl(power, goal, current, 15, 0.1);
                robot.frontRight.setPower(-power);
                robot.frontLeft.setPower(power);
                robot.backRight.setPower(power);
                robot.backLeft.setPower(-power);
            }

            if (current > goal - 5 && current < goal + 5) {
                timesatisfied = runtime.seconds() - starttime;
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Team 8018
 * Theoretically Impossible
 * Driver Control Program
 */

@TeleOp(name = "TIOldTeleOp", group = "Pushbot")

public class TIOldTeleOp extends LinearOpMode {

    //Next 2 lines determine speed of servo movement
    final double GLYPH_SPEED = .07;
    TrashBotHardware robot = new TrashBotHardware();  //Creates an object "robot" that uses our team's hardware
    //Next 4 lines: starting positions for servos
    double glyphOffsetTopRight = 0.5;
    double glyphOffsetBottomRight = 0.5;
    double glyphOffsetLeft = 0.5;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double drift;
        double lift;
        double turn;
        double max;

        //Initialize the hardware variables (init() method does everything)
        robot.init(hardwareMap);

        //Shows a message on driver station to show program is ready
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        //This pauses the program until the start button is pressed
        waitForStart();
        //The following code will run until the stop button is pressed
        while (opModeIsActive()) {

            //The next 4 lines assign the motors to an action on the controller
            drive = gamepad1.left_stick_y;
            drift = -gamepad1.right_stick_x;
            turn = -gamepad1.left_stick_x;
            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
                lift = -0.1;
            } else {
                lift = (gamepad1.left_trigger - gamepad1.right_trigger) * .65; //Limits speed to avoid damage
            }

            //Next 2 lines: power motors so the robot turns either left or right
            left = drive - turn;
            right = drive + turn;


            /*Because a motor's power cannot exceed 1.0, the following code
             *reduces the left and right drive motor powers (if they exceed 1.0)
             *by dividing each by the higher value between the two- this makes
             *it so the motor with a higher power is scaled down to 1.0, while
             *keeping power proportional to before in the other motor.
             */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            /*Next 3 lines:
             *scales the value of the motor's power so that the speed varies
             *according to how far you hold on the joysticks
             */
            left = (float) scaleSpeed(left) * .7;
            right = (float) scaleSpeed(right) * .7;
            drift = (float) scaleSpeed(drift) * .8;

            lift = (float) lift;

            //The next 4 lines set each motors power to the previously calculated values
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            robot.midMotor.setPower(-drift);
            robot.clawLift.setPower(lift);


            // Use X to close, Y to open
            if (gamepad1.y) {
                robot.glyphBottomRight.setPosition(.25);
                robot.glyphTopRight.setPosition(.25);
                robot.glyphLeft.setPosition(.75);
            } else if (gamepad1.x) {
                robot.glyphBottomRight.setPosition(.75);
                robot.glyphTopRight.setPosition(.75);
                robot.glyphLeft.setPosition(.25);
            }

            //These show values on the driver station, such as the motor powers/servo positions
            telemetry.addData("glyphLeft", "Offset = %.2f", glyphOffsetLeft);
            telemetry.addData("glyphTopRight", "Offset = %.2f", glyphOffsetTopRight);
            telemetry.addData("glyphBottomRight", "Offset = %.2f", glyphOffsetBottomRight);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("lift", "%.2f", lift);
            telemetry.addLine("Open claw: Y");
            telemetry.addLine("Close claw: X");
            telemetry.update();


            //Slows down loop for reasonable speed
            sleep(50);
        }
    }

    /*This is an important method: this is what gets the scaling for how far
     *the driver is holding on the joystick
     */
    double scaleSpeed(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.40, 0.44, 0.50, 0.56, 0.70, 0.85, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}

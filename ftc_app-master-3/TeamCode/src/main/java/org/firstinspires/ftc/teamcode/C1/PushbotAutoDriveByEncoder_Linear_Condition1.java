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

package org.firstinspires.ftc.teamcode.C1;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.C1.hardwarePushbot1;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

public class PushbotAutoDriveByEncoder_Linear_Condition1 extends LinearOpMode {
    /* Declare OpMode members. */
    hardwarePushbot1 robot   = new hardwarePushbot1();   // Use a Pushbot's hardw
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.7401575 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;

    double displaceLeftFront;
    double displaceRightFront;
    double displaceLeftBack;
    double displaceRightBack;

    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() {
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdgZIY3/////AAAAmXUQWRXnWkG1kCjgBdoO3c8G2m/FaESxeNlMJDFcUSTPiuN1SR/w+ghJhbvgHTCRqSesas5+NlX/1uXmi2jD7T7AVrCL03AL6DqJqwWA5X8FVqrXtIbA7pLq892FSI1OHV9mvxlSy8bsvdDyIM7q1VF/pehaSitbenfkKJoui+4yuCyk6+HIfWLxNe8O/8GPrejOXl3OkjrWTVGerNntB8OGDhhIJJocAj3Wvp3ntPdUyMAZVBNqjgpPqlnO2cMvVBRYc8shPl8IByMXAK758cshNdZypVvqdQZdQTs388zBKDMtQlxw7s8opSgAcGZEH0eQgq/Hwr8WIOIm7xsysqIbW7oRLvFsqFGxomDf6nMc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");*/
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();




        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        /*robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //relicTrackables.activate();
        runtime.reset();
        double position = 0.398;

        robot.colorServo.setPosition(position);
        while (opModeIsActive()) {
            robot.colorServo.setPosition(position);
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)

            sleep(3000);
            double blue = robot.sensorColor.blue();
            double red = robot.sensorColor.red();

            runtime.reset();

            for(int i = 0; i <= 2; i++) {
                robot.colorServo.setPosition(position);

                if (blue >= 200 && blue <= 550 && red <= 130) {
                    driveByTime(-1, 1, -1, 1, 0.5);
                    sleep(500);
                    robot.colorServo.setPosition(0);
                    driveByTime(1, -1, 1, -1, 0.5);
                    break;
                } else if (blue <= 130 && red >= 200 && red <= 550) {
                    driveByTime(1, -1, 1, -1, 0.5);
                    sleep(500);
                    driveByTime(-1, 1, -1, 1, 0.5);
                    break;
                }
                position += 0.001;
            }


            // send the info back to driver station using telemetry function


            robot.colorServo.setPosition(1);

            runtime.reset();

            /*RelicRecoveryVuMark vuMark = null;
            while (runtime.seconds() <= obot.colorServo.setPosition(0);
8 && opModeIsActive()) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    break;
                }
            }

            if (vuMark == RelicRecoveryVuMark.CENTER) {
                displaceLeftFront = 2;

            }*/
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            /*encoderDrive(DRIVE_SPEED, displaceLeftFront, displaceRightFront, displaceLeftBack, displaceRightBack, 10);
            encoderDrive(DRIVE_SPEED, 39.3700787, -39.3700787, -39.3700787, 39.3700787, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(DRIVE_SPEED, 24, -24, -24, 24, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -24, 24, 24, -24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout*/

            driveByTime(1, -1, -1, 1, 1.5);

            /*robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
            robot.rightClaw.setPosition(0.0);*/
            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches, double timeoutS) {
        int newLeftBack;
        int newLeftFront;
        int newRightFront;
        int newRightBack;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftBack = robot.leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newLeftFront = robot.leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFront = robot.rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBack = robot.rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            robot.leftBack.setTargetPosition(newLeftBack);
            robot.leftFront.setTargetPosition(newLeftFront);
            robot.rightBack.setTargetPosition(newRightBack);
            robot.rightFront.setTargetPosition(newRightFront);


            // Turn On RUN_TO_POSITION

            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftBack.setPower(speed);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftFront.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() &&robot.rightBack.isBusy())) {

                // Display it for the driver.

            }
            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void driveByTime(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower, double time) {
        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightBack.setPower(rightBackPower);

        ElapsedTime rtm = new ElapsedTime();
        rtm.reset();

        while(rtm.seconds() <= time) {

        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

    }
}

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
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class hardwarePushbot1 extends LinearOpMode
{
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront  = null;
    public DcMotor leftBack     = null;
    public DcMotor rightBack = null;
    public DcMotor elevator = null;
    public Servo colorServo = null;
    public ColorSensor sensorColor = null;
    public DcMotor claw = null;
    public DigitalChannel digitalTouch = null;
    //public Servo finger = null;

    public static final double MID_SERVO       =  0.5 ;
    /*public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();
    /* Constructor */
    public hardwarePushbot1(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "l1");
        rightFront = hwMap.get(DcMotor.class, "r1");
        leftBack = hwMap.get(DcMotor.class, "l2");
        rightBack = hwMap.get(DcMotor.class, "r2");
        elevator = hwMap.get(DcMotor.class, "ele");
        claw = hwMap.get(DcMotor.class, "claw");
        colorServo = hwMap.get(Servo.class, "servo");
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        digitalTouch = hwMap.get(DigitalChannel.class, "touch_sensor");
        //elevator = hwMap.get(DcMotor.class, "ele");

        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);//elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.FORWARD);


        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        claw.setPower(0);
        elevator.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        colorServo.setPosition(0.95);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftClaw  = hwMap.get(Servo.class, "lClaw");
        //rightClaw  = hwMap.get(Servo.class, "rClaw");
        //finger = hwMap.get(Servo.class, "finger");
        /*elevator.setPower(0.6);

        runtime.reset();

        while(runtime.seconds() <= 1) {

        }

        elevator.setPower(0);

        leftClaw.setPosition(0.28);
        rightClaw.setPosition(0.66);



        runtime.reset();
        while(runtime.seconds() <= 1.8) {

        }
        rightClaw.setPosition(1.31);
        leftClaw.setPosition(-0.79);

        rightClaw.setPosition(1);
        leftClaw.setPosition(0);

        //finger.setPosition(0.5);*/
    }

    @Override

    public void runOpMode() {
    }

    public void auto(int condition) {
        double position = 0.38;
        double blue;
        double red;
        for(int i = 0; i <= 2; i++) {
            colorServo.setPosition(position);
            sleep(1000);
            int cntBlue = 0;
            int cntRed = 0;

            for (int j = 0; j < 1000; j++) {
                blue = sensorColor.blue();
                red = sensorColor.red();

                if (red <= 130 && blue >= 200 && blue <= 550) {
                    cntBlue++;
                    break;
                } else if (blue <= 130 && red >= 200 && red <= 550) {
                    cntRed++;
                    break;
                }
            }

            if (cntRed <= 500 && cntBlue <= 500) {
                position += 0.1;
                continue;
            } else if ((condition == 1 || condition == 2) && cntBlue - cntRed > 100) {
                driveByTime(1, -1, -1, 1, 0.5);
                break;
            } else if ((condition == 1 || condition == 2) && cntRed - cntBlue > 100) {
                driveByTime(-1, 1, 1, -1, 0.5);
                break;
            } else if ((condition == 3 || condition == 4) && cntBlue - cntRed > 100) {
                driveByTime(-1, 1, 1, -1, 0.5);
                break;
            } else if ((condition == 3 || condition == 4) && cntRed - cntBlue > 100) {
                driveByTime(1, -1, -1, 1, 0.5);
                break;
            }
        }



        colorServo.setPosition(1);

        sleep(1000);

        if(condition == 1) {
            driveByTime(-1, 1, 1, -1, 1);
        } else if(condition == 2) {
            driveByTime(-1, 1, 1, -1, 0.5);
            sleep(500);
            driveByTime(1, 1, 1, 1, 1.5);
        } else if(condition == 3) {
            driveByTime(1, -1, -1, 1, 0.5);
            sleep(500);
            driveByTime(1, 1, 1, 1, 1.5);
        } else if(condition == 4) {
            driveByTime(1, -1, -1, 1, 1);
        }

        sleep(50000);
    }

    public void driveByTime(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower, double time) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        ElapsedTime rtm = new ElapsedTime();
        rtm.reset();

        while(rtm.seconds() <= time) {

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
 }


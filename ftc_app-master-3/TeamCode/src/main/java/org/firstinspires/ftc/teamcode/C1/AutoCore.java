package org.firstinspires.ftc.teamcode.C1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kanexie on 31/03/2018.
 */

public class AutoCore {

    public static void auto(int condition, hardwarePushbot1 robot) {
        //VuforiaLocalizer vuforia;

        ElapsedTime runtime = new ElapsedTime();

        robot.claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runtime.reset();

        robot.claw.setPower(0.6);

        while(runtime.seconds() <= 0.5) {

        }

        robot.claw.setPower(0);

        runtime.reset();

        robot.elevator.setPower(1);

        while(runtime.seconds() <= 0.6) {

        }

        robot.elevator.setPower(0);
        /*int cameraMonitorViewId = hdMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hdMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdgZIY3/////AAAAmXUQWRXnWkG1kCjgBdoO3c8G2m/FaESxeNlMJDFcUSTPiuN1SR/w+ghJhbvgHTCRqSesas5+NlX/1uXmi2jD7T7AVrCL03AL6DqJqwWA5X8FVqrXtIbA7pLq892FSI1OHV9mvxlSy8bsvdDyIM7q1VF/pehaSitbenfkKJoui+4yuCyk6+HIfWLxNe8O/8GPrejOXl3OkjrWTVGerNntB8OGDhhIJJocAj3Wvp3ntPdUyMAZVBNqjgpPqlnO2cMvVBRYc8shPl8IByMXAK758cshNdZypVvqdQZdQTs388zBKDMtQlxw7s8opSgAcGZEH0eQgq/Hwr8WIOIm7xsysqIbW7oRLvFsqFGxomDf6nMc";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible.
                if(vuMark.equals("CENTER")) {
                    if(condition == 1) {
                        driveByTime(-1, 1, 1, -1, 0.8, robot);
                    } else if(condition == 2) {
                        driveByTime(-1, 1, 1, -1, 0.2, robot);
                        driveByTime(1, 1, 1, 1, 0.1, robot);
                    } else if(condition == 3) {
                        driveByTime(1, -1, -1, 1, 0.2, robot);
                        driveByTime(1, 1, 1, 1, 0.1, robot);
                    } else if(condition == 4) {
                        driveByTime(1, -1, -1, 1, 0.8, robot);
                    }
                }
        }
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it never\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\theless, for completeness. *//*
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

        */

        double position = 0.37;
        double blue;
        double red;

        for(int i = 0; i <= 2; i++) {
            robot.colorServo.setPosition(position);

            runtime.reset();

            while(runtime.seconds() <= 1) {

            }

            int cntBlue = 0;
            int cntRed = 0;

            for (int j = 0; j < 10; j++) {
                blue = robot.sensorColor.blue();
                red = robot.sensorColor.red();

                if (blue - red > 10) {
                    cntBlue++;
                } else if (red - blue > 10) {
                    cntRed++;
                }
            }

            if (cntRed <= 5 && cntBlue <= 5) {
                position += 0.03;
                continue;
            } else if ((condition == 1 || condition == 2) && cntBlue - cntRed >= 3) {
                robot.colorServo.setPosition(0.35);
                driveByTime(-1, 1, -1, 1, 0.7, robot);
                runtime.reset();
                while(runtime.seconds() <= 1) {

                }
                robot.colorServo.setPosition(0.98);
                break;
            } else if ((condition == 1 || condition == 2) && cntRed - cntBlue >= 3) {
                robot.colorServo.setPosition(0.35);
                driveByTime(1, -1, 1, -1, 0.7, robot);
                runtime.reset();
                while(runtime.seconds() <= 1) {

                }
                robot.colorServo.setPosition(0.98);
                break;
            } else if ((condition == 3 || condition == 4) && cntBlue - cntRed >= 3) {
                robot.colorServo.setPosition(0.35);
                driveByTime(1, -1, 1, -1, 0.7, robot);
                runtime.reset();
                while(runtime.seconds() <= 1) {

                }
                robot.colorServo.setPosition(0.98);
                break;
            } else if ((condition == 3 || condition == 4) && cntRed - cntBlue >= 3) {
                robot.colorServo.setPosition(0.35);
                driveByTime(-1, 1, -1, 1, 0.7, robot);
                runtime.reset();
                while(runtime.seconds() <= 1) {

                }
                robot.colorServo.setPosition(0.98);
                break;
            }
        }



        robot.colorServo.setPosition(0.98);

        runtime.reset();

        /*while(runtime.seconds() <= 1) {


        }
        if(condition == 1) {
            driveByTime(1, -1, -1, 1, 0.4, robot);
        } else if(condition == 2) {
            driveByTime(1, -1, -1, 1, 0.3, robot);
            driveByTime(1, 1, 1, 1, 0.4, robot);
        } else if(condition == 3) {
            driveByTime(-1, 1, 1, -1, 0.3, robot);
            driveByTime(1, 1, 1, 1, 0.4, robot);
        } else if(condition == 4) {
            driveByTime(-1, 1, 1, -1, 0.4, robot);
        }

        runtime.reset();

        while(runtime.seconds() <= 1) {

        }
        if(condition == 1) {
            driveByTime(1, -1, 1, -1, 1.5, robot);
        } else if(condition == 2) {
            driveByTime(-1, 1, -1, 1, 0.75, robot);
        } else if(condition == 3) {
            driveByTime(1, -1, 1, -1, 0.75, robot);
        } else if(condition == 4) {
            driveByTime(-1, 1, -1, 1, 1.5, robot);
        }

        robot.claw.setPower(-0.6);

        runtime.reset();

        while(runtime.seconds() <= 0.7) {

        }

        driveByTime(1, 1, 1, 1, 0.7, robot);
        runtime.reset();

        while(runtime.seconds() <= 0.4) {

        }
        driveByTime(-1, -1, -1, -1, 0.2, robot);
        */
        runtime.reset();

        while(runtime.seconds() <= 100) {

        }
    }

    public static void driveByTime(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower, double time, hardwarePushbot1 robot) {
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

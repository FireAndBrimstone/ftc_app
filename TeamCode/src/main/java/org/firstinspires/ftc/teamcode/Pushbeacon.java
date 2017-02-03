package org.firstinspires.ftc.teamcode;

/**
 * Created by Jedi on 2/2/2017.
 */

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;


import org.opencv.core.*;
import org.opencv.android.*;
import org.opencv.core.Rect;
import org.opencv.core.Mat;
import org.opencv.imgproc.*;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.R.id.log;


/**
 * Created by Jedi on 1/11/2017.
 */
// change localizer to vuforia

public class Pushbeacon extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();

    TouchSensor touchSensor;

    public final static Scalar blueLow = new Scalar(108, 0, 220);
    public final static Scalar blueHigh = new Scalar(178, 255, 255);

    // initalize values for analyzing the beacon
    public int BEACON_NOT_VISIBLE = 0;
    public int BEACON_RED_BLUE = 1;
    public int BEACON_BLUE_RED = 2;
    public int BEACON_ALL_BLUE = 3;
    public int BEACON_NO_BLUE = 4;


    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // encoder set up
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // touchsensor set up
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        int counter = 0;



        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        param.vuforiaLicenseKey = "";
        param.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer Vuforia = ClassFactory.createVuforiaLocalizer(param);
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = Vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels"); // choose the target you will use as a reference point
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        Vuforia.setFrameQueueCapacity(1);

        //VuforiaLocalizer localizer = null;
        //DcMotor left_motor = null;
        //DcMotor right_motor = null;
        //localizer.setFrameQueueCapacity(1);

        // move 3.5 feet or until you see the ball
        //move into the ball and turn left until you can pick up until you see wheels
        // get color
        // move left until you see legos
        // get color
        // move over to the left of the legos beacon
        // if legos blue back up 5 inch. and then set the servo arm to 1.0. Else go back 10inch. and set servo to 1.0
        // set servo to O.O
        // go back 1


        while (opModeIsActive() && wheels.getPose() == null){
            idle();
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        int config = VortexUtilites.getBeaconConfig(getImageFromFrame(Vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), wheels, Vuforia.getCameraCalibration());

        if (config == BEACON_BLUE_RED){
            //drive to right side of the beacon
            VuforiaDrive(wheels, robot.leftMotor, robot.rightMotor, 22);

        } else if (config == BEACON_RED_BLUE){
            //drive to the left side of the beacon
            VuforiaDrive(wheels, robot.leftMotor, robot.rightMotor, -22);

        }else{
            idle();
        }





    }

    private void VuforiaDrive(VuforiaTrackableDefaultListener wheels, DcMotor left_motor, DcMotor right_motor, float offset) {
        VectorF angles = anglesFromTarget(wheels);
//
        // Figures out the angle to the beacon
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500, offset,0));

        if(trans.get(0) > 0){
            left_motor.setPower(0.5);
            right_motor.setPower(-0.5); // rotates robot right
        }
        else {
            left_motor.setPower(-0.5); // rotates robot left
            right_motor.setPower(0.5);
        }

        do {
            if (wheels.getPose() != null){
                // checks if robot has rotated enough
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500, offset,0));
            }
            idle(); // stops the robot when the angle is right
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // 150 relates to the center of the robot
        robot.leftMotor.setTargetPosition((int)(robot.leftMotor.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575*1120 )));
        robot.rightMotor.setTargetPosition((int)(robot.rightMotor.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575*1120 )));

        robot.leftMotor.setPower(0.3);
        robot.rightMotor.setPower(0.3);

        while (opModeIsActive() && robot.rightMotor.isBusy() && robot.leftMotor.isBusy()){
            idle();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // rotate to face the target
        while (opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 0)){
            if (wheels.getPose() != null){
                if (wheels.getPose().getTranslation().get(0) > 0) {
                    robot.leftMotor.setPower(-0.3);
                    robot.rightMotor.setPower(0.3);
                } else{
                    robot.leftMotor.setPower(0.3);
                    robot.rightMotor.setPower(-0.3);
                }
            } else
            {
                robot.leftMotor.setPower(-0.3);
                robot.rightMotor.setPower(0.3);
            }
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){
        long numImgs = frame.getNumImages();

        for (int i = 0; i > numImgs; i++){
            if (frame.getImage(i).getFormat() == pixelFormat){
                return frame.getImage(i);
            }
        }
        return null;
    }

    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal){
        OpenGLMatrix pose = beacon.getPose();

        if (pose != null && img != null && img.getPixels() != null){
            Matrix34F rawPose = new Matrix34F();
            float [] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float [][]  corners = new float[4][2];
            // tool project point crops and expands the image -127mm and 276mm up see 7:00 of Vuforia part 7 video FIXIT3491
            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData();//upper left
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData();// upper right
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); // bottom right
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData();// bottom left

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);

            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            // get a filled mask
            // if pixel is within acceptable blue-beacon range change to white
            //else turn to black

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            if (mmnts.get_m00() > mask.total() * 0.8){
                return BEACON_ALL_BLUE;
            } else if (mmnts.get_m00() < mask.total() * 0.1){
                return BEACON_NO_BLUE;
            }

            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2 ){
                return BEACON_RED_BLUE;
            } else{
                return BEACON_BLUE_RED;
            }
            //android.util.Log.i("CentroidX", "", ((((double)mmnts.get_m10() / (double)mmnts.get_m00()))));
            //("CentroidX", "", ((mmnts.get_m10() / mmnts.get_m00())));
            //Log.i("CentroidY", "", ((mmnts.get_m01() / mmnts.get_m00())));





        }

        return BEACON_NOT_VISIBLE;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1),
                (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]); return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "RobotAutonomousV1", group = "Autonomous")
public class RobotAutonomousV1 extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "DataSet2.tflite";

    private static final String[] LABELS = {
            "cat2",
            "dragon3",
            "fire1"
    };

    private static final String VUFORIA_KEY = "AZetH0H/////AAABmTjP16rD8k5CnO3iYMJZujsciQ5c5u9++RhNXzWwd0IBUI+4g1qPayL+DehEaPzc7HWoPIvvC9i4sz6hojL2Cc+zM+J5vit7iAKB9jyp5eXHmLR2uORBsePaJwAOaRkz4/Jop/1Q9k0H+UE7eF6hgsa9K2Xbi9JYobrFJvD2i0WpSBIWB8UOo4Xo10UfCuUO2XaPe6xxT0znOQ5C8dEKNiaoSRKaBZQbP4bLf9orxWYFQKVCC5GP+MqYrFUOHKqh4c14mzxFfUeyZ7SW1Sn5BUwJtFrm8NSVxE9iH44nAcvWWTml/iowfb35FNM8O8v2lBjPH45XFmlHcApKwlKofqB/igo89yvuxOPEIXiPwTXQ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime  runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.25;
    static final double     TURN_SPEED    = 0.18;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        robot.init(hardwareMap);
        int catsNumber = 0;
        int fireNumber = 0;
        int dragonsNumber = 0;
        int status = 0;
        
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.8, 16.0/9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() && status != 1) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        fireNumber = 0;
                        catsNumber = 0;
                        dragonsNumber = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel() == "fire1"){
                                fireNumber++;
                            }
                            else if(recognition.getLabel() == "cat2"){
                                catsNumber++;
                            }
                            else if(recognition.getLabel() == "dragon3"){
                                dragonsNumber++;
                            }
                            else;
                            telemetry.addData("Image", "%s", recognition.getLabel());
                            telemetry.update();
                            if((fireNumber >= 2 && catsNumber <= 1 && dragonsNumber <= 1) || (fireNumber == 1 && catsNumber == 0 && dragonsNumber == 0)){
                                // go to area 
                                // sleep(10000);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(440);
                                robot.setAllPower(0);
                                sleep(100);
                                turn(84);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(1500);
                                robot.setAllPower(0);
                                sleep(100);
                                turn(-84);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(1600);
                                robot.setAllPower(0);
                                status = 1;
                                break;
                            }
                            else if((catsNumber >= 2 && fireNumber <= 1 && dragonsNumber <= 1) || (fireNumber == 0 && catsNumber == 1 && dragonsNumber == 0)){
                                // sleep(10000);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(2000);
                                robot.setAllPower(0);
                                status = 1;
                                break;
                            }
                            else if((dragonsNumber >= 2 && fireNumber <= 1 && catsNumber <= 1) || (fireNumber == 0 && catsNumber == 0 && dragonsNumber == 1)){
                                // sleep(10000);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(290);
                                robot.setAllPower(0);
                                sleep(100);
                                turn(-88);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(1500);
                                robot.setAllPower(0);
                                sleep(100);
                                turn(90);
                                robot.setAllPower(FORWARD_SPEED);
                                sleep(1500);
                                robot.setAllPower(0);
                                status = 1;
                                break;
                            }
                            else;
                        }
                    }
                    else{
                        // stop all motors
                    }
                }
            }
        }
    }
    // resets currAngle Value
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 1) {
            double motorPower = (error < 0 ? -TURN_SPEED : TURN_SPEED);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.setAllPower(0);
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");
        
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}

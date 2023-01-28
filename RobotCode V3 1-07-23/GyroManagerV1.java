package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@TeleOp(name = "Gyro", group = "Autonomous")
public class GyroManager extends LinearOpMode{

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime  runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.25;
    static final double     TURN_SPEED    = 0.18;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
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

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            // robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            robot.setMotorPower(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED);
            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }
}

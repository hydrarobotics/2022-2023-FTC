package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Map;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    
    /* Public OpMode members. */
    public DcMotor LBmotor;
    public DcMotor LFmotor;
    public DcMotor RBmotor;
    public DcMotor RFmotor;
  
    public DcMotor spoolMotor;
    
    public Servo tiltServo;
    public Servo gripServo;
    
    //Define the IMU used for gyro related tasks
    BNO055IMU imu;
    
    //Creating default positions for servos to use
    public final static double TILT_SERVO_HOME = 0.0;
    public final static double TILT_SERVO_MAX = 1.0;
    public final static double TILT_SERVO_MIN = 0.0;

    public final static double GRIP_SERVO_HOME = 0.4;
    public final static double GRIP_SERVO_MAX = 1.0;
    public final static double GRIP_SERVO_MIN = 0.0;
    
    //Local OpMode members.
    HardwareMap hardwareMap = null;
    private ElapsedTime period  = new ElapsedTime();
   

    //Initialize standard Hardware interfaces
    public void init(HardwareMap hardwareMap) {
        
        // Define and Initialize Motors
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        spoolMotor = hardwareMap.get(DcMotor.class, "spoolMotor");
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        
        // Define and Initialize Servos
        tiltServo = hardwareMap.get(Servo.class, "tiltServo"); 
        //flipServo = hardwareMap.get(Servo.class, "flipServo");
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        
        // Set all motors to zero power
        LBmotor.setPower(0.0);
        LFmotor.setPower(0.0);
        RBmotor.setPower(0.0);
        RFmotor.setPower(0.0);
        spoolMotor.setPower(0.0);
        
        //Set servo positions to starting values
        tiltServo.setPosition(TILT_SERVO_HOME);
        gripServo.setPosition(GRIP_SERVO_HOME);
        
        //Use RUN_USING_ENCODERS if encoders are installed.
        //Use RUN_WITHOUT_ENCODERS if encoders are not installed.
        LBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        spoolMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        //Set to 0 Power behavior - float causes power to slowly stop vs break
        //which stops the robot
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
        //Setting parameters for the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
  
    //Method to set all power to a sepcific value
    public void setAllPower(double p) {
        setMotorPower(p,p,p,p);
    }
    
    //Method to set all the motors to their own specific values
    public void setMotorPower(double rF, double lF, double rB, double lB) {
        LFmotor.setPower(-lF);
        LBmotor.setPower(-lB);
        RBmotor.setPower(rB);
        RFmotor.setPower(rF);
    }
}

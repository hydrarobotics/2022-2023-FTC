
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RobotV1", group="Iterative Opmode")

public class RobotV1 extends OpMode {
  
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LBmotor;
    private DcMotor LFmotor;
    private DcMotor RBmotor;
    private DcMotor RFmotor;
    
    private DcMotor spoolMotor;
    private DcMotor armMotor;
    
    private Servo gripServo;
    private Servo tiltServo;
    private Servo flipServo;

    RobotHardware robot = new RobotHardware();
    
    //Motor speeds
    final double driveTrainPower = .75;
    
    //Set initial servo positions
    double tiltServoPosition = robot.TILT_SERVO_HOME;
    double flipServoPosition = robot.FLIP_SERVO_HOME;
    double gripServoPosition = robot.GRIP_SERVO_HOME;
    
    //Set servo speeds
    final double TILT_SPEED = 0.01;
    final double FLIP_SPEED = 0.01;
    final double GRIP_SPEED = 0.01;

    @Override
    public void init() {
        robot.init(hardwareMap);   
        
    }
    
    //Driver code
    @Override
    public void loop() {
        
        //Define Controller Inputs
        double G1RightStickX = gamepad1.right_stick_x;
        double G1RightStickY = gamepad1.right_stick_y;
        double G1rightTrigger = gamepad1.right_trigger;
        double G1leftTrigger = gamepad1.left_trigger;
        double G1leftStickX = gamepad1.left_stick_x;
        boolean G1LBumper = gamepad1.left_bumper;
        boolean G1RBumper = gamepad1.right_bumper;
        boolean G1A = gamepad1.a;
        boolean G1B = gamepad1.b;        
        boolean G1X = gamepad1.x;
        boolean G1Y = gamepad1.y;
        
        boolean G2Y = gamepad2.y;
        boolean G2A = gamepad2.a;
        
        //Drivetrain variables
        double turn = G1leftStickX; //Turn amount
        double power= 0; 
        double LeftMotorBP = 0; 
        double RightMotorBP = 0;
        double LeftMotorFP = 0;
        double RightMotorFP = 0;
        boolean nopower;
        
        //Lift varibles
        double armPower = 0;
        double spoolPower = 0;
        
        //Lift for robot
        if (G1RightStickX < 0 || G1RightStickX > 0 ) {
            armPower = (G1RightStickX/2);
        } else {

        } 
        
        if (G1RightStickY < 0 || G1RightStickY > 0 ) {
            spoolPower = (G1RightStickY);
        }      
        
        //Claw system
        if (G1A) {
            gripServoPosition = 0.25;
        } else {
            gripServoPosition = 0.0;
        }
        
        if(G2Y) {
            tiltServoPosition = 0.1;
        } else { 
            tiltServoPosition = 1.0;
        }
            
        if(G2A) {
            flipServoPosition = 0.0;
        } else {
            flipServoPosition = 1.0;
        }
        
    //Setting Forward/Backwards Power
        if(G1leftTrigger > 0) {
            power = -G1leftTrigger;
            turn = turn * -1;
        }
        if(G1rightTrigger > 0) {
            power = G1rightTrigger;
        }
        
        if (power != 0){
             nopower = false;
         }
         else {
             nopower = true;
         }
    
    //Turning logic
        if (turn > 0 && !nopower){
            if (power > 0) {
                RightMotorBP = (power + (turn*-1.25));
                LeftMotorBP = power;
                RightMotorFP = (power + (turn*-1.25));
                LeftMotorFP = power;
            } else {
                RightMotorBP = (power - (turn*-1.25));
                LeftMotorBP = power;
                RightMotorFP = (power - (turn*-1.25));
                LeftMotorFP = power;
            }
        } else if (turn < 0 && !nopower){
            if (power > 0) {
                RightMotorBP = power;
                LeftMotorBP = (power + (turn*1.25));
                RightMotorFP = power;
                LeftMotorFP = (power + (turn*1.25));
            } else {
                RightMotorBP = power;
                LeftMotorBP = (power - (turn*1.25));
                RightMotorFP = power;
                LeftMotorFP = (power - (turn*1.25));
            }
        }

        if (turn > 0 && nopower){
            RightMotorBP = (power - (turn));
            LeftMotorBP = (power + (turn));
            RightMotorFP = (power - (turn));
            LeftMotorFP = (power + (turn));
        }
        else if (turn < 0 && nopower){
            RightMotorBP = (power - (turn));
            LeftMotorBP = (power + (turn));
            RightMotorFP = (power - (turn));
            LeftMotorFP = (power + (turn));
        }
        
        if (turn == 0) {
            RightMotorBP = power;
            LeftMotorBP = power;
            RightMotorFP = power;
            LeftMotorFP = power;
        }
        
        //Strafing logic
        if (G1LBumper) {
            LeftMotorBP = 1;
            RightMotorBP = -1; 
            LeftMotorFP = -1;
            RightMotorFP = 1;
        }
        if (G1RBumper) {
            LeftMotorBP = -1;
            RightMotorBP = 1; 
            LeftMotorFP = 1;
            RightMotorFP = -1;
        }
        

    //Setting power to motors
    robot.LBmotor.setPower(-LeftMotorBP * driveTrainPower);
    robot.RBmotor.setPower(RightMotorBP * driveTrainPower); 
    robot.LFmotor.setPower(-LeftMotorFP * driveTrainPower);
    robot.RFmotor.setPower(RightMotorFP * driveTrainPower);
    robot.armMotor.setPower(armPower);
    robot.spoolMotor.setPower(spoolPower);
    
    //Clip servo ranges
    tiltServoPosition = Range.clip(tiltServoPosition, robot.TILT_SERVO_MIN, robot.TILT_SERVO_MAX);
    flipServoPosition = Range.clip(flipServoPosition, robot.FLIP_SERVO_MIN, robot.FLIP_SERVO_MAX);
    gripServoPosition = Range.clip(gripServoPosition, robot.GRIP_SERVO_MIN, robot.GRIP_SERVO_MAX);
    
    //Setting servo positions
    robot.tiltServo.setPosition(tiltServoPosition);
    robot.flipServo.setPosition(flipServoPosition);
    robot.gripServo.setPosition(gripServoPosition);
        
    //Display data to driver
    telemetry.addData("titlpos", tiltServoPosition);
        
    }
}

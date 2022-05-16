/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;


public class ExampleSubsystem extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */


    // motors
    TitanQuad leftMotor;
    TitanQuad rightMotor;

    // encoder
    private final TitanQuadEncoder leftEncoder;
    private final TitanQuadEncoder rightEncoder;
    
    //pose
    public double x = 0;
    public double y = 0;
    public double theta = 0 ;

    // waypoints
    public double x_D = 0;
    public double y_D = 0;
    public double theta_D = 0;
    public double dS = 0;

    // velocity
    public double u = 0;

    public double setpoint = 0;

    // PID const

    public double kP = 10;
    public double kPV = 50;


    public ExampleSubsystem() {
        leftMotor = new TitanQuad(Constants.titan_id, Constants.frequency_motor, Constants.left_motor);
        rightMotor = new TitanQuad(Constants.titan_id, Constants.frequency_motor, Constants.right_motor);

        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.right_motor, Constants.distancePerTick);
        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.left_motor, Constants.distancePerTick);
        leftEncoder.setReverseDirection();
        resetEncoders();


        //m_dr.arcadeDrive(-joy.getRawAxis(1), joy.getRawAxis(0));
    }
    public void resetEncoders(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    // Velocity
    public double robotV() {return (rightLenV() + leftLenV())/2;}

    public double rightLenV() {return (Constants.wheelRadius * Math.PI * rightMotor.getRPM()) / 60;}

    public double leftLenV() {return -((Constants.wheelRadius * Math.PI * leftMotor.getRPM()) / 60);}

    // Odometry
    public void getTheta(double dt) {
        theta += ((rightLenV()-leftLenV()) / Constants.transmission_width)*dt;
        //theta = adduction(theta);
    }

    public void getX(double dt) {x += Math.cos(theta) * robotV() * dt;}
    public void getY(double dt) {y += Math.sin(theta) * robotV() * dt;}

    public void resetPose() {
        this.x = 0;
        this.y = 0;
        this.theta =0;
    }

    //PID

    public void velocity(double x_W, double y_W, double theta_w,double dt) {
        x_D = x_W - x;
        y_D = y_W - y;
        dS = Math.sqrt((Math.pow(x_D, 2) + Math.pow(y_D, 2)));
        u = (Math.atan2(y_D, x_D) - theta)/dt;
        SmartDashboard.putNumber("angel degrees", Math.toDegrees(Math.atan2(y_D, x_D)));


        double PD_velocity = kPV*dS;
        double PD_t = kP*u;
        double L_Saturate = MathUtil.clamp(((2*PD_velocity)-(PD_t*Constants.transmission_width))/2, -0.2, 0.2);
        double R_Saturate = MathUtil.clamp(((2*PD_velocity)+(PD_t*Constants.transmission_width))/2, -0.2, 0.2);


        if (Math.abs(x_D) < 0.05 && Math.abs(y_D) < 0.05) {
            leftMotor.set(0);
            rightMotor.set(0);
            if (theta_w > 0) {
                double e_a = theta_w - theta;
                double pid_rot = kP*e_a;
                SmartDashboard.putNumber("e_a", Math.abs(e_a));
                if (Math.abs(e_a) < 0.15){
                    leftMotor.set(0);
                    rightMotor.set(0);
                } else {
                    L_Saturate = MathUtil.clamp(pid_rot, -0.2, 0.2);
                    R_Saturate = MathUtil.clamp(pid_rot, -0.2, 0.2);
                    leftMotor.set(-L_Saturate);
                    rightMotor.set(-R_Saturate);
                }
            }

        } else {
            leftMotor.set(L_Saturate);
            rightMotor.set(-R_Saturate);

        }
    }


    public void rotated(double theta_w) {
        double e_a = theta_w - theta;
        double pid_rot = kP*e_a;
        SmartDashboard.putNumber("e_a", Math.abs(e_a));
        if (Math.abs(e_a) < 0.15){
            leftMotor.set(0);
            rightMotor.set(0);
        } else {
            double L_Saturate = MathUtil.clamp(pid_rot, -0.2, 0.2);
            double R_Saturate = MathUtil.clamp(pid_rot, -0.2, 0.2);
            leftMotor.set(-L_Saturate);
            rightMotor.set(-R_Saturate);
        }
    }


    @Override
    public void periodic() {
        NetworkTableInstance.getDefault().flush();
        SmartDashboard.putNumber("dist", setpoint);

        SmartDashboard.putNumber("theta", Math.toDegrees(theta));
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("u", Math.toDegrees(u));
        SmartDashboard.putNumber("dS", dS);


    }
}

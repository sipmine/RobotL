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
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public double theta =0 ;

    // waypoints
    public double x_D = 0;
    public double y_D = 0;
    public double theta_D = 0;
    public double dS = 0;

    // velocity
    public double u = 0;

    // PID const
    private PIDController pidControllerVelocity;
    private PIDController pidControllerHeading;



    public ExampleSubsystem() {
        leftMotor = new TitanQuad(Constants.titan_id, Constants.frequency_motor, Constants.left_motor);
        rightMotor = new TitanQuad(Constants.titan_id, Constants.frequency_motor, Constants.right_motor);

        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.right_motor, Constants.distancePerTick);
        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.left_motor, Constants.distancePerTick);
        leftEncoder.setReverseDirection();
        resetEncoders();
        pidControllerVelocity = new PIDController(Constants.kP_V, Constants.kI_V, 0);
        pidControllerHeading = new PIDController(Constants.kP_H, 0, 0);
    }
    public void resetEncoders(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double adduction(double a){
        while (a > Math.PI) {a = a-2*Math.PI;}
        while (a < -Math.PI) {a = a+2*Math.PI;}
        return a;
    }

    // Velocity
    public double robotV() {return (rightLenV() + leftLenV())/2;}

    public double rightLenV() {return (Constants.wheelRadius * Math.PI * rightMotor.getRPM()) / 60;}

    public double leftLenV() {return -((Constants.wheelRadius * Math.PI * leftMotor.getRPM()) / 60);}

    // Odometry
    public void getTheta(double dt) {
        theta += ((rightLenV()-leftLenV()) / Constants.transmission_width)*dt;
        theta = adduction(theta);
    }

    public void getX(double dt) {x += Math.cos(theta) * robotV() * dt;}
    public void getY(double dt) {y += Math.sin(theta) * robotV() * dt;}

    public void resetPose() {
        this.x = 0;
        this.y = 0;
        this.theta =0;
    }

    //PID

    public void velocity(double x_W, double y_W, double theta_W) {
        x_D = x_W - x;
        y_D = y_W - y;
        dS = Math.sqrt((Math.pow(x_D, 2) + Math.pow(y_D, 2)));
        u = Math.atan2(y_D, x_D) - theta;
        theta_D = theta_W - theta;
        pidControllerVelocity.setSetpoint(u);
        pidControllerHeading.setSetpoint(theta_D);
        double PD_velocity = pidControllerVelocity.calculate(dS);

        double PD_heading = pidControllerHeading.calculate(theta);

        if (dS < 0.15) {
            leftMotor.set(0);
            rightMotor.set(0);
        } else {
            leftMotor.set(-(PD_velocity) + PD_heading);
            rightMotor.set((PD_velocity) - PD_heading);
        }

    }



    @Override
    public void periodic() {
        NetworkTableInstance.getDefault().flush();
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("u", u);

        SmartDashboard.putNumber("dS", dS);

    }
}

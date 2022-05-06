/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    //Titan id
    public static final int titan_id = 42;

    //Motors
    public static final int left_motor = 2;
    public static final int right_motor = 3;



    public static final double transmission_width = 0.27;

    public static final double wheelRadius = 0.045;
    public static final int pulsePerRotation = 1464;
    public static final double gearRatio = 1 / 1;
    public static final double encoderPulseRatio = pulsePerRotation * gearRatio;
    public static final double distancePerTick = (Math.PI * 2 * wheelRadius) / encoderPulseRatio;
    public static final int frequency_motor = 1000;

    // PID k
    public static final double kP_V = 3.9;
    public static final double kI_V = 0;

    public static final double kP_H = 5;
}
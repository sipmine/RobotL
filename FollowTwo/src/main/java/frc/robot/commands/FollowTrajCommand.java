package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

import javax.swing.plaf.IconUIResource;


public class FollowTrajCommand extends CommandBase {
    private  double lastTime;
    public final ExampleSubsystem subsystem = RobotContainer.exampleSubsystem;
    public FollowTrajCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        subsystem.resetPose();
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double dT = currentTime - lastTime;
        lastTime = currentTime;

        subsystem.velocity(0, 0.5, 0);
        //subsystem.rotated(-0.2);
        subsystem.getTheta(dT);
        subsystem.getX(dT);
        subsystem.getY(dT);



    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

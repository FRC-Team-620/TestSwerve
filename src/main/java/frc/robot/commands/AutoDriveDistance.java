package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveDistance extends CommandBase{
    private DriveSubsystem drive;
    private double desiredDistance;
    private double maxSpeed;
    private ProfiledPIDController pidController;
    private Pose2d start;
    private boolean fieldRelatvie;

    public AutoDriveDistance(DriveSubsystem drive, double desiredDistance, double maxSpeed, boolean fieldRelative){
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.maxSpeed = maxSpeed;
        this.fieldRelatvie = fieldRelative;
        
        pidController = new ProfiledPIDController(0.2, 0, 0, new Constraints(this.maxSpeed, 0.2));
        SmartDashboard.putData(this.pidController);

        addRequirements(drive);
    } 

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        this.start = this.drive.getPose();
        
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        Pose2d current = this.drive.getPose();
        double currentDistance = current.getTranslation().getDistance(this.start.getTranslation());
        double output = MathUtil.clamp(pidController.calculate(currentDistance, this.desiredDistance), -this.maxSpeed, this.maxSpeed);
        this.drive.drive(output, 0, 0, this.fieldRelatvie, false);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        Pose2d current = this.drive.getPose();
        double currentDistance = current.getTranslation().getDistance(this.start.getTranslation());
        return (Math.abs(this.desiredDistance-currentDistance) <= 0.1);
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        this.drive.drive(0, 0, 0, true, false);
    }
}

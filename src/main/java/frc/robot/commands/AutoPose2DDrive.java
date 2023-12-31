package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// TODO: I am not sure if rotation works as intended(didn't know how rotation works)
public class AutoPose2DDrive extends CommandBase{
    private DriveSubsystem drive;
    private Pose2d startPose;
    private Pose2d desiredPose;
    private ProfiledPIDController xControl;
    private ProfiledPIDController yControl;
    private ProfiledPIDController rotationControl;

    private double desiredX;
    private double desiredY;
    //rotation is in radian
    private double desiredRot;

    private double maxTranslation;
    private double maxRotation;

    private boolean fieldRelative;
    public AutoPose2DDrive(DriveSubsystem drive, Pose2d desiredPose, double maxTranslation, double maxRotation, boolean fieldRelative){
        this.drive = drive;
        this.desiredPose = desiredPose;
        this.maxTranslation = maxTranslation;
        //TODO: what value?
        this.maxRotation = maxRotation;
        desiredX = this.desiredPose.getTranslation().getX();
        desiredY = this.desiredPose.getTranslation().getY();
        // TODO: radian/degree?
        desiredRot = this.desiredPose.getRotation().getDegrees();

        this.fieldRelative = fieldRelative;
        this.xControl = new ProfiledPIDController(0.2, 0, 0, new Constraints(maxTranslation, 0.2));
        this.yControl = new ProfiledPIDController(0.2, 0,0, new Constraints(maxTranslation, 0.2));
        this.rotationControl = new ProfiledPIDController(0.2, 0, 0, new Constraints(maxRotation, 0.2));
    }
    
    
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        startPose = this.drive.getPose();
    }
    @Override
    public void execute() {
        // curPose is the current pose get from drivetrain
        Pose2d curPose = this.drive.getPose();
        
        // ())Dis gives the robot's current distance from the start(origin)
        double xDis = curPose.getTranslation().getDistance(this.startPose.getTranslation());
        double yDis = curPose.getTranslation().getDistance(this.startPose.getTranslation());
        double rDis = (startPose.getRotation().getDegrees() - curPose.getRotation().getDegrees());

        double xOutput = xControl.calculate(xDis, desiredX);
        double yOutput = yControl.calculate(yDis, desiredY);
        double rOutput = rotationControl.calculate(rDis, desiredRot);

        this.drive.drive(xOutput, yOutput, rOutput, this.fieldRelative, false);

    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        this.drive.drive(0,0,0,this.fieldRelative,false);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        Pose2d curPose = this.drive.getPose();
        boolean atX = (this.desiredX - curPose.getTranslation().getX() <= 0.1);
        boolean atY = (this.desiredY - curPose.getTranslation().getY() <= 0.1);
        boolean atR = (this.desiredRot - curPose.getRotation().getDegrees() <= 0.1);

        return atX && atY && atR;
    }
}

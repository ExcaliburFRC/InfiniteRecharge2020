package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ChassiCommands.TimedStrightDrive;
import frc.robot.GeneralCommands.LowerShootProccess;
import frc.robot.GeneralCommands.ShootProccess;
import frc.robot.GeneralCommands.Wait;

public class AutoCommandGenerator{
    public static Command goBackAndShoot(){
        var shootCommand = new ShootProccess(false);
        var shooterWaitCommand = new Wait(8);
        var moveBack = new TimedStrightDrive(1100, -0.6);

        return moveBack.andThen(shootCommand);
    }

    public static Command goToLowerAndShoot(){
        var moveForward= new TimedStrightDrive(1500, 0.6);
        var shooterWaitCommand = new Wait(6);
        var shootCommand = new LowerShootProccess(true);
        
        return moveForward.andThen(shooterWaitCommand.deadlineWith(shootCommand));
    }
}
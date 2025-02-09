/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunReverseIntake extends CommandBase
{
    /**
     * Creates a new RunReverseIntake.
     */
    Intake m_intake;
    public RunReverseIntake(Intake intake)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        //DriverStation.reportWarning("Intake Reverse Command running", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        m_intake.setIntakePower(-Constants.kIntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted)
    {
        m_intake.setIntakePower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}

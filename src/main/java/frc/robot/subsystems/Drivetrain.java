/**
 * This Class is For Controlling our Robot Drive Train
 * it has All our Functions Organized and used in all most all of our  commands
 * @author Luke Crum, Nicholas Blackburn
 */
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


import frc.robot.util.PID;

public class Drivetrain extends SubsystemBase
{
    private final WPI_TalonSRX m_leftFrontDriveMotor;
    private final WPI_TalonSRX m_rightFrontDriveMotor;
    private final WPI_TalonSRX m_leftRearDriveMotor;
    private final WPI_TalonSRX m_rightRearDriveMotor; 

    private final ADXRS450_Gyro m_gyro;


    private final PID m_turnPID;
    private final PID m_drivePID;

    private final SpeedControllerGroup leftDriveGroup; 
    private final SpeedControllerGroup rightDriveGroup;

    private final DifferentialDrive m_drive;

    private double m_gyroWorkingZero = 0;

    public Drivetrain(WPI_TalonSRX leftFrontDriveMotor, WPI_TalonSRX rightFrontDriveMotor, 
                      WPI_TalonSRX leftRearDriveMotor, WPI_TalonSRX rightRearDriveMotor,
                      ADXRS450_Gyro gyro)
    {
        System.out.println("\t In Drivetrain Constructor");
        
        m_leftFrontDriveMotor = leftFrontDriveMotor;
        m_rightFrontDriveMotor = rightFrontDriveMotor;
        m_leftRearDriveMotor = leftRearDriveMotor;
        m_rightRearDriveMotor = leftRearDriveMotor;

        m_gyro = gyro;// = new ADXRS450_Gyro();


        m_turnPID = new PID(Constants.PTurn, Constants.ITurn, Constants.DTurn, Constants.turnEpsilon);
        m_drivePID = new PID(Constants.PDrive, Constants.IDrive, Constants.DDrive, 1.0);



        leftDriveGroup = new SpeedControllerGroup(m_leftFrontDriveMotor, m_leftRearDriveMotor);
        rightDriveGroup = new SpeedControllerGroup(m_rightFrontDriveMotor, m_rightRearDriveMotor);
    
        m_drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

        driveTrainConfig();
    }


    /**
     * Gets creates and Configures the Drivetrain used in other subsystems
     *
     * @author Nicholas Blackburn
     */
    private void driveTrainConfig()
    {
        // leftDriveGroup.get
    
        m_leftFrontDriveMotor.configFactoryDefault();
        m_leftRearDriveMotor.configFactoryDefault();
        m_rightFrontDriveMotor.configFactoryDefault();
        m_rightRearDriveMotor.configFactoryDefault();

        m_turnPID.setMaxOutput(1.0);
        m_drivePID.setMaxOutput(1.0);

        m_rightFrontDriveMotor.setInverted(true);
        m_rightRearDriveMotor.setInverted(true);

        m_leftFrontDriveMotor.configOpenloopRamp(Constants.kRampRate);
        m_leftRearDriveMotor.configOpenloopRamp(Constants.kRampRate);
        m_rightFrontDriveMotor.configOpenloopRamp(Constants.kRampRate);
        m_rightRearDriveMotor.configOpenloopRamp(Constants.kRampRate);

        m_leftFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_leftRearDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_rightFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_rightRearDriveMotor.setNeutralMode(NeutralMode.Brake);

        m_leftFrontDriveMotor.enableCurrentLimit(true);
        m_leftFrontDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_leftFrontDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_leftFrontDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_leftRearDriveMotor.enableCurrentLimit(true);
        m_leftRearDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_leftRearDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_leftRearDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_rightFrontDriveMotor.enableCurrentLimit(true);
        m_rightFrontDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_rightFrontDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_rightFrontDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_rightRearDriveMotor.enableCurrentLimit(true);
        m_rightRearDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_rightRearDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_rightRearDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);
    }

    public void autoDriveTrainConfig()
    {
        m_leftFrontDriveMotor.configFactoryDefault();
        m_leftRearDriveMotor.configFactoryDefault();
        m_rightFrontDriveMotor.configFactoryDefault();
        m_rightRearDriveMotor.configFactoryDefault();

        m_turnPID.setMaxOutput(1.0);
        m_drivePID.setMaxOutput(1.0);

        m_rightFrontDriveMotor.setInverted(true);
        m_rightRearDriveMotor.setInverted(true);

        m_leftFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_leftRearDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_rightFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_rightRearDriveMotor.setNeutralMode(NeutralMode.Brake);

        m_leftFrontDriveMotor.enableCurrentLimit(true);
        m_leftFrontDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_leftFrontDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_leftFrontDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_leftRearDriveMotor.enableCurrentLimit(true);
        m_leftRearDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_leftRearDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_leftRearDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_rightFrontDriveMotor.enableCurrentLimit(true);
        m_rightFrontDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_rightFrontDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_rightFrontDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_rightRearDriveMotor.enableCurrentLimit(true);
        m_rightRearDriveMotor.configContinuousCurrentLimit(Constants.kContinuousCurrentLimit);
        m_rightRearDriveMotor.configPeakCurrentDuration(Constants.kPeakCurrentDuration);
        m_rightRearDriveMotor.configPeakCurrentLimit(Constants.kPeakCurrentLimit);

        m_leftFrontDriveMotor.setSafetyEnabled(false);
        m_leftRearDriveMotor.setSafetyEnabled(false);
        m_rightFrontDriveMotor.setSafetyEnabled(false);
        m_rightRearDriveMotor.setSafetyEnabled(false);
    }


    /**
     * Cal's Gyro to current pos of robot for MAXIMUM ACCURACY
     *
     * @param Gyro
     *
     * @author Nicholas Blackburn
     */
    public void calibrateGyro()
    {
        m_gyro.calibrate();
    }

    /**
     * Sets Drivetrain Power to 3% speed
     *
     * @param Drivetrain
     *
     * @author Nicholas Blackburn
     */
    public void driveForwardSlowly()
    {
        m_drive.arcadeDrive(0.35, 0);
    }

    /**
     *
     * Sets @param Drivetrain Motor Power in a simple and Organized way
     *
     * @author Luke Crumb
     */
    public void setLeftRightPower(final double left, final double right)
    {
        m_leftFrontDriveMotor.set(left);
        m_leftRearDriveMotor.set(left);
        m_rightFrontDriveMotor.set(right);
        m_rightRearDriveMotor.set(right);
    }

    /**
     * Distance in a Simple return type The output Should Be used in A Distace
     * Related Method Gets
     *
     * @param LeftEncoder
     *
     * @author Nicholas Blackburn
     */
    public double leftEncoderDistance()
    {
        return -m_leftFrontDriveMotor.getSelectedSensorPosition();
    }

    /**
     *
     * Gets Distance in a Simple return type The output Should Be used in A Distace
     * Related Method
     *
     * @param RightEncoder
     *
     * @author Nicholas Blackburn
     */
    public double rightEncoderDistance()
    {
        return -m_rightFrontDriveMotor.getSelectedSensorPosition();
    }

    /**
     * Gets Rate in a Simple return type The output Should Be used in A Rate Related
     * Method
     *
     * @param LeftEncoder
     *
     * @author Nicholas Blackburn
     */
    public double leftEncoderRate()
    {
        return -m_leftFrontDriveMotor.getSelectedSensorVelocity();
    }

    /**
     * Gets Rate in a Simple return type The output Should Be used in A Rate Related
     * Method
     *
     * @param RightEncoder
     *
     * @author Nicholas Blackburn
     */
    public double rightEncoderRate()
    {
        return -m_leftFrontDriveMotor.getSelectedSensorVelocity();
    }

    /**
     * Gets Encoders Rate in a Simple return type The output Should Be used in A
     * Rate Related Method
     *
     * @param Encoders
     *
     * @author Nicholas Blackburn
     */
    public void zeroSensors()
    {
        m_gyro.reset();
        m_leftFrontDriveMotor.setSelectedSensorPosition(0);
        m_rightFrontDriveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets Gyro Yaw in a Simple return type The output Should Be used in A Double
     * Related Method
     *
     * @param Gyro
     *
     * @author Nicholas Blackburn
     */

    public double getGyroYaw()
    {
        return m_gyro.getAngle() - m_gyroWorkingZero;
    }

    /**
     * Sets Gyro Yaw in a Simple Double type The output Should Be used in A Double
     * Related Method
     *
     * @param Gyro
     *
     * @author Nicholas Blackburn
     */
    public void setGyroYaw(final double yaw)
    {
        m_gyroWorkingZero = m_gyro.getAngle() - yaw;
    }

    /**
     * Sets Motor Power to 0
     *
     * @param Drivetrain
     * @param TalonSRX
     *
     * @author Nicholas Blackburn
     */
    public void stop()
    {
        m_leftRearDriveMotor.set(0);
        m_leftFrontDriveMotor.set(0);
        m_rightRearDriveMotor.set(0);
        m_rightFrontDriveMotor.set(0);
    }

    /**
     *
     * Sets Motor Power of Front Left motor for Drivetrain
     *
     *
     * @param Drivetrain
     *
     * @author Nicholas Blackburn
     */
    public void setFrontLDrive(final double power)
    {
        m_leftFrontDriveMotor.set(power);
    }

    /*
     * Sets Motor Power For Backl motor for Drivetrain
     *
     * @param Driver
     *
     * @author Nicholas Blackburn
     */
    public void setBackLDrive(final double power)
    {
        m_leftRearDriveMotor.set(power);
    }

    public void setFrontRDrive(final double power)
    {
        m_rightFrontDriveMotor.set(power);
    }

    public void setBackRDrive(final double power)
    {
        m_rightRearDriveMotor.set(power);
    }

    public boolean angleIsStable = false;

    public void turnToAngle(final double setpointAngle)
    {
        final double currentAngle = getGyroYaw();
        m_turnPID.setMaxOutput(1.0);
        m_turnPID.setConstants(Constants.PTurn, Constants.ITurn, Constants.DTurn);

        if (!m_turnPID.isDone())
        {
            m_turnPID.setDesiredValue(setpointAngle);
            final double turnPower = m_turnPID.calcPID(currentAngle);
            setLeftRightPower(turnPower, -turnPower);
            angleIsStable = false;
        }
        else
        {
            setLeftRightPower(0, 0);
            angleIsStable = true;
        }
    }

    public boolean distanceIsStable = false;

    /*public void driveToDistancePID(final double setpointDistance, final double maxPower, final double angle) {
        if (!drivePID.isDone()) {
            turnPID.setConstants(Constants.PdriveTurn, Constants.IdriveTurn, Constants.DdriveTurn);
            turnPID.setMaxOutput(.6);
            drivePID.setMaxOutput(maxPower);
            drivePID.setDesiredValue(setpointDistance);
            turnPID.setDesiredValue(angle);
            final double power = drivePID.calcPID(leftEncoder.getDistance() / 217.3);
            final double turn = turnPID.calcPID(getGyroYaw());
            setLeftRightPower(power + turn, power - turn);
            distanceIsStable = false;
        } else {
            setLeftRightPower(0, 0);
            distanceIsStable = true;
        }
    }*/

    boolean turned = false;

    /*public void fastTurnAndDriveDistancePID(final double setpointDistance, final double maxPower, final double angle) {
        final double currentAngle = getGyroYaw();
        if (!turned) {
            turnPID.setConstants(Constants.PTurn, Constants.ITurn, Constants.DTurn);
            turnPID.setDesiredValue(angle);
            final double turnPower = turnPID.calcPID(currentAngle);
            setLeftRightPower(turnPower, -turnPower);
            if (Math.abs(currentAngle - angle) < 5)
                turned = true;
            else
                turned = false;
        } else if (!drivePID.isDone()) {
            turnPID.setConstants(Constants.PdriveTurn, Constants.IdriveTurn, Constants.DdriveTurn);
            drivePID.setMaxOutput(maxPower);
            drivePID.setDesiredValue(setpointDistance);
            turnPID.setDesiredValue(angle);
            final double power = drivePID.calcPID(leftEncoder.getDistance());
            final double turn = turnPID.calcPID(getGyroYaw());
            setLeftRightPower(power + turn, power - turn);
            distanceIsStable = false;
        } else {
            setLeftRightPower(0, 0);
            distanceIsStable = true;
        }

    }*/

    public void resetPID()
    {
        turned = false;
        distanceIsStable = false;
        angleIsStable = false;
        zeroSensors();
    }

    /*
     * public void holdPosition() { final double left = leftEncoder.getDistance() *
     * 217.3 * Constants.PHold; final double right = rightEncoder.getDistance() *
     * 217.3 * Constants.PHold; setLeftRightPower(left, right); }
     */

    public void arcadeDrive(final double throttle, final double turn)
    {
        m_drive.arcadeDrive(throttle, turn);
    }

    public void deadbandedArcadeDrive()
    {
        double throttle, turn;
        if (RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) > 0.1
                || RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) < -0.1)
        {
            if (RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) < 0)
            {
                throttle = -Math.sqrt(Math.abs(RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX)));
            }
            else
            {
                throttle = Math.sqrt(RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX));
            }
        }
        else
        {
            throttle = 0;
        }

        /* check deadband */
        if (RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) > 0.2
                || RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) < -0.2)
        {
            if (RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) < 0)
            {
                turn = -Math.sqrt(Math.abs(RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY)));
            }
            else
            {
                turn = Math.sqrt(RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY));
            }
        }
        else
        {
            turn = 0;
        }
        arcadeDrive(throttle, -turn);
    }

    public void setHoldPosition()
    {
        zeroSensors();
        m_drivePID.setDesiredValue(0);
        m_turnPID.setDesiredValue(0);
    }
}

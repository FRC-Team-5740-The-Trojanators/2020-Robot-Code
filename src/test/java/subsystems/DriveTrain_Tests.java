package subsystems;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.subsystems.Drivetrain;



public class DriveTrain_Tests 
{
    private WPI_TalonSRX m_leftFrontDriveMotor; // = new WPI_TalonSRX(Constants.leftDriveACAN);
    private WPI_TalonSRX m_rightFrontDriveMotor; // = new WPI_TalonSRX(Constants.rightDriveACAN);
    private WPI_TalonSRX m_leftRearDriveMotor; // = new WPI_TalonSRX(Constants.leftDriveBCAN);
    private WPI_TalonSRX m_rightRearDriveMotor;

    private ADXRS450_Gyro m_gyro;// = new ADXRS450_Gyro();

    private Drivetrain m_dt;

    public void setup()
    {
        //System.out.println("start here");
        
        m_leftFrontDriveMotor = mock(WPI_TalonSRX.class);
        m_rightFrontDriveMotor = mock(WPI_TalonSRX.class);
        m_leftRearDriveMotor = mock(WPI_TalonSRX.class);
        m_rightRearDriveMotor = mock(WPI_TalonSRX.class);

        m_gyro = mock(ADXRS450_Gyro.class);

        m_dt = new Drivetrain(m_leftFrontDriveMotor, m_rightFrontDriveMotor, 
                              m_leftRearDriveMotor, m_rightRearDriveMotor,
                              m_gyro);
    }

    @Test
    public void ThisWillFail()
    {
        setup();

        when(m_leftFrontDriveMotor.getSelectedSensorPosition()).thenReturn(10.0);

        System.out.println(m_leftFrontDriveMotor.getSelectedSensorPosition());
        
       // m_dt.stop();
        assertTrue(true);
    }

    @Test
    public void deadbandArcadeDrive_InsideDeadband_Returns_0()
    {
        // Arrange
        setup();

        var joyStickValue = 0.07;
        var deadBandLowLimit = -0.1;
        var deadbandHighLimit = 0.1;

        // Act
        var actual = m_dt.calculateDeadband(joyStickValue, deadBandLowLimit, deadbandHighLimit);

        // Assert
        assertEquals(0.0, actual);
    }

    @Test
    public void deadbandArcadeDrive_valueLT_Deadband_Returns_NegativeValue()
    {
        // Arrange
        setup();

        var joyStickValue = -0.5;
        var deadBandLowLimit = -0.1;
        var deadbandHighLimit = 0.1;

        // Act
        var actual = m_dt.calculateDeadband(joyStickValue, deadBandLowLimit, deadbandHighLimit);

        // Assert
        assertTrue(actual < 0);
    }

    @Test
    public void deadbandArcadeDrive_valueGT_Deadband_Returns_PositiveValue()
    {
        // Arrange
        setup();

        var joyStickValue = 0.5;
        var deadBandLowLimit = -0.1;
        var deadbandHighLimit = 0.1;

        // Act
        var actual = m_dt.calculateDeadband(joyStickValue, deadBandLowLimit, deadbandHighLimit);

        // Assert
        assertTrue(actual > 0);
    }
    
    
}

package subsystems;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
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
        System.out.println("start here");
        
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
    
}

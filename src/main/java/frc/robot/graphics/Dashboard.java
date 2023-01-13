package frc.robot.graphics;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;

import java.awt.geom.Ellipse2D;

import javax.swing.SwingUtilities;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;


public class Dashboard extends Model{
    private Frame v;
	@Override
	public void drawShape(Graphics2D g, double[] d) {
		int width = 1920;
        int height = 1080;

		
		//Drive Train
        g.setColor(Color.WHITE);
        g.setFont(new Font("Times New Roman", Font.PLAIN, 48));
        g.drawString("Speed", 60, 60);

        g.setColor(Color.GRAY);
        Ellipse2D speedometer = new Ellipse2D.Float(60, 90, 120, 120);
        g.fill(speedometer);

        g.setColor(Color.RED);

        double y = Math.sqrt(60*60 - (d[0] - 120) + 160);

        g.drawLine(120, 160, (int)d[0]*56 + 62, (int)y);
		
	}
	/**
	 * @param args
	 */
	public static void main(String[] args) {
	    
		Dashboard a = new Dashboard();
        Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
        double xAngle = m_mainStick.getRawAxis(0);
        double yAngle = m_mainStick.getRawAxis(1);
    
        double mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
		double[] driveInfo = {mag};
	    //v = new Frame(a, driveInfo);
        //v.setVisible(true);
        while (true) {
            xAngle = m_mainStick.getRawAxis(0);
            yAngle = m_mainStick.getRawAxis(1);
    
            mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
            driveInfo[0] = mag;
            
        }
	    
	    
	
	}

    public void refresh() {
        SwingUtilities.updateComponentTreeUI(v);
        //v.setDriveInfo(driveInfo);
    }
	
}

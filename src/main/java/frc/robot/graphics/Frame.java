package frc.robot.graphics;

import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import javax.swing.JFrame;
import javax.swing.JPanel;
public class Frame extends JFrame {
	
	private Model model;
    private double[] d;
	public Frame(Model m, double[] DriveInfo) {
		model = m;
        this.d = DriveInfo;
		
		JPanel panel = new JPanel() {
			
			public void paint(Graphics g) {
				super.paint(g);
				
				Graphics2D myGraphics = (Graphics2D) g;
				
				Color blue = new Color(33, 37, 92);
				Color blue2 = new Color(57, 66, 164);
				GradientPaint gp = new GradientPaint(0, 0, blue, 400, 700,
						blue2, true);
				
				myGraphics.setPaint(gp);
				
				myGraphics.fillRect(0, 0, 1920, 1080);
				
				model.drawShape(myGraphics, DriveInfo);
			}
		};
		
		setContentPane(panel);
		
		panel.setOpaque(true);
		
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		setSize(1920, 1080);
		
		setTitle("Team 1160 Dashboard");
	}

    public void setDriveInfo(double[] d) {
        this.d = d;
    }
}
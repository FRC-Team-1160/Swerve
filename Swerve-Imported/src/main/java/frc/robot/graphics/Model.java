package frc.robot.graphics;

import java.awt.Graphics2D;
public abstract class Model {
	
	protected float x = 50, y = 100;
	/*This is a default constructor 
	 * a constructor that does not pass any arguments 
	 * this constructor updates x and y to specific values
	 */
	
	
	public Model(){
		this(0, 0);
	}
	
	/*
	 * This is a 2 argument constructor the first argument is float xNew the
	 * second argument is float yNew this constructor updates x and y to values
	 * passed in as arguments
	 */
	public Model(float xNew, float yNew) {
		x = xNew;
		y = yNew;
	}
	public abstract void drawShape(Graphics2D g, double[] d);
}
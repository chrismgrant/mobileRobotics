package edu.cmu.ri.mrpl.control;

import java.util.Date;
/**
 * PID controller for error driving
 * Attach to each error value that needs feedback control
 * @author WangHeli
 *
 */
class PIDController {

	private double p, i, d, dLast, iInt, iCap;
	private Date clockD;
	private long lastClockD;
	private Date clockI;
	private long lastClockI;
	
	/**
	 * Creates a controller using a PID interface
	 * @param proportional proportional constant
	 * @param derivative derivative constant
	 * @param integral integral constant
	 * @param integralCap maximum value of integral
	 */
	PIDController(double proportional, double derivative, double integral, double integralCap){
		p = proportional;
		i = integral;
		d = derivative;
		lastClockD = 0;
		clockD = new Date();
		lastClockI = 0;
		clockI = new Date();
		dLast = 0;
		iInt = 0;
		iCap = integralCap;
	}
	/**
	 * Creates a controller using a PD interface
	 * Integral constant is 0
	 * @param proportional proportional constant
	 * @param derivative derivative constant
	 */
	PIDController(double proportional, double derivative){
		this(proportional, derivative, 0,0);
	}
	/**
	 * Creates a controller using a P interface
	 * Integral, derivative constant is 0
	 * @param proportional proportional constant
	 */
	PIDController(double proportional){
		this(proportional, 0,0,0);
	}
	/**
	 * Uses PID logic to return suitable output for input.
	 * Does not clamp return value, advise to do so outside of PID
	 * @param inputValue value to give PID controller
	 * @return output value determined by PID controller
	 */
	double getOutput(double inputValue){
		return p*inputValue+d*getDerivative(inputValue)+i*getIntegral(inputValue);
	}
	private double getDerivative(double inputValue){
		double dv = inputValue - dLast;
		dLast = inputValue;
		lastClockD = clockD.getTime();
		clockD.setTime(System.currentTimeMillis());
		long dt = clockD.getTime() - lastClockD;
		dt = (dt==0)?1:dt;
		return dv/dt;
	}
	private double getIntegral(double inputValue){
		lastClockI = clockI.getTime();
		clockI.setTime(System.currentTimeMillis());
		long dt = clockI.getTime() - lastClockI;
		iInt += inputValue*dt;
		if (iCap < iInt){
			clearIntegral();
		}
		return iInt;
	}
	/**
	 * Clears the integral term to prevent overshoot
	 */
	void clearIntegral(){
		iInt = 0;
	}
	
}

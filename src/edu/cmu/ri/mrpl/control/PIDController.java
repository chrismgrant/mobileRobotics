package edu.cmu.ri.mrpl.control;

import java.util.Date;

import org.apache.commons.collections.buffer.CircularFifoBuffer;

/**
 * PID controller for error driving
 * Attach to each error value that needs feedback control
 * @author WangHeli
 *
 */
class PIDController {

	private double p, i, d, dLast, iInt;
	private Date clockD;
	private long lastClockD;
	private Date clockI;
	private long lastClockI;
	private CircularFifoBuffer iRing;
	private int ringLimit;
	
	/**
	 * Creates a controller using a PID interface
	 * @param proportional proportional constant
	 * @param derivative derivative constant
	 * @param integral integral constant
	 * @param integralCap buffer size of integral storage. Must be greater than 0.
	 */
	PIDController(double proportional, double derivative, double integral, int integralCap){
		p = proportional;
		i = integral;
		d = derivative;
		lastClockD = 0;
		clockD = new Date();
		lastClockI = 0;
		clockI = new Date();
		dLast = 0;
		iRing = new CircularFifoBuffer(integralCap);
		ringLimit = integralCap;
		clearIntegral();
	}
	/**
	 * Creates a controller using a PD interface
	 * Integral constant is 0
	 * @param proportional proportional constant
	 * @param derivative derivative constant
	 */
	PIDController(double proportional, double derivative){
		this(proportional, derivative, 0,1);
	}
	/**
	 * Creates a controller using a P interface
	 * Integral, derivative constant is 0
	 * @param proportional proportional constant
	 */
	PIDController(double proportional){
		this(proportional, 0,0,1);
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
		dt = (dt == 0)?1:dt;
		return dv/dt;
	}
	private double getIntegral(double inputValue){
		lastClockI = clockI.getTime();
		clockI.setTime(System.currentTimeMillis());
		long dt = clockI.getTime() - lastClockI;
		dt = (dt == 0)?1:dt;
		double iLast = (Double) iRing.get();
		iInt -= iLast;
		iInt += inputValue*dt;
		iRing.add(new Double(iInt));
		return iInt;
	}
	/**
	 * Clears the integral term to prevent overshoot
	 */
	void clearIntegral(){
		iInt = 0;
		for (int j = 0; j < ringLimit; j++){
			iRing.add(0.0);
		}
	}
	
}

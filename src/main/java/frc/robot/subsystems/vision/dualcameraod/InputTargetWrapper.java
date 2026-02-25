package frc.robot.subsystems.vision.dualcameraod;

import edu.wpi.first.units.measure.Time;

public class InputTargetWrapper {
	//periodic runs 20 ms by default ... that helps us!
		//that'd mean we'd need 50 slots per second ... assuming that our object detection is that fast (hint: it never is)
	//after 30 seconds, all our prior input should be deleted
	public InputTargetWrapper(Time deleteInputs){
		//latest scan needs both rough and timestamped info (I suspect that rough will never be a problem, but who knows)
		//from there, only keep the timestamped info (meant for predicting trajectories in the future, though we don't need it now)
		//at one third of our deleteinputs, we only need half of the slots (25 slots per second)
		//at two thirds of our deleteInputs, we only want one scan per second
	}
}

//In reality, I might want to break this all down into two sections
	//first, modify the latesttargets into two sections: rough and timestamped
		//include a working timestamp for the timestamped ones

//second, create a class for the calculated targets, including generated IDs
	//when a target with an ID disappears, wait 1-2 seconds, and if it doesn't show up again, delete it

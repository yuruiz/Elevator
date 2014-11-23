/*
18-649 (Fall 2014)
Group 5:
Vijay Jayaram
James Sakai*
Siyu Wei
Yurui Zhou
*/

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;

/* Author: Vijay Jayaram (vijayj) */
public class HallButtonControl extends Controller {
	
    /***************************************************************************
     * Declarations
     **************************************************************************/
	
    //local physical state
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;

    // network interface
    private WriteableCanMailbox networkHallCall;
    private HallCallCanPayloadTranslator mHallCall;

    //received door closed message
    private Utility.DoorClosedArray mDoorClosed;
    
    //translator for the AtFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkAtFloor;
    private AtFloorCanPayloadTranslator mAtFloor;
    
    //translator for the DesiredFloor message -- this translator is specific
    //to this message and is provided elevatormodules package
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;
    
    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_IDLE,
        STATE_ACTIVE
    }
    
    //state variable initialized to the initial state FLASH_OFF
    private State currentState = State.STATE_IDLE;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public HallButtonControl(int floor, Hallway hallway, Direction direction,SimTime period, boolean verbose) {
        //call to the Controller superclass constructor is required
        super("HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction), verbose);
        
        //stored the constructor arguments in internal state
        this.period = period;
        this.hallway = hallway;
        this.direction = direction;
        this.floor = floor;
        
        /* 
         * The log() method is inherited from the Controller class.  It takes an
         * array of objects which will be converted to strings and concatenated
         * only if the log message is actually written.  
         */
        log("Created " + getReplicationName() + " with period = ", period);

        /* 
         * Create readable payloads and translators for all messages in input interface
         * 		- mAtFloor[f,b,d], mDesiredFloor, mDoorClosed[b,r], HallCall[f,b,d]
         */
       
        // mAtFloor
        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        
        // mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        
        // mDoorClosedLeft, mDoorClosedRight
        mDoorClosed  = new Utility.DoorClosedArray(hallway, canInterface);

        // HallCall
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);

        // Register input messages periodically
        canInterface.registerTimeTriggered(networkAtFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        physicalInterface.registerTimeTriggered(localHallCall);
        
        
        /* 
         * Create writeable payloads for all messages in output interface
         * 		- HallLight[f,b,d], mHallLight[f,b,d], mHallCall[f,b,d]
         */
        
        // HallLight
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);

        // mHallCall
        networkHallCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallCall = new HallCallCanPayloadTranslator(networkHallCall);
        
        // Send/broadcast output messages periodically
        physicalInterface.sendTimeTriggered(localHallLight, period);
        canInterface.sendTimeTriggered(networkHallCall, period);

        // Start the periodic timer
        timer.start(period);
    }

	@Override
	public void timerExpired(Object callbackData) {
		State newState = currentState;
        switch(currentState) {
            case STATE_IDLE:
            	localHallLight.set(false);
            	mHallCall.set(false);
            	//#transition HBC.1
            	if (localHallCall.pressed()) {
            		newState = State.STATE_ACTIVE;
            	}
                break;
            case STATE_ACTIVE:
            	/* Someone made a hall call */
            	localHallLight.set(true);
            	mHallCall.set(true);
            	//#transition HBC.2
            	//XXX: Change: Removed && !localHallCall.pressed() (Should turn off as soon as floor is reached)
            	if (mAtFloor.getValue() && mDesiredFloor.getDirection().equals(direction) && !mDoorClosed
                        .getBothClosed()) {
            		newState = State.STATE_IDLE;
            	}
                break;
            default:
            	throw new RuntimeException("State " + currentState + " was not recognized.");
        }
        
    	log(currentState.toString() + " -> " + newState.toString());
        currentState = newState;

        timer.start(period);
	}
	
	private String getReplicationName() {
		return "HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction);
	}

}

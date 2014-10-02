package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/* Author: Vijay Jayaram (vijayj) */
public class HallButtonControl extends Controller {
	
    /***************************************************************************
     * Declarations
     **************************************************************************/
	
    //local physical state
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;
    
    //network interface
    private WriteableCanMailbox networkHallLightOut;
    // translator for the hall light message -- this is a generic translator
    private HallLightCanPayloadTranslator mHallLight;
    
    // network interface
    private WriteableCanMailbox networkHallCall;
    private HallCallCanPayloadTranslator mHallCall;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedLeft;
    private ReadableCanMailbox networkDoorClosedRight;
    
    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private DoorClosedCanPayloadTranslator mDoorClosedLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedRight;
    
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
        STATE_ACTIVE,
        STATE_SWITCH_OFF
    }
    
    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_IDLE;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public HallButtonControl(SimTime period, int floor, Hallway hallway, Direction direction, boolean verbose) {
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
        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedLeft, hallway, Side.LEFT);
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(networkDoorClosedRight, hallway, Side.RIGHT);

        // HallCall
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);

        // Register input messages periodically
        canInterface.registerTimeTriggered(networkAtFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        canInterface.registerTimeTriggered(networkDoorClosedRight);
        physicalInterface.registerTimeTriggered(localHallCall);
        
        
        /* 
         * Create writeable payloads for all messages in output interface
         * 		- HallLight[f,b,d], mHallLight[f,b,d], mHallCall[f,b,d]
         */
        
        // HallLight
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
        
        // mHallLight
        networkHallLightOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallLight = new HallLightCanPayloadTranslator(networkHallLightOut, floor, hallway, direction);
        
        // mHallCall
        networkHallCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallCall = new HallCallCanPayloadTranslator(networkHallCall, floor, hallway, direction);
        
        /* Send/broadcast output messages periodically */
        physicalInterface.sendTimeTriggered(localHallLight, period);
        canInterface.sendTimeTriggered(networkHallLightOut, period);
        canInterface.sendTimeTriggered(networkHallCall, period);

        // Start the periodic timer
        timer.start(period);
    }

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		log("Entering " + state);
        switch(state) {
            case STATE_IDLE:
            	localHallLight.set(false);
            	mHallLight.set(false);
            	mHallCall.set(false);
            	//#transition HBC.T.1
            	if (localHallCall.pressed()) {
            		newState = State.STATE_ACTIVE;
            	}
                break;
                
                /* Design issue: 
                 * 	- In STATE_ACTIVE, transition back to STATE_IDLE
                 *  - localHallCall will assume the value of mHallCall, which is true
                 *  - It will be pressed regardless
                 *  - Need to add an off state that will always transition back to IDLE
                 */
            case STATE_ACTIVE:
            	localHallLight.set(true);
            	mHallLight.set(true);
            	mHallCall.set(true);
            	//#transition HBC.T.2
            	//XXX: Change: Removed && !localHallCall.pressed() (Should turn off as soon as floor is reached)
            	if (mAtFloor.getValue() && mDesiredFloor.getFloor() == floor &&
            			mDesiredFloor.getDirection().equals(direction)) {
            		newState = State.STATE_SWITCH_OFF;
            	}
                break;
            case STATE_SWITCH_OFF:
            	/* XXX: Addition:
            	 *  Temporary state to reset all output messages for one period before re-entering idle */
            	localHallLight.set(false);
            	mHallLight.set(false);
            	mHallCall.set(false);
            	//#transition HBC.T.3
            	if (true) {
            		newState = State.STATE_IDLE;
            	}
            	break;
            default:
            	throw new RuntimeException("State " + state + " was not recognized.");
        }
    	log(state.toString() + " -> " + newState.toString());
        state = newState;

        timer.start(period);
	}
	
	private String getReplicationName() {
		return "HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction);
	}

}

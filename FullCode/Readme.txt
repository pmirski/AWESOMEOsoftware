
1. COPY CODE and make symmetrical (new folder)				
	PHYSICAL CHANGE: lower & connect other QRD block
	CODE CHANGE: lots. add that other pin sensor
2. Make changes for the one sing hor trolley (only one position to go to: pulls agent up, exits loop soon if dun register switch, goes to crane move, claw opens, go to next
	PHYSICAL CHANGE: 1. take off last stripe
			 2. maybe Cam's pancake stack increase? <--CONCERN: NO TUB CLEARANCE
	CODE CHANGE: 1. make exit ClBlk loop after some time (pancake stack may suffice)
		     2. ma

Also: w 16.7, too far on IRgate approach (and only shorten IR approach there)
Also: BRING HEAT/GLUE GUN FOR ADDING PANCAKE STACK (or...velcro? sth ready, off/on
Also: 111 on 16.4, 105 on 16.7
Also: ZL full turn works well with 16.7

CHARGE ALL BATTERIES ON OTHER PPLS RIGHT WHEN ENTER

CAM SAYS: "Agent at position #2 is at LO , robot tried to grab at either Hi or Medium"



Pawel: TESTING IS #1
IMPLEMENT ERROR HANDLING AFTER TESTING IS SUCCESSFUL
	Tuesday morning: error handling considerations + general cleanup
1. Testing:
	a) Grabbing at table
	b) Grabbing at tub
2. Change so that tapefollow a little longer, then crane turns
3. Error handling!!! Consider all possibilities
4. Test mirrored surface
5. Test transition from retrieval to zipline
6. Move edge sensors <---test if can
7. Change times: wait for Kurt's voltage regulation


Wednesday PROBLEMS: 
	1) Didn't read IR signal from some distance; not sure if b/c batteries were low 
	(Voltages of IR batteries 1-3 runs after noticed this: 7.79, 7.36)
	2) After Jake's merge, both go up ramp (goes too far, despite no voltage difference) and appraoch circle (just kept tapefollowing)
	3) ATTEMPTED: CBlk overshot going down; just lower the speed cuz that should be ok
	4) Trolley: If wanna go to trolley and register says =0 even before function is called, ensure that the switch must still be hit to get at 0.




Thu morning THINGS TO ADDRESS:

1) Continue with yesterday's testing, moving on to leaving the circle and ziplining, and eventually doing full run, start to finish.
	
2) Test code for other surface (just select R for other surface when upload FullCode.ino and start up.robot (asks u to select))
(R = go Right after IR gate
L = go Left after IR gate)
	
3) Test IR capabilities *with fully-charged IR batteries*
	
4) Ensure can both, in series, go up ramp and enter circle w most recent code. (Can compare with code pushed yesterday labeled something like "before merge w Jake's code")
	
5) Attempt water retrieval
	
6) Fix cable routing in front that has been effed with
	
7) Add maybe-seran-wrappy "splash guard" onto back of basket re: bunch of wet agents tumbling in, near circuitry&wires
	
8) Detecting IR gate arrival would be greaaaaaaaat
	
9) Ensure we can quickly adjust jib height

10) We must be able to ensure height clearance without being on surface (i.e. on test day)

12) Consider only haveing one white tape piece for hor translation

// Nota bene:
//    0. All code in one file because remains ready for one-big-case conversion
//    1. RESET AFTER EVERY TEST; otherwise variable values do not reset. Many initial values important.
//    2. Trolley (at end (BUT SWITCH CANNOT BE DEPRESSED OR WILL NEVER MOVE), ClawBlock (at jib), and Claw (open) position values are initially 0. 
//    3. Global variable motorspeed changes. But initialized just below it is reset.
//    4. currently, must have clawblock in lowest position, and TROL AT END WITHOUT SWITCH DPRESSED (here it matters)


// To do for one person (i.e. Pawel):
//    -12. OPEN ANGLE IS CURRENTLY NOT TOTALLY OPEN; CHANGE FOR WATER RETRIEVAL
//    -11. ENSURE NOT SPEEDING UP JUST BEFORE RAMP ENTER (RE: WALLFUCK)
//    -10. At/approaching IR gate: Consider sensing earlier to maybe save time
//     -9. Ensure that this gets you to Line1, from where you wanna take off
//     -8. If claw is mashing agents' heads during retrieval, implement Jakes come-in-2-pick-up idea
//     -7. Ensure all code symmetrical re: platforms
//     -6. Reset at beginning: make it go to end, and all way up (i.e. have motor run thusly, briefly). Also, reset in between each retrieval, in case shit is fucked up.
//     -5. Cut away wood where tape may hit
//     -4. Determine whether need to send constant signal to servos so they dun sway while driving 
//     -3. IF THEY'RE NEEDED, make delays into constants
//     -2. Make IR gate code:
		-Wait for low to high transition, and ensure get high signal for e.g. 50ms and if any low in there, interrupt
//     -1. DO PWM FOR SLOWER SERVO TURNS <---ON WEEKEND
//	0. GO THRU CODE AND READ/DO ALL TO-DO'S IN COMMENTS
//	1. Determine exact servo angle values for: all 2or3 crane angles, all 2or3 claw angles
//	4. Determine HOR and VERT motor speeds sufficient to get QRD well into white tape territory; transition zones results in poor hi/lo voltage readings.
// 	5. Concern about HOR and VERT going off white tape while other functions perform:
		-MAYBE: Have HOR or VER continually floating between black and white and code 
		-MAYBE: Make white spaces wide if can
			JAKE'S IDEA: LIMITER/RESET SWITCHES.
			Do those need to be interrupts?

// To do as team:
//	1. Determine protocol for getting agents (e.g. Repeat condition? (i.e. 2nd search if unsuccessful first time? Currently single attempt))


// To do but LessImportant: 
//	A. Ensure have correct function descriptions
//	B. take away initialization values for vars with comment "Initialization value completely arbitrary"


// Things just for testing (remove later?): 
//	1. Existing delays in functions called
//	2. Displays in those functions


// Potential future changes:
//	1. I commented out or delete any claw sensor code (so can proceed without it). Can uncomment / re-integrate.
		(so this is not useful right now: 
		"Determine angle of having an agent re: Snsr_Claw_CnvrsnRatio, Snsr_Claw_CnvrsnRatio, all three open/haveagent/closed variables"
		)
//	2. Go2ZL: 
		i) just straight out at Line2
		ii) QRD sensors
		iii) edge tracking

Concerns:
//	1. Gripping agent: may hit rim
		MAYBE: 	Clamp downward
	

*********************************CONCERN SANDBOX*******************************************
-DC motor speeds: Can have Well over 125 going up claw block motor speed. Try 170?

-If up speed, enlarge white space, make limiters, etc.

Added Thu: 
Right now (1pm):		note: ensure water retrieval l8r
	1. TR: enter/exit argument
		Agent2: MaxExtensn
		Agent3: MaxExtensn	
		Agent4: MaxExtensn	
		Agent5: MaxExtensn  QUESTIONABLE
		Agent6: MaxExtensn  QUESTIONABLE
	2. Claw max open
	3. Crane tub angle
	4. Claw open (not max)
		If wet: claw open MAX
	5. CB goes to input argument (re: agent height)
		If wet: CB goes to low agent
	5.5 TR over dry agent
	6. Claw close
	7. CB to jib
	7.5 TR: enter/exit argument
	8. Crane over bot
	9. TR over basket
	10. Claw open


grabbed platform
// Nota bene:
//    0. All code in one file because remains ready for one-big-case conversion
//    1. RESET AFTER EVERY TEST; Variable values do not reset. Many initial values important.
		e.g.	Postn_Trol_Register & Postn_ClBlk_Register must initially be 0.
//    2. Trolley (at tower), ClawBlock (at jib), and Claw (open) position values are initially 0. 
//    3. Global variable motorspeed changes. But initialized just below it is reset.


// To do for one person (i.e. Pawel):
//	0. GO THRU CODE AND READ/DO ALL TO-DO'S IN COMMENTS
//	1. Determine exact servo angle values for: all 2or3 crane angles, all 2or3 claw angles
//	2. Determine length of time to wait for servo to complete action (i.e. close/pivot)
//	3. Determine lengths of all delays, ensure ones that are testing only are marked so that excised for final robot
//	4. Determine HOR and VERT motor speeds sufficient to get QRD well into white tape territory; transition zones results in poor hi/lo voltage readings.
// 	5. Concern about HOR and VERT going off white tape while other functions perform:
		-MAYBE: Have HOR or VER continually floating between black and white and code 
		-MAYBE: Make white spaces wide if can

// To do as team:
//	1. Determine protocol for getting agents (e.g. Repeat condition? (i.e. 2nd search if unsuccessful first time? Currently single attempt))


// To do but LessImportant: 
//	A. Ensure have correct function descriptions
//	B. take away initialization values for vars with comment "Initialization value completely arbitrary"
//	C. Try changing "postn" to "pos" at end


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
	

make cut lines: Q3
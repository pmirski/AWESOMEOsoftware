// Nota bene:
//    1. Variable values do not reset during testing, unless you reset them.
//    2. Trolley (at tower), ClawBlock (at jib), and Claw (open) position values are initially 0. *Ideally, you can choose these initial values from menu.*
//    3. Slow downs added at top of ramp, where glboal variable value changes (never want old speed again)

// To do:

After testing: 
	Turn Postn_Trol_Register & Postn_ClBlk_Register initialization values back to 0.

//   -3. YOU MUST ENSURE THAT THE HOIZONTAL&VERTICAL TRANSLAITON SPEEDS ARE SUFFICIENT TO GET QRD WELL INTO WHITE TAPE TERRITORY; transition zones results in poor hi/lo voltage readings.
//   -2. Note that you're starting from 2nd line for sure. Nothing for special 1st line retrieval done (and will not be, unless group agrees).
//   -1. Complete Robot State variables, and start utilizing once integrate DRIVEfunction code block
//    0. Determine protocol for getting agents
//    1. Determine angle values for: all 3 crane angles, all 3 claw angles
//    2. Determine angle of having an agent re: Snsr_Claw_CnvrsnRatio, Snsr_Claw_CnvrsnRatio, all three open/haveagent/closed variables
//    3. Determine taking out delays from
//    4. Determine length of time to wait for servo to close
//    5. LessImportant: ensure have correct function descriptions
//    6. LessImportant: take away initialization values for vars with comment "Initialization value completely arbitrary"
//    7. Less important: Try changing "postn" to "pos" at end
//    8. Less important: Don't initialize variables whose initial value doesn't matter


// Not yet addded:
//    1. 2nd search if unsuccessful first time; just single attempt right now
//    2. Wet retrievals (but setClawBlockVerticalPosition is ready for it)

// Things just for testing (remove later?): 
//    1. Existing delays in functions called
//    2. Displays in those functions

make cut lines: Q3
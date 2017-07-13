// Nota bene:
//    1. Variable values do not reset during testing, unless you reset them.
//    2. Trolley (at tower), ClawBlock (at jib), and Claw (open) position values are initially 0. *Ideally, you can choose these initial values from menu.*

// To do:
//    1. Determine angle values for: all 3 crane angles, all 3 claw angles
//    2. Determine angle of having an agent re: Snsr_Claw_CnvrsnRatio, Snsr_Claw_CnvrsnRatio, all three open/haveagent/closed variables
//    3. Determine taking out delays from
//    4. Determine length of time to wait for servo to close
//    5. LessImportant: ensure have correct function descriptions
//    6. LessImportant: take away initialization values for vars with comment "Initialization value completely arbitrary"
//    7. Try changing "postn" to "pos" at end

// Not yet addded:
//    1. 2nd search if unsuccessful first time; just single attempt right now
//    2. Wet retrievals (but setClawBlockVerticalPosition is ready for it)

// Things just for testing (remove later?): 
//    1. Existing delays in functions called
//    2. Displays in those functions
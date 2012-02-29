KeysDictionary("galilactor", (1,1), 
# version...

# parameters
    Key("DesOrient",
        Float(invalid="nan", help = "Piston", units = "um"),
        Float(invalid="nan", help = "Tilt X", units = "arcseconds"),
        Float(invalid="nan", help = "Tilt Y", units = "arcseconds"),
        Float(invalid="nan", help = "Translation X", units = "um"),
        Float(invalid="nan", help = "Translation Y", units = "um"),
        Float(invalid="nan", help = "Rotation about Z", units = "arcseconds"),
        help = "The desired mirror orientation."
    ),
    Key("DesOrientAge", 
        Float(invalid = "nan", units = "seconds"),
        help = "Elapsed time since a desired orientation was set."
    ),    
    Key("CmdMount",
        Float(invalid = "nan")*(nAct,),
        help = "Commanded mount positions."
    ),
    Key("Cmd", String(invalid = "?"), help = "The current command (verb) executing."),
    Key("MaxDuration",
        Float(invalid = "nan", units = "seconds"),
        help = "Maximum amount of time expected for current process."
    ),
    Key("Duration",
        Float(invalid = "nan", units = "seconds"),
        help = "Time spent executing so far."
    ),
    Key("Text", String(), help = "Text for humans to read"),
    Key("Orient",
        Float(invalid = "nan", help = "Piston", units = "um"),
        Float(invalid = "nan", help = "Tilt X", units = "arcseconds"),
        Float(invalid = "nan", help = "Tilt Y", units = "arcseconds"),
        Float(invalid = "nan", help = "Translation X", units = "um"),
        Float(invalid = "nan", help = "Translation Y", units = "um"),
        Float(invalid = "nan", help = "Rotation about Z", units = "arcseconds"),
        help = "The measured mirror orientation."
    ),
    Key("ActMount", Float(invalid = "nan"), help = "The measured actuator mount positions")*(nAct,),
    Key("Iter", Int(invalid = "nan"), help = "Current move iteration."),
    Key("AxisHomed", Bool(0, 1, invalid = "?"), help = "Axes that are homed.")*(nAct,),
    Key("Status", Bits("Stop Code:8", 
                        "not used",
                        "Home switch activated",
                        "Reverse limit switch activated",
                        "Forward limit switch activated",
                        "undefined",
                        "Motor on",
                        "not used",
                        "Axis in motion",
                        "
                        )
    )*(nAct,]),
    Key("MaxIter" = int
Homing = int[nAct]
UnparsedReply = str[, str]
BadGalilReply = str, str
GParSoftwareVersion = int
GParNAXES = int
GParDOAUX = int
GParMOFF = int
GParNCORR = int
GParWTIME = Float
GParENCTIME = Float
GParLSTIME = Float
GPar_RNGxdiv2 = int[nAct]
# these look equal but opposite, get rid of one?
GParRNGxdiv2 = int[nAct]
GParSPDx = int[nAct]
GParHMSPDx = int[nAct]
GParACCx = int[nAct]
GParMINCORRx = int[nAct]
GParMAXCORRx = int[nAct]
GParST_FSx = int[nAct]
GParMARGx = int[nAct]
GParINDSEP = int[nAct]
GParENCRESx = Float[nAct]

)
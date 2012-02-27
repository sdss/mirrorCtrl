KeysDictionary("galil", (?,?), 
# version...

# parameters
    Key("DesOrient",
        float(help = "Piston", units = "um"),
        float(help = "Tilt X", units = "arcseconds"),
        float(help = "Tilt Y", units = "arcseconds"),
        float(help = "Translation X", units = "um"),
        float(help = "Translation Y", units = "um"),
        float(help = "Rotation about Z", units = "arcseconds"),
        help = "The desired mirror orientation"
    ),
    Key("DesOrientAge", 
        float(invalid = "nan", units = "seconds"),
        help = "Elapsed time since a desired orientation was set"
    ),
    
CmdMount = float[nAct]
Cmd = str
MaxDuration = float
Duration = float
Text = str
Orient = float[6]
ActMount = float[nAct]
Iter = int
AxisHomed = int[nAct]
Status = int[nAct]
MaxIter = int
Homing = int[nAct]
UnparsedReply = str[, str]
BadGalilReply = str, str
GParSoftwareVersion = int
GParNAXES = int
GParDOAUX = int
GParMOFF = int
GParNCORR = int
GParWTIME = float
GParENCTIME = float
GParLSTIME = float
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
GParENCRESx = float[nAct]

)
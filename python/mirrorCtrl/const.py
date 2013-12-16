"""Collection of constants and functions used throughout the package
"""
import math
import numpy

## millimeters per micron
MMPerMicron = 1 / 1000.0      
## millimeters per inch
MMPerInch = 25.4  
## radians per degree
RadPerDeg  = math.pi / 180.0    
## arcseconds per degree
ArcSecPerDeg = 60.0 * 60.0   
## radians per arcsec 
RadPerArcSec = RadPerDeg / ArcSecPerDeg 

_ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                                        MMPerMicron, MMPerMicron, RadPerArcSec], dtype = float)

def convOrient2UMArcsec(orientation):
    """Convert an orientation from mm and radians to um and arcsec
    @param[in] orientation: list/array type of 1 to 6 elements
    @return orientation, of same length as input orient, with units in um and arcsec
    """
    return numpy.asarray(orientation)/_ConvertOrient[:len(orientation)]

def convOrient2MMRad(orientation):
    """Convert an orientation from um and arcsec to mm and radians
    @param[in] orientation: list/array type of 1 to 6 elements
    @return orientation, of same length as input orient, with units in mm and radians
    """
    return numpy.asarray(orientation)*_ConvertOrient[:len(orientation)]

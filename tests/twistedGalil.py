"""
This runs on someother computer.  It pings faked Galil replies back for testing
"""


# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.


from twisted.internet import reactor, protocol


class Echo(protocol.Protocol):
    """This is just about the simplest possible protocol"""
    
    def dataReceived(self, data):
        "As soon as any data is received, look at it and write something back." 
        print 'data recieved'
        replyStr = genStrFromData(data)
        self.transport.write(replyStr)


def main():
    """This runs the protocol on port 8000"""
    factory = protocol.ServerFactory()
    factory.protocol = Echo
    reactor.listenTCP(8000,factory)
    reactor.run()

def genStrFromData(data):

    str = 'ok' # ok is recognized as end of command in galil code
    return str

# this only runs if the module was *not* imported
if __name__ == '__main__':
    main()
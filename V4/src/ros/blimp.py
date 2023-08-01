
class Blimp:
    def __init__(self, ID, name):
        self.IP = 1
        self.ID = ID
        self.name = name
        self.connected = False

        # ROS2 published values
        self.auto = False
        self.killed = False
        self.motorCommands = [0, 0, 0, 0]
        self.grabbing = False
        self.shooting = False
        self.baseBarometer = 123456.000001

        # ROS2 backend values
        self.nodeHandler = None

        # Misc values
        self.lastHeartbeatDetected = 0

        self.targetGoal = "Y" #"Y","O"
        self.targetEnemy = "B" #"R","G","B"

        self.receivedAuto = "Null"
        self.receivedStatus = "Null"

        #self.receivedAuto_Surface = getTextSurface(self.receivedAuto, 25)
        #self.receivedStatus_Surface = getTextSurface(self.receivedStatus, 25)

        self.surfaces = {}

        #self.nameSurface = getTextSurface(self.name, int(40 - len(self.name)))
        self.lastHeartbeatDiff = 0
        self.heartbeatDisconnectDelay = 5  # seconds
        self.lastTimeInputDataSent = 0
        self.timeInputDelay = 0.05  # seconds
        self.lastBarometerSentTime = 0
        self.barometerSendDelay = 1/20  # seconds
        self.lastTargetGoalSentTime = 0
        self.targetGoalSendDelay = 1  # seconds

        self.data = []
        self.receivedState = -1
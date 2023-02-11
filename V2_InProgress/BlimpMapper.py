class BlimpMapper:
    # ============================== INIT ==============================
    def __init__(self, blimpHandler, inputHandler):
        self.blimpHandler = blimpHandler
        self.inputHandler = inputHandler

        self.inputToBlimpMap = {}
        self.blimpToInputMap = {}
        self.inputIndexMap = {}
        self.blimpIndexMap = {}

        self.lastNumBlimps = 0
        self.lastNumInputs = 0

    def update(self):
        self.checkBadMappings()

    # Enforces valid mapping state
    def checkBadMappings(self):
        # Check for removed inputs
        if len(self.inputHandler.inputs) < self.lastNumInputs:
            # Uh oh, an input has been removed...
            # Make set of valid inputs
            validInputs = set()
            for input in self.inputHandler.inputs:
                validInputs.add(input.name)
            # Iterate through mappings, remove invalid inputs
            possibleInputs = list(self.inputToBlimpMap.keys())
            for possibleInput in possibleInputs:
                if not possibleInput in validInputs:
                    # Remove mapping!
                    invalidBlimp = self.inputToBlimpMap.pop(possibleInput)
                    self.blimpToInputMap.pop(invalidBlimp)
        self.lastNumInputs = len(self.inputHandler.inputs)
        # Check for removed blimps
        if len(self.blimpHandler.blimps) < self.lastNumBlimps:
            # Uh oh, a blimp has been removed...
            # Make set of valid blimps
            validBlimps = set()
            for blimp in self.blimpHandler.blimps:
                validBlimps.add(blimp.name)
            # Iterate through mappings, remove invalid blimps
            possibleBlimps = list(self.blimpToInputMap.keys())
            for possibleBlimp in possibleBlimps:
                if not possibleBlimp in validBlimps:
                    # Remove mapping!
                    invalidInput = self.blimpToInputMap.pop(possibleBlimp)
                    self.inputToBlimpMap.pop(invalidInput)
        self.lastNumBlimps = len(self.blimpHandler.blimps)

    def updateMapping(self, inputName, blimpName):
        if inputName in self.inputToBlimpMap and blimpName in self.blimpToInputMap and self.inputToBlimpMap[inputName] == blimpName:
            # Mapping exists... remove it!
            self.inputToBlimpMap.pop(inputName)
            self.blimpToInputMap.pop(blimpName)
        else:
            # Mapping does not exist... create it!
            if inputName in self.inputToBlimpMap:
                self.inputToBlimpMap.pop(inputName)
            if blimpName in self.blimpToInputMap:
                self.blimpToInputMap.pop(blimpName)
            self.inputToBlimpMap[inputName] = blimpName
            self.blimpToInputMap[blimpName] = inputName

    def updateIndices(self):
        self.inputIndexMap = {}
        for i in range(0,len(self.inputHandler.inputs)):
            self.inputIndexMap[self.inputHandler.inputs[i].name] = i
        self.blimpIndexMap = {}
        for i in range(0,len(self.blimpHandler.blimps)):
            self.blimpIndexMap[self.blimpHandler.blimps[i].name] = i

    # Assumes valid mapping state (see checkBadMapping)
    def getMappedBlimp(self, inputName):
        if inputName in self.inputToBlimpMap:
            blimpName = self.inputToBlimpMap[inputName]
            blimpIndex = self.blimpHandler.getBlimpIndex(blimpName)
            if blimpIndex != -1:
                blimp = self.blimpHandler.blimps[blimpIndex]
                return blimp
        return None

    # Assumes valid mapping state (see checkBadMapping)
    def getMappedInput(self, blimpName):
        if blimpName in self.blimpToInputMap:
            inputName = self.blimpToInputMap[blimpName]
            inputIndex = self.inputHandler.getInputIndex(inputName)
            if inputIndex != -1:
                input = self.inputHandler.inputs[inputIndex]
                return input
        return None

    # Assumes valid mapping state (see checkBadMapping)
    def mapUp(self, inputName):
        numBlimps = len(self.blimpHandler.blimps)
        # By default, start looking for new mappings from the end of the list, iterating upwards
        prevIndex = numBlimps
        # Check if there is an existing mapping to start from
        if inputName in self.inputToBlimpMap:
            # Mapping exists
            blimpName = self.inputToBlimpMap[inputName]
            self.updateMapping(inputName, blimpName)
            prevIndex = self.blimpHandler.getBlimpIndex(blimpName)
        validMappingFound = False
        # Iterate through blimps to find an unmapped blimp to map to
        while prevIndex > 0:
            nextIndex = prevIndex - 1
            nextBlimpName = self.blimpHandler.blimps[nextIndex].name
            if nextBlimpName in self.blimpToInputMap:
                # Mapping exists... move on
                prevIndex = nextIndex
            else:
                # Mapping doesn't exist... create it!
                self.updateMapping(inputName, nextBlimpName)
                validMappingFound = True
                break
        if not validMappingFound:
            # print("No valid blimps to map to.")
            pass

    def mapDown(self, inputName):
        numBlimps = len(self.blimpHandler.blimps)
        # By default, start looking for new mappings from the beginning of the list, iterating downwards
        prevIndex = -1
        # Check if there is an existing mapping to start from
        if inputName in self.inputToBlimpMap:
            # Mapping exists
            blimpName = self.inputToBlimpMap[inputName]
            self.updateMapping(inputName, blimpName)
            prevIndex = self.blimpHandler.getBlimpIndex(blimpName)
        validMappingFound = False
        # Iterate through blimps to find an unmapped blimp to map to
        while prevIndex < numBlimps-1:
            nextIndex = prevIndex + 1
            nextBlimpName = self.blimpHandler.blimps[nextIndex].name
            if nextBlimpName in self.blimpToInputMap:
                # Mapping exists... move on
                prevIndex = nextIndex
            else:
                # Mapping doesn't exist... create it!
                self.updateMapping(inputName, nextBlimpName)
                validMappingFound = True
                break
        if not validMappingFound:
            # print("No valid blimps to map to.")
            pass

    def clearMappings(self):
        self.inputToBlimpMap.clear()
        self.blimpToInputMap.clear()

    #Return type is list of (inputIndex, blimpIndex)
    def getMappings(self):
        inputNames = list(self.inputToBlimpMap.keys())
        mappings = []
        for inputName in inputNames:
            inputIndex = self.inputHandler.getInputIndex(inputName)
            if inputIndex != -1:
                blimpName = self.inputToBlimpMap[inputName]
                blimpIndex = self.blimpHandler.getBlimpIndex(blimpName)
                if blimpIndex != -1:
                    mapping = (inputIndex, blimpIndex)
                    mappings.append(mapping)
        return mappings

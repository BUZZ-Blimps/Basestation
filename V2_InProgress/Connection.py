class Connection:
    def __init__(self, inputName, blimpName, blimpHandler, inputHandler):
        self.inputName = inputName
        self.blimpName = blimpName
        self.blimpHandler = blimpHandler
        self.inputHandler = inputHandler

        self.lastInputIndex = -1
        self.lastBlimpIndex = -1

    def updateIndices(self):
        # Check input
        lastInput = self.inputHandler.inputs[self.lastInputIndex]
        if lastInput.name is not self.inputName
            pass
class Connection:
    def __init__(self, inputName, blimpName, inputIndex, blimpIndex):
        self.inputName = inputName
        self.blimpName = blimpName
        self.inputIndex = inputIndex
        self.blimpIndex = blimpIndex
        self.names = (inputName,blimpName)

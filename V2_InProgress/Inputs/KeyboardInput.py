from Input import Input

class KeyboardInput(Input):
    def __init__(self, name, data):
        self.name = name
        self.keys = data
        super().__init__("Keyboard", name, data)
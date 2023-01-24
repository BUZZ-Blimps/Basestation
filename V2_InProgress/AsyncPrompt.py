import pyautogui
from threading import Thread

class AsyncPrompt:
    def __init__(self):
        self.results = []
        self.threads = []

    def prompt(self, text, title, data):
        thread = Thread(target=self.ask, args=(text,title,data))
        self.threads.append(thread)
        thread.start()

    def ask(self, text, title,data):
        answer = pyautogui.prompt(text=text,title=title)
        result = (data,answer)
        self.results.append(result)

    def getResults(self):
        return self.results

    def close(self):
        for thread in self.threads:
            if (thread.is_alive()):
                thread.join()
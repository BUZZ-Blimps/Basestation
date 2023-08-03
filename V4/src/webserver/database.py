import os

# Allowed User
# Steve
# ip_allowed = '10.138.177.116'
# Laptop 3
ip_allowed = '192.168.0.203'
# Personal Laptop
# ip_allowed = '192.168.0.136'

# Database file
database_file = '../src/database/database.txt'
# Current Blimps
blimps_file = '../src/database/blimps.txt'

class Database:
    def __init__(self):
        self.clear_file(blimps_file)
        self.getData()
        self.saveData()
        self.goal_color = "yellow"
        self.target1_color = "green"
        self.target2_color = "green"
        self.client_ip = ip_allowed
        self.blimps = {}

    def getData(self):
        # Read the color from the database.txt file on startup
        if os.path.exists(database_file):
            with open(database_file, 'r') as file:
                data = file.read().strip().split('\n')
                self.goal_color = data[0].split(' = ')[1]
                self.target1_color = data[1].split(' = ')[1]
                self.target2_color = data[2].split(' = ')[1]

    def setClientIP(self, client_ip):
        self.client_ip = client_ip

    def toggle_goal_color(self):
        if self.client_ip == ip_allowed:
            if self.goal_color == 'yellow':
                self.goal_color = 'orange'
            else:
                self.goal_color = 'yellow'
            self.saveData()

    def toggle_target1_color(self):
        if self.client_ip == ip_allowed:
            if self.target1_color == 'blue':
                self.target1_color = 'red'
            elif self.target1_color == 'red':
                self.target1_color = 'green'
            elif self.target1_color == 'green':
                self.target1_color = 'purple'
            else:
                self.target1_color = 'blue'
            self.saveData()

    def toggle_target2_color(self):
        if self.client_ip == ip_allowed:
            if self.target2_color == 'blue':
                self.target2_color = 'red'
            elif self.target2_color == 'red':
                self.target2_color = 'green'
            elif self.target2_color == 'green':
                self.target2_color = 'purple'
            else:
                self.target2_color = 'blue'
            self.saveData()

    def add_blimp(self, blimp):
        with open(blimps_file, 'a') as f:
            f.write(f"Blimp ID = {blimp.ID}, Blimp Name = {blimp.name}\n")

    # Fix
    def saveData(self):
        # Write the colors to the database.txt file
        with open(database_file, 'w') as file:
            file.write(f'Goal = {self.goal_color}\nTarget 1 = {self.target1_color}\nTarget 2 = {self.target2_color}')

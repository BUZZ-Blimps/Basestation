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
        self.goal_color = "yellow"
        self.target1_color = "green"
        self.target2_color = "green"
        self.client_ip = ip_allowed
        self.blimps = {}
        self.getData()

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
    
    def remove_blimp(self, blimp):
        # Load all lines from the file.
        with open(blimps_file, 'r') as f:
            lines = f.readlines()

        # Filter out the line that matches the blimp to be removed.
        lines = [line for line in lines if line.strip() != f"Blimp ID = {blimp.ID}, Blimp Name = {blimp.name}"]

        # Write the remaining lines back to the file.
        with open(blimps_file, 'w') as f:
            f.writelines(lines)

    def get_blimp_names(self):
        # Initialize an empty dictionary to store the blimp data.
        blimps = {}

        # Open the file and read the lines.
        with open(blimps_file, 'r') as f:
            lines = f.readlines()

        # For each line in the file, parse the blimp ID and name and store them in the dictionary.
        for line in lines:
            line = line.strip()  # Remove any leading/trailing whitespace.
            parts = line.split(",")  # Split the line into parts based on the comma.

            # Extract the blimp ID and name from the parts.
            blimp_id_part, blimp_name_part = parts
            blimp_id = int(blimp_id_part.split("=")[1].strip())  # Extract the blimp ID and convert to an integer.
            blimp_name = blimp_name_part.split("=")[1].strip()  # Extract the blimp name.

            # Store the blimp name in the dictionary, using the blimp ID as the key.
            blimps[blimp_id] = blimp_name

        # Get the blimp IDs (keys of the dictionary) and sort them.
        sorted_blimp_ids = sorted(blimps.keys())

        # Get the blimp names corresponding to the sorted blimp IDs and return them.
        sorted_blimp_names = [blimps[blimp_id] for blimp_id in sorted_blimp_ids]
        return sorted_blimp_names

    def saveData(self):
        # Write the colors to the database.txt file
        with open(database_file, 'w') as file:
            file.write(f'Goal = {self.goal_color}\nTarget 1 = {self.target1_color}\nTarget 2 = {self.target2_color}')

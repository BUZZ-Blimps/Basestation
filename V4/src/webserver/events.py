from webserver.packages import *

blimp_files = '../src/ros/blimps'

def send_blimp_names():
    with app.app_context():  # Push an application context
        while True:
            blimpNames = []

            blimp_dir = os.listdir('../src/ros/blimps')

            for blimp_file in blimp_dir:
                if blimp_file.endswith('.txt'):  # If you only want to process text files
                    file_path = os.path.join('../src/ros/blimps', blimp_file)
                    connected = read_value(file_path, 'Connected')
                    if connected == 'True':
                        name = read_value(file_path, 'Basestation Name')
                        if name:
                            blimpNames.append(name)

            socketio.emit('updateBlimpNames', blimpNames, namespace='/')
            time.sleep(0.001)

@socketio.on('connect')
def handle_connect():
    socketio.start_background_task(send_blimp_names)
    db.getData()
    goal_colors = read_value_in_all_files(blimp_files, 'Goal')
    print(goal_colors)
    emit('setGoalColor', goal_colors)
    emit('setTarget1Color', db.target1_color)
    emit('setTarget2Color', db.target2_color)

"""
@socketio.on('toggleGoalColor')
def handle_toggle_goal_color():
    db.setClientIP(request.remote_addr)
    db.toggle_goal_color()
    write_value('../src/ros/blimps/BurnCreamBlimp.txt', 'Goal', db.goal_color)
    emit('setGoalColor', db.goal_color, broadcast=True)
"""

@socketio.on('toggleGoalColor')
def handle_toggle_goal_color(index):
    db.setClientIP(request.remote_addr)

    goal_colors = []  # List to hold the goal colors for each connected blimp

    for blimp_file in blimp_files:
        if blimp_file.endswith('.txt'):  # If you only want to process text files
            file_path = os.path.join('../src/ros/blimps', blimp_file)
            connected = read_value(file_path, 'Connected')
            if connected == 'True':
                db.toggle_goal_color(index)
                write_value(file_path, 'Goal', db.goal_color)
                goal_colors.append(db.goal_color)  # Add the color to the list for each connected blimp

    emit('setGoalColor', goal_colors, broadcast=True)

@socketio.on('toggleTarget1Color')
def handle_toggle_target1_color():
    db.setClientIP(request.remote_addr)
    db.toggle_target1_color()
    emit('setTarget1Color', db.target1_color, broadcast=True)

@socketio.on('toggleTarget2Color')
def handle_toggle_target2_color():
    db.setClientIP(request.remote_addr)
    db.toggle_target2_color()
    emit('setTarget2Color', db.target2_color, broadcast=True)

# Put functions below in separate file potentially !!!

def read_value(filename, variable): 
        with open(filename, 'r') as file:
            for line in file.readlines():
                if line.startswith(variable):
                    _, val = line.strip().split(' = ', 1)
                    return val
        return None  # Return None if the variable was not found in the file

def read_value_in_all_files(directory, variable):
    values = []
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):  # only process text files
            value = read_value(os.path.join(directory, filename), variable)
            if value is not None:
                values.append(value)
    return values

def write_value(filename, variable, value):
        lines = []
        with open(filename, 'r') as file:
            lines = file.readlines()

        with open(filename, 'w') as file:
            for line in lines:
                if line.startswith(variable):
                    file.write(f'{variable} = {value}\n')
                else:
                    file.write(line)



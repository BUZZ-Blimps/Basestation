from webserver.packages import *

def send_blimp_names():
    with app.app_context():  # Push an application context
        while True:
            blimpNames = db.get_blimp_names()
            socketio.emit('updateBlimpNames', blimpNames, namespace='/')
            time.sleep(0.001)

@socketio.on('connect')
def handle_connect():
    socketio.start_background_task(send_blimp_names)
    db.getData()
    emit('setGoalColor', db.goal_color)
    emit('setTarget1Color', db.target1_color)
    emit('setTarget2Color', db.target2_color)

@socketio.on('toggleGoalColor')
def handle_toggle_goal_color():
    db.setClientIP(request.remote_addr)
    db.toggle_goal_color()
    emit('setGoalColor', db.goal_color, broadcast=True)

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

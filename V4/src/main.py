#!/usr/bin/env python3

from webserver.packages import *

def terminate(signal, frame):
    print('\nTerminating...\n')
    sys.exit(0)

def run_flask_app():
    # Terminate if Ctrl+C Caught
    signal.signal(signal.SIGINT, terminate)
    # Testing (Delete Later)
    db.add_blimp_name('BurnCreamBlimp')

    # Start the Flask webserver
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)

if __name__ == '__main__':
    run_flask_app()

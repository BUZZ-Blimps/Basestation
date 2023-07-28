from webserver.packages import *

@app.route('/')
def index():
    db.getData()
    return render_template('main.html', goal_color=db.goal_color, target1_color=db.target1_color, target2_color=db.target2_color)

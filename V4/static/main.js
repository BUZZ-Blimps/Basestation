// IP Address Allowed
const ip_allowed = '192.168.0.200';

// Connect to SocketIO server
const socket = io();

// Timeout
const TIMEOUT = 2000;

// Timers
const blimp_timers = {};

// Receive Connection
socket.on('connect', () => {
    // Client IP Address
    //let client_ip = "{{ client_ip }}";
    console.log('Client connected with IP:', client_ip);
});

socket.on('update', (blimp_dict) => {
    //console.log(blimp_dict)
    update_basestation(blimp_dict);
});

// Catching Blimps before Attack Blimps
var blimpOrder = ["Burn Cream Blimp", "Silly Ah Blimp", "Turbo Blimp", "Game Chamber Blimp", "Five Guys Blimp", "Catch 1", "Catch 2", "Attack 1", "Attack 2"];

var blimpList = [];

var sortedNameRows = [];

var sortedStateRows = {};

var blimpsTableBody = document.getElementById('blimpsTableBody');

var statesTableBody = document.getElementById('statesTableBody');

var goalButtonsContainer = document.querySelector('.goalButtonsContainer');

var targetButtonsContainer = document.querySelector('.targetButtonsContainer');

var leftStickX = 0;
var leftStickY = 0;
var rightStickX = 0;
var rightStickY = 0;

//Keeps track of which blimp the controller is connected to
var Controller_1_currConnection = -1;

function update_target_button_color(blimp_dict, target_color, target_color_button) {
    // Get data from blimp dictionary
    let blimp_name = blimp_dict["blimp_name"];
    
    // Target Color Button does not exist
    if (!target_color_button) {
        // Create new button
        target_color_button = document.createElement('button');
        target_color_button.id = `target_color_1_button_${blimp_name}`;
        target_color_button.style.backgroundColor = target_color;
        
        // Attach the click event listener to the button
        target_color_button.addEventListener('click', (event) => {
            // Client IP Address
            //let client_ip = client_ip;
            
            if (client_ip === ip_allowed) {
                // Change the target color and the data
                if (target_color === 'blue') {
                    target_color = 'red';
                    blimp_dict["target_color"] = 1;
                } else {
                    target_color = 'blue';
                    blimp_dict["target_color"] = 0;
                }
                
                console.log(blimp_name, 'has a target color:', target_color);

                // Get the target color button for the specific blimp
                let target_color_button = document.getElementById(`target_color_1_button_${blimp_name}`);
                
                // Set target color of the button
                target_color_button.style.backgroundColor = target_color;
                
                // Send the data to the backend to update over ROS
                socket.emit('update_target_color', blimp_dict);
            }
        });
        
        // Create a div element that will be used to wrap the button and force it to a new line
        const buttonWrapper = document.createElement('div');
        buttonWrapper.appendChild(target_color_button);
        buttonWrapper.setAttribute('blimp_name', blimp_name); // Store the blimp name
        
        // Center the button horizontally
        buttonWrapper.style.display = 'flex';
        buttonWrapper.style.justifyContent = 'center';

        targetButtonsContainer.appendChild(buttonWrapper);
    }
}

function update_target_button_colors(blimp_dict, target_color_1, target_color_1_button, target_color_2, target_color_2_button) {
  // Get data from blimp dictionary
  let blimp_name = blimp_dict["blimp_name"];

  // Create a row wrapper for the target color buttons
  let targetRowWrapper = document.createElement('div');
  targetRowWrapper.classList.add('target-row'); // You can style this class for better alignment

  // Target Color Button 1 does not exist
  if (!target_color_1_button) {
      target_color_1_button = document.createElement('button');
      target_color_1_button.id = `target_color_1_button_${blimp_name}`;
      target_color_1_button.style.backgroundColor = target_color_1;

      targetRowWrapper.appendChild(target_color_1_button);
  }

  // Target Color Button 2 does not exist
  if (!target_color_2_button) {
      target_color_2_button = document.createElement('button');
      target_color_2_button.id = `target_color_2_button_${blimp_name}`;
      target_color_2_button.style.backgroundColor = target_color_2;

      targetRowWrapper.appendChild(target_color_2_button);
  }
  targetRowWrapper.setAttribute('blimp_name', blimp_name); // Store the blimp name
  targetButtonsContainer.appendChild(targetRowWrapper);
}

function update_goal_button_color(blimp_dict, goal_color, goal_color_button) {
  // Get data from blimp dictionary
  let blimp_name = blimp_dict["blimp_name"];
  
  // Goal Color Button does not exist
  if (!goal_color_button) {
      // Create new button
      goal_color_button = document.createElement('button');
      goal_color_button.id = `goal_color_button_${blimp_name}`;
      goal_color_button.style.backgroundColor = goal_color;
      
      // Attach the click event listener to the button
      goal_color_button.addEventListener('click', (event) => {
        // Client IP Address
        //let client_ip = "{{ client_ip }}";

        if (client_ip === ip_allowed) {
          // Change the goal color and the data
          if (goal_color === 'orange') {
              goal_color = 'yellow';
              blimp_dict["goal_color"] = 1;
          } else {
              goal_color = 'orange';
              blimp_dict["goal_color"] = 0;
          }

          console.log(blimp_name, 'has a goal color:', goal_color)

          // Get the goal color button for the specific blimp
          let goal_color_button = document.getElementById(`goal_color_button_${blimp_name}`);

          // Set goal color of the button
          goal_color_button.style.backgroundColor = goal_color;

          // Send the data to the backend to update over ROS
          socket.emit('update_goal_color', blimp_dict);
        }
      });

      // Create a div element that will be used to wrap the button and force it to a new line
      const buttonWrapper = document.createElement('div');
      buttonWrapper.appendChild(goal_color_button);
      buttonWrapper.setAttribute('blimp_name', blimp_name); // Store the blimp name

      goalButtonsContainer.appendChild(buttonWrapper);

      // Sort the goal rows based on your desired order
      sortedGoalRows = Array.from(goalButtonsContainer.querySelectorAll('[blimp_name]'));
      sortedGoalRows.sort(function(a, b) {
          const blimpNameA = a.getAttribute('blimp_name');
          const blimpNameB = b.getAttribute('blimp_name');
          return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
      });

      goalButtonsContainer.innerHTML = ''; // Clear the container
      sortedGoalRows.forEach(function(row) {
          goalButtonsContainer.appendChild(row);
      });
    }
}

function update_empty_button_color(blimp_dict, goal_color, goal_color_button, goalButtonsContainer) {
  // Get data from blimp dictionary
  let blimp_name = blimp_dict["blimp_name"];
  
  // Goal Color Button does not exist
  if (!goal_color_button) {
      // Create new button
      goal_color_button = document.createElement('button');
      goal_color_button.id = `goal_color_button_${blimp_name}`;
      goal_color_button.style.backgroundColor = goal_color;

      // Create a div element that will be used to wrap the button and force it to a new line
      const buttonWrapper = document.createElement('div');
      buttonWrapper.appendChild(goal_color_button);
      buttonWrapper.setAttribute('blimp_name', blimp_name); // Store the blimp name

      // Error Currently !!!
      goalButtonsContainer.appendChild(buttonWrapper);

      // Sort the goal rows based on your desired order
      sortedGoalRows = Array.from(goalButtonsContainer.querySelectorAll('[blimp_name]'));
      sortedGoalRows.sort(function(a, b) {
          const blimpNameA = a.getAttribute('blimp_name');
          const blimpNameB = b.getAttribute('blimp_name');
          return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
      });

      goalButtonsContainer.innerHTML = ''; // Clear the container
      sortedGoalRows.forEach(function(row) {
          goalButtonsContainer.appendChild(row);
      });
    }
}

function get_state(number) {
  let state;

  if (number === 0) {
      state = "searching";
  } else if (number === 1) {
      state = "approach";
  } else if (number === 2) {
      state = "catching";
  } else if (number === 3) {
      state = "caught";
  } else if (number === 4) {
      state = "goalSearch";
  } else if (number === 5) {
      state = "approachGoal";
  } else if (number === 6) {
      state = "scoringStart";
  } else if (number === 7) {
      state = "shooting";
  } else if (number === 8) {
      state = "scored";
  } else {
      state = "error"; // Default state if the number doesn't match any of the provided values
  }

  return state;
}

var keys = { w: false, a: false, s: false, d: false };
var dot = document.getElementById("dot1");

window.onkeydown = function(e) {
    if (keys.hasOwnProperty(e.key.toLowerCase())) {
        keys[e.key.toLowerCase()] = true;
    }
    moveDot();
};

window.onkeyup = function(e) {
    if (keys.hasOwnProperty(e.key.toLowerCase())) {
        keys[e.key.toLowerCase()] = false;
    }
    moveDot();
};

function moveDot() {
    dot.style.left = "50%";
    dot.style.top = "50%";

    if (keys['w'] && keys['a'] && keys['s'] && keys['d']) return;

    if (keys['w'] && keys['s']) {
        if (keys['a']) dot.style.left = "25%";
        else if (keys['d']) dot.style.left = "75%";
        return;
    }

    if (keys['a'] && keys['d']) {
        if (keys['w']) dot.style.top = "25%";
        else if (keys['s']) dot.style.top = "75%";
        return;
    }

    if (keys['w'] && !keys['s']) dot.style.top = "25%";
    if (!keys['w'] && keys['s']) dot.style.top = "75%";
    if (keys['a'] && !keys['d']) dot.style.left = "25%";
    if (!keys['a'] && keys['d']) dot.style.left = "75%";
}

var dot1 = document.getElementById('dot1');
var dot2 = document.getElementById('dot2');

var controllerCheck = setInterval(pollController, 0);

function pollController() {
  var gamepads = navigator.getGamepads();
  for (var i = 0; i < gamepads.length; i++) {
    var gamepad = gamepads[i];
    if (gamepad) {
      moveDots(gamepad);
      handleGamepadButtons(gamepad);
    }
  }
}

function moveDots(gamepad) {
  // Gamepad.axes array contains the values for all axes of the controller.
  // It's typical for the left stick to use axes 0 (X) and 1 (Y)
  // and for the right stick to use axes 2 (X) and 3 (Y).
  leftStickX = gamepad.axes[0];
  leftStickY = gamepad.axes[1];
  rightStickX = gamepad.axes[2];
  rightStickY = gamepad.axes[3];

  // Define a dead zone threshold (adjust as needed)
  const deadZero = 0.1;
  const deadOne = 0.01;

  // Apply tolerance by checking if values are within the dead zone
  leftStickX = Math.abs(leftStickX) < deadZero ? 0 : leftStickX;
  leftStickY = Math.abs(leftStickY) < deadZero ? 0 : leftStickY;
  rightStickX = Math.abs(rightStickX) < deadZero ? 0 : rightStickX;
  rightStickY = Math.abs(rightStickY) < deadZero ? 0 : rightStickY;

  leftStickX = leftStickX > 1 - deadOne ? 1 : leftStickX;
  leftStickY = leftStickY > 1 - deadOne ? 1 : leftStickY;
  rightStickX = rightStickX > 1 - deadOne ? 1 : rightStickX;
  rightStickY = rightStickY > 1 - deadOne ? 1 : rightStickY;

  leftStickX = leftStickX < -1 + deadOne ? -1 : leftStickX;
  leftStickY = leftStickY < -1 + deadOne ? -1 : leftStickY;
  rightStickX = rightStickX < -1 + deadOne ? -1 : rightStickX;
  rightStickY = rightStickY < -1 + deadOne ? -1 : rightStickY;

  leftStickX = leftStickX.toFixed(2);
  leftStickY = leftStickY.toFixed(2);
  rightStickX = rightStickX.toFixed(2);
  rightStickY = rightStickY.toFixed(2);

  // Multiply by -1 to Invert the Y-Axes
  leftStickY = leftStickY * -1;
  rightStickY = rightStickY * -1;

  // Apply the joystick positions to the dot positions
  // The joystick returns a value between -1 and 1.
  // We shift this range to be 0 - 100% for use with the CSS styles.
  dot3.style.left = `${50 + leftStickX * 45}%`;
  dot3.style.top = `${50 + leftStickY * -45}%`;
  dot4.style.left = `${50 + rightStickX * 45}%`;
  dot4.style.top = `${50 + rightStickY * -45}%`;

  //console.log("Left Stick X: ", 50 + leftStickX * 45);
}

// Initialize a variable to keep track of the previous state of the controller buttons and right trigger
let controllerState = {
  up: false,
  down: false,
  left: false,
  right: false,
  rightTrigger: false,
  leftTrigger: false,
  rightBumper: false,
  leftBumper: false,
  xButton: false,
  yButton: false,
  bButton: false,
  aButton: false
};

// Function to handle gamepad button presses and releases
function handleGamepadButtons(gamepad) {
  // Check if the D-pad Up button was pressed in the previous state but is not pressed now (released)
  if (controllerState.up && !gamepad.buttons[12].pressed) {
    console.log('Xbox D-Pad Up released.');
  }
  // Check if the D-pad Down button was pressed in the previous state but is not pressed now (released)
  if (controllerState.down && !gamepad.buttons[13].pressed) {
    console.log('Xbox D-Pad Down released.');
  }
  // Check if the D-pad Left button was pressed in the previous state but is not pressed now (released)
  if (controllerState.left && !gamepad.buttons[14].pressed) {
    console.log('Xbox D-Pad Left released.');
  }
  // Check if the D-pad Right button was pressed in the previous state but is not pressed now (released)
  if (controllerState.right && !gamepad.buttons[15].pressed) {
    console.log('Xbox D-Pad Right released.');
  }
  
  // Check if the right trigger was pressed in the previous state but is not pressed now (released)
  if (controllerState.rightTrigger && gamepad.buttons[7].value === 0) {
    console.log('Xbox Right Trigger released.');
  }

  // Check if the left trigger was pressed in the previous state but is not pressed now (released)
  if (controllerState.leftTrigger && gamepad.buttons[6].value === 0) {
    console.log('Xbox Left Trigger released.');
  }

  // Check if the right bumper was pressed in the previous state but is not pressed now (released)
  if (controllerState.rightBumper && !gamepad.buttons[5].pressed) {
    console.log('Xbox Right Bumper released.');
  }

  // Check if the left bumper was pressed in the previous state but is not pressed now (released)
  if (controllerState.leftBumper && !gamepad.buttons[4].pressed) {
    console.log('Xbox Left Bumper released.');
  }

  // Check if the X button was pressed in the previous state but is not pressed now (released)
  if (controllerState.xButton && !gamepad.buttons[2].pressed) {
    console.log('Xbox X Button released.');
  }

  // Check if the Y button was pressed in the previous state but is not pressed now (released)
  if (controllerState.yButton && !gamepad.buttons[3].pressed) {
    console.log('Xbox Y Button released.');
  }

  // Check if the B button was pressed in the previous state but is not pressed now (released)
  if (controllerState.bButton && !gamepad.buttons[1].pressed) {
    console.log('Xbox B Button released.');
  }

  // Check if the A button was pressed in the previous state but is not pressed now (released)
  if (controllerState.aButton && !gamepad.buttons[0].pressed) {
    console.log('Xbox A Button released.');
  }

  // Update the previous state of the controller buttons and right trigger
  controllerState = {
    up: gamepad.buttons[12].pressed,
    down: gamepad.buttons[13].pressed,
    left: gamepad.buttons[14].pressed,
    right: gamepad.buttons[15].pressed,
    rightTrigger: gamepad.buttons[7].value !== 0,
    leftTrigger: gamepad.buttons[6].value !== 0,
    rightBumper: gamepad.buttons[5].pressed,
    leftBumper: gamepad.buttons[4].pressed,
    xButton: gamepad.buttons[2].pressed,
    yButton: gamepad.buttons[3].pressed,
    bButton: gamepad.buttons[1].pressed,
    aButton: gamepad.buttons[0].pressed
  };

}

function update_basestation(blimp_dict) {
  // Get data from blimp dictionary
  let blimp_name = blimp_dict["blimp_name"];

  // Clear the timers
  clearTimeout(blimp_timers[blimp_name]);

  // Get goal color from the data
  let goal_color;
  if (blimp_dict["goal_color"] === 0) {
      goal_color = 'orange';
  } else {
      goal_color = 'yellow';
  }

  // Get the goal color button for the specific blimp
  let goal_color_button = document.getElementById(`goal_color_button_${blimp_name}`);

  // Number of targets
  let target_num;

  // Target Buttons
  let target_color_1_button;
  let target_color_2_button;

  // Target Color (Attack Blimps)
  let target_color;

  // State Machine
  let state;

  // Get blimp type from the data
  let blimp_type = blimp_dict["blimp_type"];
  if (blimp_type === 0) {
    target_num = 2;
    target_color_1_button = document.getElementById(`target_color_1_button_${blimp_name}`);
    target_color_2_button = document.getElementById(`target_color_2_button_${blimp_name}`);
  }
  else {
    target_num = 1;
    target_color_1_button = document.getElementById(`target_color_1_button_${blimp_name}`);
    if (blimp_dict["target_color"] === 0) {
      target_color = 'blue';
    } else {
      target_color = 'red';
    }    
  }

  // Only proceed if the name is not in the table
  if (!(blimpList.includes(blimp_name))) {
    // Create a new row and cell for the blimp name
    var newRow = document.createElement('h3');
    var newCell = document.createElement('h3');
    newCell.textContent = blimp_name;
    newRow.appendChild(newCell);
    blimpsTableBody.appendChild(newRow);

    // Store the row in the blimpList
    blimpList.push(blimp_name);

    // Sort the name rows based on your desired order
    sortedNameRows = Array.from(blimpsTableBody.getElementsByTagName('h3'));
    sortedNameRows.sort(function(a, b) {
        return blimpOrder.indexOf(a.textContent) - blimpOrder.indexOf(b.textContent);
    });
    blimpsTableBody.innerHTML = ''; // Clear the table
    sortedNameRows.forEach(function(row) {
      blimpsTableBody.appendChild(row);
    });

    // State Machine
    state = get_state(blimp_dict['state_machine']);

    // Clear the statesTableBody
    statesTableBody.innerHTML = '';

    // Create a new row and cell for the blimp state
    var newRow = document.createElement('h3');
    var newCell = document.createElement('h3');
    newCell.textContent = state; // Get the state using blimp_name
    newRow.appendChild(newCell);
    statesTableBody.appendChild(newRow);

    // Store the state in sortedStateRows using blimp_name as the key
    sortedStateRows[blimp_name] = newRow;

    // Sort the state rows based on the blimp names and their states
    var sortedRows = Object.keys(sortedStateRows).sort(function (a, b) {
        return blimpOrder.indexOf(a) - blimpOrder.indexOf(b);
    });

    // Clear the existing content of statesTableBody
    statesTableBody.innerHTML = '';

    // Append the sorted rows to statesTableBody
    sortedRows.forEach(function (rowKey) {
        statesTableBody.appendChild(sortedStateRows[rowKey]);
    });

    if (target_num === 2) {
      update_target_button_colors(blimp_dict, 'green', target_color_1_button, 'purple', target_color_2_button);
      update_goal_button_color(blimp_dict, goal_color, goal_color_button);
    }
    // Target number is 1 (Attack Blimps)
    else {
      update_target_button_color(blimp_dict, target_color, target_color_1_button);
      update_empty_button_color(blimp_dict, 'white', goal_color_button);
    }
    // Sort the target rows based on your desired order
    sortedTargetRows = Array.from(targetButtonsContainer.querySelectorAll('[blimp_name]'));
    sortedTargetRows.sort(function(a, b) {
        const blimpNameA = a.getAttribute('blimp_name');
        const blimpNameB = b.getAttribute('blimp_name');
        return blimpOrder.indexOf(blimpNameA) - blimpOrder.indexOf(blimpNameB);
    });
    
    targetButtonsContainer.innerHTML = ''; // Clear the container
    sortedTargetRows.forEach(function(row) {
        targetButtonsContainer.appendChild(row);
    });
  }
  else {

    //If up or down is pressed, then check if a connection had been established
    //WARNING: This logic only works with one blimp
    //Gamepad not working
    
    if(controllerState.up || controllerState.down)
    {
      //If not, establish connection with first value in blimp list
      if(Controller_1_currConnection === -1)
      {
        blimp_dict["connected"] = true;
        socket.emit('update_connection', blimp_dict);
        Controller_1_currConnection = 0;
      }
    }

    if(controllerState.left) // && !gamepad.buttons[14].pressed)
    {
      console.log("Disconnecting " + blimp_name);
      blimp_dict["connected"] = false;
      socket.emit('update_connection', blimp_dict);
      Controller_1_currConnection = -1;
    }

    // Set goal color of the button for all clients to see
    if (blimp_type === 0) {
      if (goal_color_button) {
        goal_color_button.style.backgroundColor = goal_color;
      }
    }
    else {
      if (goal_color_button) {
        goal_color_button.style.backgroundColor = 'white';
        goal_color_button.style.border = "1px solid white";
      }
    }
    // Set target color of the button for all clients to see
    if (target_color_1_button) {
      target_color_1_button.style.backgroundColor = target_color;
    }

    //Verify if controller is connected to this blimp
    if(blimp_dict["connected"])
    {
      console.log(blimp_name + " is connected");
      sortedStateRows[blimp_name].style.color = 'blue';
    }
    else
    {
      console.log(blimp_name + " Not working");      
      sortedStateRows[blimp_name].style.color = 'black';
    }

    // Update the state row's textContent for all clients to see
    state = get_state(blimp_dict['state_machine']);
    if (sortedStateRows[blimp_name]) {
      sortedStateRows[blimp_name].textContent = state;
    }
  }
  
  // Convert the values to Float64
  var motorCommands = new Float64Array([leftStickX, leftStickY, rightStickX, rightStickY]);
  //console.log(motorCommands);

  // Convert the Float64Array to binary data
  const binaryData = new Uint8Array(motorCommands.buffer)
  // Add UI for only connected blimp eventually
  socket.emit('update_motorCommands', binaryData);

  // Set a new timer for this blimp name
  blimp_timers[blimp_name] = setTimeout(() => {
      // Check if the blimp name is in the list
      if (blimpList.includes(blimp_name)) {
        sortedNameRows.forEach(function(row) {
          if (row.textContent === blimp_name) {
              row.textContent = ''; // Clear the content of the cell
          }
        });

        blimpList = blimpList.filter(item => item !== blimp_name);

        // Remove the state row from the map if the key matches blimp_name
        Object.entries(sortedStateRows).forEach(([key, value]) => {
          if (key === blimp_name) {
            value.textContent = ''; // Set the value to ''
          }
        });

        let goal_color_button = document.getElementById(`goal_color_button_${blimp_name}`);
        if (goal_color_button) {
          goal_color_button.remove();
        }

        let target_color_1_button = document.getElementById(`target_color_1_button_${blimp_name}`);
        let target_color_2_button = document.getElementById(`target_color_2_button_${blimp_name}`);
        if (target_color_1_button) {
          target_color_1_button.remove();
        }
        if (target_color_2_button) {
          target_color_2_button.remove();
        }
      }

      // Clear the timer entry
      delete blimp_timers[blimp_name];

  }, TIMEOUT);
}
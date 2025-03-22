let ros;
let cmdVel;

function startConnection() {
  ros = new ROSLIB.Ros({
    // url: 'ws://localhost:9090'
    url: 'ws://192.168.1.141:9090'
  });

  ros.on('connection', () => {
    console.log('Connected to ROS Bridge');
    cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
  });

  ros.on('error', (error) => {
    console.error('Error connecting to ROS: ', error);
    alert('Error connecting to ROS: 9090', error);
  });

  ros.on('close', () => {
    console.log('Disconnected from ROS');
    alert('Disconnected from ROS');
  });
}

function send(direction) {
  if (!ros) {
    console.error('ROS connection not established');
    alert('ROS connection not established');
    return;
  }

  if (!cmdVel) {
    console.error('Publisher not created');
    alert('Publisher not created');
    return;
  }

  const twist = new ROSLIB.Message({
    linear: {
      x: direction.linear.x,
      y: direction.linear.y,
      z: direction.linear.z
    },
    angular: {
      x: direction.angular.x,
      y: direction.angular.y,
      z: direction.angular.z
    }
  });

  cmdVel.publish(twist);
}

function move(linear, angular) {
  const movemsg = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular }
  };
  send(movemsg);
}

startConnection(); // 初始化ROS连接
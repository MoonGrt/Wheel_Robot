var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        ws_address: 'ws://192.168.1.141:9090',
        logs: [],
        loading: false,
        topic: null,
        message: null,
    },
    // helper methods to connect to ROS
    methods: {
        connect: function () {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toLocaleTimeString('en-GB', { hour12: false }) + ' - Connected!')
                this.connected = true
                this.loading = false
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toLocaleTimeString('en-GB', { hour12: false }) + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toLocaleTimeString('en-GB', { hour12: false }) + ' - Disconnected!')
                this.connected = false
                this.loading = false
            })
        },
        disconnect: function () {
            this.ros.close()
        },
        setTopic: function () {
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                compression: "cbor",
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
        },
        move: function (linear, angular) {
            this.message = new ROSLIB.Message({
                linear: { x: linear, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: angular, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        forward: function () {
            this.move(0.5, 0)
        },
        stop: function () {
            this.move(0, 0)
        },
        backward: function () {
            this.move(-0.5, 0)
        },
        turnLeft: function () {
            this.move(0, 0.2)
        },
        turnRight: function () {
            this.move(0, -0.2)
        },
    },
    mounted() {
    },
})
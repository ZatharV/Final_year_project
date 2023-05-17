const http = require('http')
const express = require('express')
const os = require('os-utils')
const socketio = require('socket.io')
const mqtt = require('mqtt')


const app = express()
const httpServer = http.createServer(app)

const url = "mqtt://broker.hivemq.com:1883"

const client = mqtt.connect(url)

client.on('connect',()=>{
    console.log("hello, mqtt")
    
    client.subscribe('topic/temperature',()=>{
        console.log(`topic name: /topic/temperature`)    
    })

    client.subscribe('topic/humidity',()=>{
        console.log(`topic name: /topic/humidity`)    
    })
    // client.subscribe('topic/battery',()=>{
    //     console.log(`topic name: /topic/battery`)    
    // })
    // client.subscribe('topic/light',()=>{
    //     console.log(`topic name: /topic/light`)    
    // })
    // client.subscribe('topic/lpg',()=>{
    //     console.log(`topic name: /topic/lpg`)    
    // })
    // client.subscribe('topic/co',()=>{
    //     console.log(`topic name: /topic/co`)    
    // })
    // client.subscribe('topic/smoke',()=>{
    //     console.log(`topic name: /topic/smoke`)    
    // })
    // client.subscribe('topic/motor',()=>{
    //     console.log(`topic name: /topic/motor`)    
    // })
})

const io = new socketio.Server(httpServer, {
    transports:['websocket','polling']
});

io.on("connection", socket => {
    
    console.log("hello, socket")
    tick = 0
   setInterval(()=>{
    // os.cpuUsage((cpuPercentage)=>{

    //     socket.emit('cpu',{
    //         name: tick++,
    //         value:cpuPercentage,
    //     });
    // })
    // os.cpuFree((free)=>{
    //    socket.emit('free',{
    //         name: tick++,
    //         value:free
    //     })
    // })
    client.on('message',(t,p)=>{
        if(t === 'topic/temperature'){
            socket.emit('temp',{
                name: tick++,
                value: parseFloat(p)
            })
        }
        if(t === 'topic/humidity'){
            socket.emit('humid',{
                name: tick++,
                value: parseFloat(p)
            })
        }
        // if(t === '/topic/battery'){
        //     socket.emit('bat',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
        // if(t === '/topic/light'){
        //     socket.emit('light',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
        // if(t === '/topic/lpg'){
        //     socket.emit('lpg',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
        // if(t === '/topic/co'){
        //     socket.emit('co',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
        
        // if(t === '/topic/smoke'){
        //     socket.emit('smoke',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
        // if(t === '/topic/motor'){
        //     socket.emit('mot',{
        //         name: tick++,
        //         value: parseFloat(p)
        //     })
        // }
    })
    },1000)
})


httpServer.listen(4000);

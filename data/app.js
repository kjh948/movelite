const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const ws = new WebSocket(`ws://${window.location.hostname}/ws`);

let robotState = {x:0, y:0, theta:0, mode:0};
let mapData = null;

const W = 100, H = 100, RES = 0.1;
const PX_PER_CELL = canvas.width / W;

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === 'state') {
        robotState = data;
        document.getElementById('valX').innerText = data.x.toFixed(2);
        document.getElementById('valY').innerText = data.y.toFixed(2);
        document.getElementById('valT').innerText = data.theta.toFixed(2);
        const modes = ["IDLE", "MAPPING", "MAP_READY", "NAVIGATING", "REPLANNING", "ARRIVED", "ERROR"];
        document.getElementById('mode').innerText = modes[data.mode] || "UNKNOWN";
        draw();
    } else if (data.type === 'map') {
        // Base64 decode
        const bin = atob(data.data);
        mapData = new Uint8Array(bin.length);
        for(let i=0; i<bin.length; i++) mapData[i] = bin.charCodeAt(i);
        draw();
    }
};

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw map
    if (mapData) {
        for(let y=0; y<H; y++) {
            for(let x=0; x<W; x++) {
                const val = mapData[y*W + x];
                if (val === 0) ctx.fillStyle = '#111'; // unknown
                else if (val === 255) ctx.fillStyle = '#f00'; // occupied
                else ctx.fillStyle = '#fff'; // free
                
                ctx.fillRect(x*PX_PER_CELL, y*PX_PER_CELL, PX_PER_CELL, PX_PER_CELL);
            }
        }
    }

    // Draw Robot
    const rx = (robotState.x / RES) + W/2;
    const ry = (robotState.y / RES) + H/2;
    
    ctx.save();
    ctx.translate(rx * PX_PER_CELL, ry * PX_PER_CELL);
    ctx.rotate(robotState.theta);
    
    ctx.fillStyle = '#0f0';
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, Math.PI*2);
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(10, 0);
    ctx.stroke();
    
    ctx.restore();
}

canvas.onclick = (e) => {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const gx = Math.floor(x / PX_PER_CELL);
    const gy = Math.floor(y / PX_PER_CELL);
    ws.send(JSON.stringify({type: 'goal', gx: gx, gy: gy}));
};

function sendCmd(action) {
    ws.send(JSON.stringify({type: 'cmd', action: action}));
}

// Joystick logic
const joy = document.getElementById('joystick');
const stick = document.getElementById('stick');
let joyActive = false;

joy.onmousedown = joy.ontouchstart = (e) => { joyActive = true; updateJoy(e); };
document.onmouseup = document.ontouchend = () => { 
    if(!joyActive) return;
    joyActive = false; 
    stick.style.transform = `translate(0px, 0px)`;
    ws.send(JSON.stringify({type: 'joystick', v: 0, omega: 0}));
};
document.onmousemove = document.ontouchmove = (e) => { if(joyActive) updateJoy(e); };

function updateJoy(e) {
    const rect = joy.getBoundingClientRect();
    const cx = rect.left + rect.width/2;
    const cy = rect.top + rect.height/2;
    const clientX = e.touches ? e.touches[0].clientX : e.clientX;
    const clientY = e.touches ? e.touches[0].clientY : e.clientY;
    
    let dx = clientX - cx;
    let dy = clientY - cy;
    const maxR = rect.width/2 - 20;
    
    const r = Math.min(maxR, Math.sqrt(dx*dx + dy*dy));
    const angle = Math.atan2(dy, dx);
    
    dx = r * Math.cos(angle);
    dy = r * Math.sin(angle);
    
    stick.style.transform = `translate(${dx}px, ${dy}px)`;
    
    // Convert to v, omega (max v=0.3m/s, max w=1.5rad/s)
    const v = -(dy / maxR) * 0.3;
    const omega = -(dx / maxR) * 1.5;
    
    ws.send(JSON.stringify({type: 'joystick', v: v, omega: omega}));
}
